print('Importing dependencies...', end='')
import sys
sys.stdout.flush()
sys.path.append('/home/pi/.local/lib/python3.7/site-packages')

from time import sleep

import numpy as np
import board
import neopixel

import scara_arm as arm
import scara_imaging as imaging
print('done.')


def read_config(filename):
    ''' Read config file to dict, containing servo pulse ranges and 
        image warp matrix.
    Inputs:
        filename: name (and path, if necessary) of config file (str)
    Returns: config dict (keys = config names, 
                          vals = config values (as strings))
    '''
    config = {}
    with open(filename) as config_file:
        for line in config_file:
            try:
                (key, val) = line.split(': ')
                config[key] = val
            except ValueError:
                pass
    # Reformat config entries from strings to their proper types
    warp_matrix = config['Image warp matrix']
    warp_matrix = warp_matrix.strip('[ ]')
    warp_matrix = warp_matrix.replace('] [', '; ')
    config['Image warp matrix'] = np.array(np.mat(warp_matrix))
    config['Shoulder min pulse'] = int(config['Shoulder min pulse'])
    config['Shoulder max pulse'] = int(config['Shoulder max pulse'])
    config['Elbow min pulse'] = int(config['Elbow min pulse'])
    config['Elbow max pulse'] = int(config['Elbow max pulse'])
    
    return config


def get_menu_input(commands, input_text=''):
    ''' Gets and validates user input for menu selections
    Inputs:
        commands (dict): valid commands.
            keys (str): commands
            values (str): modes
        input_text (str, optional): text to print pre-input. Defaults to empty.
    Returns: validated user input (str)
    '''
    while True:
        user_input = input(input_text)
        if user_input == 'ls':
            print('Valid commands:')
            for key in commands:
                padded_key = key.ljust(14, '.')
                print(f'   {padded_key}{commands[key]}')
        elif user_input not in commands.keys():
            print('Invalid command. ', end='')
        else:
            return user_input


def angle_entry(shoulder, elbow):
    ''' Direct angle entry
    Inputs: 
        shoulder, elbow: joint objects for interacting directly with joints
    Returns: None
    '''
    commands = {'s': 'Shoulder', 'e': 'Elbow', 'x': 'Exit'}
    print(
        '\n*****ANGLE MODE*****\n'
        'Enter angle, or "j" to select joint.\n'
        '   Shoulder joint range: ±90° \n   Elbow joint range: ±158°\n'
        'Enter "j" to select joint. Enter "x" to exit at any time.\n'
        )
    angles = [None, None]
    while True:
        joint = get_menu_input(commands, 'Select joint ("ls" for options): ')
        if joint == 'x':
            print('Exiting to menu.\n')
            return
        else:
            print(f'{commands[joint]} joint selected.')
            while True:
                user_input = input('Enter angle: ')
                if user_input == 'x':
                    print('Exiting to menu.\n')
                    return
                elif user_input == 'j':
                    break
                try:
                    if joint == 's':
                        angles[0] = float(user_input)
                    elif joint == 'e':
                        angles[1] = float(user_input)
                    shoulder.set_angle(angles[0])
                    elbow.set_angle(angles[1])
                except (ValueError, TypeError):
                    print('Invalid input (Double-check valid angles).')


def coordinate_entry(shoulder, elbow, arm_lengths):
    ''' Direct coordinate entry
    Inputs:
        kit (ServoKit object): arm servo ServoKit
        a (float): upper arm length (inches)
        b (float): lower arm length (inches)
    Returns: None 
    '''
    def get_coord():
        ''' Generates coordinate pair based on user input.
        Inputs: none
        Returns: either list containing user-inputted coordinates,
        in format (x, y), or None
        '''
        while True:    
            user_input = input('Enter coordinates: ')
            if user_input == 'x':
                return None, None
            else:
                try:
                    user_input = user_input.split(',')
                    user_input = [float(i) for i in user_input]
                    return user_input
                except ValueError:
                    print('Invalid input.')

    print(
        '\n*****COORDINATE MODE***** \nInput coordinates in format "x, y".'
        '\n   x range: 0 ≤ x ≤ 10.5\n   y range: ±10.5'
        '\nEnter "x" to exit at any time.\n'
        )
    angles0 = [0, 0]
    while True:
        x, y = get_coord()
        if x is None:
            print('Exiting to menu.\n')
            return
        # Attempt first solution, then attempt second if first is invalid
        try:
            angles = arm.find_angles(x, y, arm_lengths)
            if (abs(angles[0][0]-angles0[0])
                > abs(angles[1][0]-angles0[0])):
                # Order IK solutions to prefer less shoulder movement
                angles[0], angles[1] = angles[1], angles[0]
            shoulder.set_angle(angles[0][0])
            elbow.set_angle(angles[0][1])
            angles0 = angles[0]
        except ValueError:
            try:
                shoulder.set_angle(angles[1][0])
                elbow.set_angle(angles[1][1])
                angles0 = angles[1]
            except:
                print('Input error. Ensure coordinate is reachable.')


def target_finding(camera, warp_matrix, target_color, maps):
    ''' Finds target disk
    Inputs:
        camera: PiCamera camera object
        warp_matrix (numpy array): transformation matrix defining image warp
        target_color (tup): 2-element tuple housing numpy arrays. First element
            houses minimum HSV values of target, second houses maximum values.
    Returns: None
    '''
    print(
        '\n*****TARGET-FINDING MODE*****\n'
        'Place target disk within camera view, then press "enter" '
        'to get coordinates. Type "x" to exit at any time.'
        )
    while True:
        is_ready = input(': ')
        if not is_ready:
            x, y = imaging.find_target(
                camera, warp_matrix, target_color, maps
                )
            print(f'Target coordinates: ({x}, {y})')
        else:
            return


def auto_operation(camera, warp_matrix, target_color, arm_lengths,
                   shoulder, elbow, maps):
    ''' Autonomous operation for SCARA arm. On loop, gets an image, detects if
        there is a target present, and moves the arm to its location.
    Inputs:
        camera: PiCamera camera object.
        warp_matrix (numpy array): transformation matrix defining image warp
        target_color (tup): 2-element tuple housing numpy arrays. First element
            houses minimum HSV values of target, second houses maximum values.
        a, b (floats): upper and lower arm lengths (in inches)
        kit: ServoKit object housing arm servos
        maps (tuple of arrays): fisheye correction arrays for remapping
    Returns: None
    '''
    print(
        '\n*****AUTONOMOUS OPERATION*****\n'
        'Place target disk within camera view.\n'
        'Press ctrl-C to exit at any time.'
        )
    x = 0
    y = 0
    x0 = 0
    y0 = 0
    angles0 = [0, 0]
    while True:
        try:
            sleep(0.2)
            #input('Press enter when ready.')
            try:
                x, y = imaging.find_target(
                    camera, warp_matrix, target_color, maps
                    )
            except ZeroDivisionError:
                pass
            if x is None:
                pass
            elif abs(x-x0) >= 0.1 and abs(y-y0) >= 0.1:
                x0 = x
                y0 = y
                print(f'Target coordinates: ({x}, {y})')
                try:
                    angles = arm.find_angles(x, y, arm_lengths)
                    if (abs(angles[0][0]-angles0[0])
                        > abs(angles[1][0]-angles0[0])):
                        # Order IK solutions to prefer less shoulder movement
                        angles[0], angles[1] = angles[1], angles[0]
                    shoulder.set_angle(angles[0][0])
                    elbow.set_angle(angles[0][1])
                    angles0 = angles[0]
                except ValueError:
                    try:
                        shoulder.set_angle(angles[1][0])
                        elbow.set_angle(angles[1][1])
                        angles0 = angles[1]
                    except:
                        print('Error. Ensure coordinate is reachable.')
        except KeyboardInterrupt:
            shoulder.set_angle(None)
            elbow.set_angle(None)
            return


def get_color():
    ''' Get user-input RGBW color tetrad, and return as tuple
    Inputs:
        None
    Returns: 
        color (tup): tuple housing color data in form (R, G, B, W)
    '''
    while True:
        try:
            color = input('Enter color, in format (r, g, b, w): ')
            color = color.strip('()')
            color = color.split(',')
            color = [int(i) for i in color]
            color = tuple(color)
            return color
        except ValueError:
            print('Invalid input')


print('Loading configuration........',end='')
sys.stdout.flush()
config = read_config('scara_config.txt')
warp_matrix = config['Image warp matrix']
# HSV bounds on target color to detect (lower bound, upper bound).
target_color = (np.array([1, 125, 50]), np.array([14, 255, 200]))
# Arm lengths in inches (shoulder, elbow)
arm_lengths = (6.0, 4.5)
print('done.')

print('Initializing........',end='')
sys.stdout.flush()
i2c, shoulder, elbow = arm.arm_init(config)
# Initialize ringlight and turn all LEDs on white
pixels = neopixel.NeoPixel(board.D18, 12, bpp=4)
rl_color = (255, 255, 255, 255)
pixels.fill(rl_color)
camera, maps = imaging.camera_init()
print('done.')

commands = {'main': {
                    'arm': 'Arm control','auto': 'Autonomous mode',
                    'cam': 'Camera control', 'light': 'Ringlight control',
                    'x': 'Exit'
                    },
            'arm': {'ang': 'Angle entry', 'cal': 'Arm calibration',
                    'coord': 'Coordinate entry', 'k': 'Kill motors',
                    'x': 'Exit'
                    },
            'auto': None,
            'cam': {'cal': 'Camera calibration', 'img': 'Capture image',
                    'targ': 'Locate target', 'x': 'Exit'
                    },
            'light': {'on': 'Turn on', 'off': 'Turn off',
                      'col': 'Change color', 'x': 'Exit'
                      }
            }

print('\n*****WELCOME TO SCARA BOT 1.0*****\n')
print('***MAIN MENU***')
while True:
    mode = get_menu_input(commands['main'],
        'What would you like to do? ("ls" for valid commands): '
        )
    if mode == 'arm':
        while True:
            print('\n***MANUAL ARM CONTROL***')
            mode = get_menu_input(commands['arm'],
                'What would you like to do? ("ls" for valid commands): '
                )
            if mode == 'ang':
                angle_entry(shoulder, elbow)
            elif mode == 'cal':
                config = arm.servo_calibration(shoulder, elbow, config)
            elif mode == 'coord':
                coordinate_entry(shoulder, elbow, arm_lengths)
            elif mode == 'k':
                print('Killing motor power........',end='')
                shoulder.set_angle(None)
                elbow.set_angle(None)
                print('done.\n')
            elif mode == 'x':
                print('Returning to main menu.\n\n***MAIN MENU***')
                break
    elif mode == 'auto':
        auto_operation(
            camera, warp_matrix, target_color,
            arm_lengths, shoulder, elbow, maps
            )
        print('Autonomous operation ended. Returning to main menu.'
              '\n\n***MAIN MENU***'
              )
    elif mode == 'cam':
        while True:
            print('\n***CAMERA CONTROL***')
            mode = get_menu_input(commands['cam'],
                'What would you like to do? ("ls" for valid commands): '
                )
            if mode == 'cal':
                warp_matrix = imaging.calibrate_camera(
                    camera, target_color, maps, shoulder, elbow
                    )
            elif mode == 'img':
                print('Image Capture Mode')
                imaging.image_capture(camera)
            elif mode == 'targ':
                target_finding(
                    camera, warp_matrix, target_color, maps
                    )
            elif mode == 'x':
                print('Returning to main menu.\n\n***MAIN MENU***')
                break
    elif mode == 'light':
        while True:
            print('\n***RINGLIGHT CONTROL***')
            mode = get_menu_input(commands['light'],
                'What would you like to do? ("ls" for valid commands): '
                )
            if mode == 'on':
                pixels.fill(rl_color)
            elif mode == 'off':
                pixels.fill((0, 0, 0, 0))
            elif mode == 'col':
                rl_color = get_color()
                pixels.fill(rl_color)
            elif mode == 'x':
                print('Returning to main menu.\n\n***MAIN MENU***')
                break
    elif mode == 'x':
        print('Shutting down........',end='')
        arm.arm_deinit(i2c, shoulder, elbow)
        pixels.deinit()
        camera.close()
        print('Goodbye.\n')
        sys.exit()
    else:
        print('Invalid command. ', end='')