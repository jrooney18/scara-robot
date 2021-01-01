import math

import board
import busio
import adafruit_pca9685
from adafruit_servokit import ServoKit


class joint():
    def __init__(self,
                 kit,
                 name,
                 num,
                 joint_range, 
                 servo_range,
                 min_pulse,
                 max_pulse,
                 flipped=False
                 ):
        ''' Create a SCARA joint object. 
        Attributes:
            kit: ServoKit object allowing direct interfacing with servo board
            name (str): name of the joint (eg. "shoulder")
            num (int): header that the servo is connected to (0 to 15)
            arm_range: joint range of motion, in degrees
            servo_range: servo range of motion, in degrees
            min_pulse: minimum servo pulse width
            max_pulse: maximum servo pulse width
            flipped (bool): True if joint angle is inverted from servo angle
                (eg. if joint max angle is reached at servo min angle). 
                Defaults to False
        '''
        self.name = name
        self.num = num
        self.servo = kit.servo[self.num]
        self.joint_min = 0 - joint_range/2
        self.joint_max = 0 + joint_range/2
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        if flipped:
            self.servo_min = 90 + servo_range/2
            self.servo_max = 90 - servo_range/2
        else:
            self.servo_min = 90 - servo_range/2
            self.servo_max = 90 + servo_range/2
        self.servo.set_pulse_width_range(self.min_pulse, self.max_pulse)
    
    def set_angle(self, angle):
        ''' Set servo angle
        Inputs:
            angle: the joint angle to set (input Nonetype to power down servo)
        Returns: None
        '''
        if angle is None:
            servo_angle = None
        else:            
            servo_angle = (angle*(self.servo_max - self.servo_min)
                           /(self.joint_max - self.joint_min) + 90
                           )
        self.servo.angle = servo_angle
    
    def set_pulses(self, min_pulse, max_pulse):
        ''' Set PWM pulse widths for minimum and maximum servo positions
        Inputs:
            min_pulse: pulse width for the servo's minimum
            max_pulse: pulse width for the servo's maximum
        '''
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.servo.set_pulse_width_range(self.min_pulse, self.max_pulse)


def arm_init(config):
    ''' Initialize SCARA arm
    Inputs:
        config (dict): configuration containing servo pulses
    Returns: 
        i2c (busio object): i2c communications object
        shoulder, elbow: joint objects for interacting directly with joints
    '''
    i2c = busio.I2C(board.SCL, board.SDA)
    hat = adafruit_pca9685.PCA9685(i2c)
    hat.frequency = 50
    kit = ServoKit(channels = 16)
    shoulder = joint(kit,
                     'shoulder',
                     0,
                     180,
                     180,
                     config['Shoulder min pulse'],
                     config['Shoulder max pulse'],
                     flipped=True
                     )
    elbow = joint(kit,
                  'elbow',
                  15,
                  316,
                  158,
                  config['Elbow min pulse'],
                  config['Elbow max pulse'],
                  )
    return i2c, shoulder, elbow


def arm_deinit(i2c, shoulder, elbow):
    ''' De-initialize SCARA arm
    Inputs:
        i2c (busio object): i2c communications object
        shoulder, elbow: joint objects for interacting directly with joints
    Returns: None
    '''
    shoulder.set_angle(None)
    elbow.set_angle(None)
    i2c.deinit()
    return


def find_angles(x, y, arm_lengths):
    ''' Inverse Kinematics solver for 2 arm segments, given desired position'
        'of end effector. Treats shoulder joint as (0,0).'
    Inputs:
        x: desired end effector x-coordinate
        y: desired end effector y-coordinate
        arm_lengths (tup): arm lengths, in inches (shoulder, elbow)
    Returns: 2-element list housing the solution pairs, in form
    [[Shoulder angle 1, Elbow angle 1], [Shoulder angle 2, Elbow angle 2]
    '''
    a = arm_lengths[0]
    b = arm_lengths[1]
    # Inverse Kinematics solution set 1
    angle_elbow1 = -math.acos((x**2+y**2-a**2-b**2)/(2*a*b))
    angle_shoulder1 = (math.atan2(y,x)-math.atan2(b*math.sin(angle_elbow1),
                                                 a+b*math.cos(angle_elbow1)))
    # Inverse Kinematics solution set 2
    angle_elbow2 = math.acos((x**2+y**2-a**2-b**2)/(2*a*b))
    angle_shoulder2 = (math.atan2(y,x)-math.atan2(b*math.sin(angle_elbow2),
                                                 a+b*math.cos(angle_elbow2)))
    output = [[angle_shoulder1, angle_elbow1], [angle_shoulder2, angle_elbow2]]
    output = [[math.degrees(angle) for angle in pair] for pair in output]
    return output


def calib_min_pulse(joint, angle):
    """ Utility for modifying minimum pulse length sent to servos.
    Inputs:
        joint: joint object
        angle: joint angle for which to modify pulse
    Returns: updated pulse value for input angle (str)
    """
    print(f'Current {joint.name} {angle}° pulse: {joint.min_pulse}. '
          'Enter new value, or press "enter" to proceed: ')
    pulse = joint.min_pulse
    while True:
        joint.set_angle(angle)
        new_val = input(': ')
        if not new_val:
            return pulse
        try:
            pulse = new_val
            joint.set_pulses(pulse, joint.max_pulse)
        except (ValueError, TypeError):
            print('Invalid input. Please enter a positive integer.')


def calib_max_pulse(joint, angle):
    """ Utility for modifying maximum pulse length sent to servos.
    Inputs:
        joint: joint object
        angle: joint angle for which to modify pulse
    Returns: updated pulse value for input angle (int)
    """
    print(f'Current {joint.name} {angle}° pulse: {joint.max_pulse}. '
          'Enter new value, or press "enter" to proceed: ')
    pulse = joint.max_pulse
    while True:
        joint.set_angle(angle)
        new_val = input(': ')
        if not new_val:
            return int(pulse)
        try:
            pulse = new_val
            joint.set_pulses(joint.min_pulse, pulse)
        except (ValueError, TypeError):
            print('Invalid input. Please enter a positive integer.')


def servo_calibration(shoulder, elbow, config):
    ''' Arm calibration
    Inputs:
        shoulder, elbow: joint objects for interacting directly with joints
        config (dict): dictionary containing config information
    Returns: config
    '''
    print('\n*****CALIBRATION MODE*****')
    start = input('Press Enter to start, or type "x" to exit: ')
    if not start:
        print('\nStarting calibration. Moving shoulder to -90°.\n')
        shoulder_max = calib_max_pulse(shoulder, -90)
        print('\nShoulder -90° pulse set. Moving shoulder to 90°.\n')
        shoulder_min = calib_min_pulse(shoulder, 90)
        print('\nShoulder 90° pulse set. Moving elbow to -158°.\n'
              )
        shoulder.set_angle(0)
        elbow_min = calib_min_pulse(elbow, -158)
        print('\nElbow -158° pulse set. Moving elbow to 158°.\n')
        elbow_max = calib_max_pulse(elbow, 158)
        print('\nElbow 158° pulse set.')
        is_save = input(
            'Calibration complete. Press "enter" to save new values, '
            'or type "x" to quit without saving.'
            )
        if not is_save:
            print('Saving calibration data to file........',end='')
            shoulder.set_pulses(shoulder_min, shoulder_max)
            elbow.set_pulses(elbow_min, elbow_max)
            write_data = ''.join(
                [f'{key}: {val}\n' for key, val in config.items()]
                )
            with open('scara_config.txt', 'w') as scara_config:
                scara_config.write(write_data)
                scara_config.truncate()
            print('done.')
            shoulder.set_angle(None)
            elbow.set_angle(None)
        print('Exiting to menu.\n')
        return config