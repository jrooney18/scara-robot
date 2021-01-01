import sys
import warnings

import cv2 as cv
import numpy as np
from picamera import PiCamera

class ScaraImage:
    ''' Holds data for a single image taken by the robot's camera
    Attributes:
        self.raw: the raw image data
        self.unfisheyed: image corrected for fisheye distortion
        self.warped: image warped to correct for perspective
        self.contoured: warped image with contours overlaid
        self.cx, self.cy: image coordinates of a target disk's centroid
        self.dims: dimensions of the raw image
    '''
    def __init__(self, camera):
        ''' Initializes a ScaraImage object.
        camera: PiCamera camera object.
        '''
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            self.raw = np.empty((480, 640, 3), dtype=np.uint8)
            camera.capture(self.raw, format='bgr', use_video_port=True)

    def dims(self):
        ''' Returns image dimensions.
        '''
        return self.raw.shape[:2][::-1]

    def unfisheye(self, maps):
        ''' Corrects for fisheye distortion, using predetermined lens
            parameters K and D (specific to Inland 130­° wide-angle camera
            module for RPi, found using calibrate.py)
        Inputs:
            maps (tuple of arrays): pre-calculated remapping arrays 
                (see camera_init)
        Returns: Image corrected for fisheye distortion.
        '''
        self.unfisheyed = cv.remap(
            self.raw, maps[0], maps[1], interpolation=cv.INTER_LINEAR,
            borderMode=cv.BORDER_CONSTANT
            )
        return self.unfisheyed

    def find_target(self, target_color, image=np.zeros(1)):
        ''' Finds centroid of a target disk.
        Inputs:
            target_color (tup): 2-element tuple housing numpy arrays. First 
                element houses minimum HSV values of target, 
                second houses maximum values.
            img: OpenCV image to process. Defaults to self.warped if not set.
        Returns: 
             cx, cy: coordinates of the disk's centroid (None, None if no disk)
        '''
        def arraysize(array):
            ''' Helper function for array sorting
            '''
            return array.size
        
        if not image.any():
            image = self.unfisheyed
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        # Mask out HSV image based on defined bounds, and find contour of disk
        self.mask = cv.inRange(hsv, target_color[0], target_color[1])
        contours, hierarchy = cv.findContours(
            self.mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        contours.sort(reverse=True, key=arraysize)
        target = None
        for contour in contours:
            # Iterate through reverse-sorted contour list to find largest 
            # roughly circular contour of correct size
            x, y, w, h = cv.boundingRect(contour)
            if abs(w - h) <= 3 and w > 19:
                target = contour
                break
        
        if target is not None:
            self.contoured = image.copy()
            cv.drawContours(self.contoured, contours, -1, (0, 0, 255), 2)
            cv.drawContours(self.contoured, [target], 0, (0, 255, 0), 2)
            
            moments = cv.moments(target)
            self.cx = int(moments['m10']/moments['m00'])
            self.cy = int(moments['m01']/moments['m00'])
        else:
            self.cx = None
            self.cy= None
        return (self.cx, self.cy)

    def warp(self, warp_matrix):
        ''' Transforms target coordinates to correct for perspective distortion
        Inputs:
            warp_matrix: 3x3 numpy array
        Returns: warped image
        '''
        if self.cx is not None:
            x_num = (warp_matrix[0][0]*self.cx
                     + warp_matrix[0][1]*self.cy
                     + warp_matrix[0][2]
                     )
            y_num = (warp_matrix[1][0]*self.cx
                     + warp_matrix[1][1]*self.cy
                     + warp_matrix[1][2]
                     )
            denom = (warp_matrix[2][0]*self.cx
                     + warp_matrix[2][1]*self.cy
                     + warp_matrix[2][2]
                     )
            self.cx_corrected = x_num/denom
            self.cy_corrected = y_num/denom
        else:
            self.cx_corrected = None
            self.cy_corrected = None
        return self.cx_corrected, self.cy_corrected

    def get_real_coords(self):
        ''' Transforms target coordinate units from pixels to inches
        '''
        if self.cx is not None:
            self.target_x = 11.5 - self.cy_corrected * 12/(334)
            self.target_y = 11.5 - self.cx_corrected * 23/(640)
        else:
            self.target_x = None
            self.target_y = None
        return (self.target_x, self.target_y)


def calibrate_camera(camera, target_color, maps):
    ''' Find image warp matrix to correct for perspective distortion, based on
    four reference points.
    Inputs:
        camera: PiCamera camera object
        target_color (tup): 2-element tuple housing numpy arrays. First 
                element houses minimum HSV values of target, 
                second houses maximum values.
        maps (tuple of arrays): pre-calculated remapping arrays 
                (see camera_init)
    Outputs:
        warp_matrix: 3x3 numpy array housing perspective transform
    '''
    print('\n*****CALIBRATION MODE*****')
    # Actual coordinates to use for calibration, in inches from top left
    coords_corrected = np.float32([[1,11.5], [22,11.5], [4.5,4.5], [18.5,4.5]])
    coords_corrected[:,0] = coords_corrected[:,0]*640/23
    coords_corrected[:,1] = coords_corrected[:,1]*334/12
    input(
        'Place calibration disks on indicated points. Press "enter" to begin:'
        )
    coords_raw = []
    points = ['(0, 10.5)', '(0, -10.5)', '(7, 7)', '(7, -7)']
    for i in range(4):
        input(f'Point {i+1}: {points[i]}. Press "enter" when set:')
        img = ScaraImage(camera)
        img.unfisheye(maps)
        x, y = img.find_target(target_color, image=img.unfisheyed)
        print(f'Target coordinates: ({x}, {y})')
        cv.imwrite('contoured.jpg', img.contoured)
        coords_raw.append((x, y))
    coords_raw = np.asfarray(coords_raw, dtype='float32')
    print('Calibrating........', end='')
    sys.stdout.flush()
    warp_matrix = cv.getPerspectiveTransform(coords_raw, coords_corrected)
    print('done.', end='')
    is_save = input('Press "enter" to save new calibration, '
                    'or type "x" to exit without saving: '
                    )
    if not is_save:
        print('Saving calibration data to file........', end='')
        sys.stdout.flush()
        config = {}
        with open('scara_config.txt', 'r+') as scara_config:
            for line in scara_config:
                try:
                    (key, val) = line.split(': ')
                except ValueError:
                    pass
                config[key] = val
            config['Image warp matrix'] = str(warp_matrix[0,:])+' '+str(warp_matrix[1,:])+' '+str(warp_matrix[2,:])
            write_data = ''.join([f'{key}: {val}' for key, val in config.items()])
            scara_config.seek(0)
            scara_config.write(write_data)
            scara_config.truncate()
        print('done.\nExiting to menu.\n')
        return warp_matrix
    print('Exiting to menu.\n')


def camera_init():
    ''' Initialize PiCamera
    Inputs: None
    Returns: 
        camera (PiCamera object): camera, with resolution as specified
        maps (tuple of array): fisheye correction arrays for cv.remap
    '''
    camera = PiCamera()
    camera.resolution = (640, 480)
    
    # Calculate fisheye correction matrices for current resolution
    K=np.array([
        [1462.6951722364404, 0.0, 1299.7703062382923],
        [0.0, 1470.2235929713356, 1117.7747313303344],
        [0.0, 0.0, 1.0]
        ])
    D=np.array([
        [0.009814035612875105], [-0.11013806105044097],
        [0.10444574816339373], [-0.026263330725614136]
        ])
    dim = (int(640), int(480)) # Dimensions of input image
    dim_cal = (2592, 1944) # Dimensions of lens calibration images
    
    scaled_K = K*dim[0]/dim_cal[0]
    scaled_K[2][2] = 1.0
    
    new_K = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(
        scaled_K, D, dim, np.eye(3), balance=0.0
        )
    map1, map2 = cv.fisheye.initUndistortRectifyMap(
        scaled_K, D, np.eye(3), new_K, dim, cv.CV_16SC2
        )
    maps = (map1, map2)
    return camera, maps


def image_capture(camera):
    ''' Capture image to file
    Inputs:
        camera: PiCamera camera object
    Returns: none
    '''
    filename = input('Enter file name: ')
    while True:
        try:
            print('Capturing image...')
            with warnings.catch_warnings():
                warnings.simplefilter('ignore')
                camera.capture(filename)
            print('done. Returning to menu.')
            return
        except (TypeError, ValueError):
            print('Invalid file name.')


def find_target(camera, warp_matrix, target_color, maps):
    ''' Finds target disk
    Inputs:
        camera: PiCamera camera object
        warp_matrix (numpy array): transformation matrix defining image warp
        target_color (tup): 2-element tuple housing numpy arrays. First element
            houses minimum HSV values of target, second houses maximum values.
    Returns:
        Disk coordinates x, y (None, None if no target is found)
    '''
    img = ScaraImage(camera)
    # cv.imwrite('raw.jpg', img.raw)
    img.unfisheye(maps)
    # cv.imwrite('corrected.jpg', img.unfisheyed)
    img.find_target(target_color)
    # cv.imwrite('contoured.jpg', img.contoured)
    # cv.imwrite('mask.jpg', img.mask)
    img.warp(warp_matrix)
    x, y = img.get_real_coords()
    if x is not None:
        x = round(x, 2)
        y = round(y, 2)
    return x, y