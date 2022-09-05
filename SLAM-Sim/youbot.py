import math
import random
import time

import numpy as np
from PIL import Image

import vrep

ONE_SHOT_MODE = vrep.simx_opmode_oneshot_wait
STREAM_MODE = vrep.simx_opmode_streaming
BLOCKING_MODE = vrep.simx_opmode_blocking
BUFFER_MODE = vrep.simx_opmode_buffer


class YouBot(object):
    def __init__(self, client_id, postfix):
        self.client_id = client_id
        self.postfix = postfix
        self.handles = self.__get_handles()
        self.standard_deviation = 0.01

    def __get_handles(self):
        handles = {}
        labels = ['rollingJoint_rr', 'rollingJoint_rl', 'rollingJoint_fr',
                  'rollingJoint_fl', 'ResizableFloor', 'youBot_positionTarget',
                  'youBot_orientationBase', 'kinect_rgb', 'kinect_depth', 'youBot', 'ME_Platfo2_sub1']
        if self.postfix != '':
            for i in range(len(labels)):
                labels[i] = labels[i] + self.postfix
        for i in labels:
            error_code, handles[i] = vrep.simxGetObjectHandle(self.client_id, i, ONE_SHOT_MODE)
        return handles

    def replace_robot(self, x, y, z):
        """Immediately changes robot's position to specified by coordinates x, y, z"""
        # min z = 0.1
        arr = [x, y, z]
        vrep.simxSetObjectPosition(self.client_id, self.handles['youBot' + self.postfix], -1, arr, ONE_SHOT_MODE)

    def get_position(self):
        position = vrep.simxGetObjectPosition(
            self.client_id, self.handles['youBot_positionTarget' + self.postfix],
            -1, BLOCKING_MODE
        )[1]
        return position


    def get_rotation(self):
        angle = vrep.simxGetObjectOrientation(
            self.client_id, self.handles['youBot_positionTarget' + self.postfix],
            -1, BLOCKING_MODE
        )[1]
        return angle

    # Basic movements
    def stop(self):
        """Stop robot"""
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_rr' + self.postfix], 0,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_rl' + self.postfix], 0,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_fr' + self.postfix], 0,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_fl' + self.postfix], 0,
                                        ONE_SHOT_MODE)

    def forward(self, speed):
        """Make robot go forward with specified speed"""
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_rr' + self.postfix], -speed,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_rl' + self.postfix], -speed,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_fr' + self.postfix], -speed,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_fl' + self.postfix], -speed,
                                        ONE_SHOT_MODE)

    def backward(self, speed):
        """Make robot go backward with specified speed"""
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_rr' + self.postfix], speed,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_rl' + self.postfix], speed,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_fr' + self.postfix], speed,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_fl' + self.postfix], speed,
                                        ONE_SHOT_MODE)

    def right(self, speed):
        """Make robot turn right with specified speed"""
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_rr' + self.postfix], speed,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_rl' + self.postfix], -speed,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_fr' + self.postfix], speed,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_fl' + self.postfix], -speed,
                                        ONE_SHOT_MODE)

    def left(self, speed):
        """Make robot turn left with specified speed"""
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_rr' + self.postfix], -speed,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_rl' + self.postfix], speed,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_fr' + self.postfix], -speed,
                                        ONE_SHOT_MODE)
        vrep.simxSetJointTargetVelocity(self.client_id, self.handles['rollingJoint_fl' + self.postfix], speed,
                                        ONE_SHOT_MODE)

    # Additional movement functions
    def move(self, delta, speed=0.5):
        """Moves robot on determined distance with determined speed"""
        start_point = self.get_position()
        distance = 0
        self.forward(speed)
        while distance < delta:
            current_point = self.get_position()
            distance = math.sqrt(
                (start_point[0] - current_point[0]) ** 2 +
                (start_point[1] - current_point[1]) ** 2
            )
        self.stop()


    # TODO: Add standard deviation for turn-angle function
    def turn_angle(self, angle, speed=1.0):
        """Turn robot on determined angle"""
        mt_buf = bytearray()
        error = random.normalvariate(0.5, self.standard_deviation)

        res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(
            self.client_id,
            'youBot_ref' + self.postfix,
            vrep.sim_scripttype_childscript,
            'GetRobotAngle', [], [], [], mt_buf,
            BLOCKING_MODE)

        start_angle = ret_floats[0] + error
        delta = 0

        # вызов скрипта поворота
        vrep.simxCallScriptFunction(
            self.client_id,
            'youBot_ref' + self.postfix,
            vrep.sim_scripttype_childscript,
            'Turn', [],
            [speed], [], mt_buf,
            BLOCKING_MODE)

        while delta <= angle:
            res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(
                self.client_id,
                'youBot_ref' + self.postfix,
                vrep.sim_scripttype_childscript,
                'GetRobotAngle', [], [], [], mt_buf,
                BLOCKING_MODE)

            current_angle = ret_floats[0] + error
            delta += math.fabs(current_angle - start_angle)
            start_angle = current_angle

        vrep.simxCallScriptFunction(
            self.client_id,
            'youBot_ref' + self.postfix,
            vrep.sim_scripttype_childscript,
            'Turn', [], [0.0], [], mt_buf,
            BLOCKING_MODE)

    def move_distance(self, distance, speed=1.0):
        """Move forward on determined distance"""
        distance = random.normalvariate(distance, self.standard_deviation)

        start_point = self.get_position()
        traveled_distance = 0
        while traveled_distance < distance:
            self.backward(speed)
            current_point = self.get_position()
            traveled_distance = math.sqrt(
                math.pow((start_point[0] - current_point[0]), 2) + math.pow((start_point[1] - current_point[1]), 2))
        self.stop()

    # Sensors
    def get_robot_speed(self):
        ret = vrep.simxGetObjectVelocity(self.client_id, self.handles['youBot' + self.postfix], BLOCKING_MODE)
        return ret[1]

    def get_lidar_data(self):
        """Returns data measured using LIDAR. Make sure robot you use has a lidar"""
        error_code, ranges = vrep.simxGetStringSignal(self.client_id, 'scan ranges', STREAM_MODE)
        time.sleep(0.1)
        while len(ranges) == 0:
            error_code, ranges = vrep.simxGetStringSignal(self.client_id, 'scan ranges', BUFFER_MODE)
        ranges = vrep.simxUnpackFloats(ranges)
        return ranges

    def get_image_from_kinect(self):
        res, resolution, image = vrep.simxGetVisionSensorImage(self.client_id, self.handles['kinect_rgb' + self.postfix], 0,
                                                               ONE_SHOT_MODE)
        image_nparray = np.array(image, dtype=np.uint8)
        image_nparray.resize([resolution[1], resolution[0], 3])
        image = Image.fromarray(image_nparray.astype('uint8'), 'RGB')
        image = image.transpose(Image.ROTATE_180).save("tmp/tmp.gif")
        return image

    def get_depth_from_kinect(self):
        _, depth_resolution, depth_image = vrep.simxGetVisionSensorDepthBuffer(
            self.client_id, self.handles['kinect_depth' + self.postfix], ONE_SHOT_MODE)
        depth_arr = np.reshape(depth_image, (480, 640))
        return depth_arr.tolist()

    # Sensor parameters
    params_f = {'near_clipping_plane': 1000,
                'far_clipping_plane': 1001,
                'perspective_angle': 1004
                }

    params_i = {'vision_sensor_resolution_x': 1002,
                'vision_sensor_resolution_y': 1003
                }

    def set_parameter(self, sensor_name, parameter_name, parameter_value):
        """Set sensor parameters. Sensor name can be fastHokuyo_sensor1, kinect_depth, kinect_rgb.
        To get list of possible params call youbot.parameters_list"""
        if parameter_name == 'perspective_angle':
            parameter_value = parameter_value / (180 * 2) * math.pi
        if parameter_name in self.params_f:
            error = vrep.simxSetObjectFloatParameter(
                self.client_id,
                self.handles[sensor_name + self.postfix],
                self.params_f[parameter_name],
                parameter_value,
                ONE_SHOT_MODE
            )
            vrep.simxSetFloatSignal(
                self.client_id,
                'change_params',
                parameter_value,
                ONE_SHOT_MODE
            )
            vrep.simxClearFloatSignal(
                self.client_id,
                'change_params',
                ONE_SHOT_MODE
            )
            return error
        elif parameter_name in self.params_i:
            error = vrep.simxSetObjectFloatParameter(
                self.client_id,
                self.handles[sensor_name + self.postfix],
                self.params_i[parameter_name],
                parameter_value,
                ONE_SHOT_MODE
            )
            vrep.simxSetFloatSignal(
                self.client_id,
                'change_params',
                parameter_value,
                ONE_SHOT_MODE
            )
            vrep.simxClearFloatSignal(
                self.client_id,
                'change_params',
                ONE_SHOT_MODE
            )
            return error
        else:
            return 'Parameter not found'

    def get_parameter(self, sensor_name, parameter_name):
        if parameter_name in self.params_f:
            _, value = vrep.simxGetObjectFloatParameter(
                self.client_id,
                self.handles[sensor_name + self.postfix],
                self.params_f[parameter_name],
                ONE_SHOT_MODE
            )
            if parameter_name == 'perspective_angle':
                value = value * 180 * 2 / math.pi
            return value
        elif parameter_name in self.params_i:
            _, value = vrep.simxGetObjectIntParameter(
                self.client_id,
                self.handles[sensor_name + self.postfix],
                self.params_i[parameter_name],
                ONE_SHOT_MODE
            )
            return value
        else:
            return 'Parameter not found'

    def sensor_parameters_list(self):
        """Get list of available parameters"""
        return list(self.params_f.keys()) + list(self.params_i.keys())
