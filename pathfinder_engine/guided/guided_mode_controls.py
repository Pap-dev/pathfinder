"""
Most basic commands (taken from mavlink's MAV_CMD in common.xml)
"""

from pymavlink import mavutil
import time
import inputs

class VehicleCommands(object):
    JOYSTICK_ABSOLUTE_MAX = 32767
    JOYSTICK_ABSOLUTE_MIN = -32768
    TRIGGER_ABSOLUTE_MAX = 255

    def gripper_control(self, cargo_release):
        if cargo_release == 'release cargo':
            print("Releasing cargo...")
            gripper_action = 0 
        elif cargo_release == 'grab cargo':
            print("Grabbing cargo...")
            gripper_action = 1

        gripper_instance = 1

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_DO_GRIPPER, 
            0, 
            gripper_instance, 
            gripper_action, 
            0, 0, 0, 0, 0)

        self.vehicle.send_mavlink(msg)

    def winch_control(self, winch_action, winch_release_rate):
        winch_instance = 1
        if winch_action == 'relaxed':
            action_instance = 0
        elif winch_action == 'operate':
            print("Operating winch.")
            action_instance = 2

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_DO_WINCH, 
            0, 
            winch_instance, 
            action_instance,
            0, 
            winch_release_rate, 
            0, 0, 0)

        self.vehicle.send_mavlink(msg)

    def camera_mode(self, camera_mode):
        if camera_mode == 'photo':
            mode_instance = 0
        elif camera_mode == 'video':
            mode_instance = 1
        elif camera_mode == 'image survey':
            mode_instance = 2

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_SET_CAMERA_MODE, 
            0, 0, 
            mode_instance,
            0, 0, 0, 0, 0)

        self.vehicle.send_mavlink(msg)

    def video_capture(self, is_on = False):
        video_stream_id = 0
        status_frequency = 0

        if is_on:
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0, 
                mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE , 
                0, 
                video_stream_id, 
                status_frequency,
                0, 0, 0, 0, 0)

            print("Capturing video...")
            
            self.vehicle.send_mavlink(msg)
            return

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE, 
            0, 
            video_stream_id, 
            0, 0, 0, 0, 0, 0)

        self.vehicle.send_mavlink(msg)

    def take_pictures(self, is_on = False):
        number_of_images = 0 # take shots forever until is_on = False
        interval_between_shots = 0
        sequence_number = 0

        if is_on:
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0, 
                mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE, 
                0, 0, 
                interval_between_shots, 
                number_of_images, 
                sequence_number, 
                0, 0, 0)

            self.vehicle.send_mavlink(msg)
            return

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE, 
            0, 0, 0, 0, 0, 0, 0, 0)
            
        self.vehicle.send_mavlink(msg)

    def set_parachute_settings(self, parachute_setting):
        if parachute_setting == 'disable auto-release':
            parachute_action = 0  # i.e. release triggered by crash detectors
        elif parachute_setting == 'enable auto-release':
            parachute_action = 1
        elif parachute_setting == 'release and kill motors':
            parachute_action = 2 
            
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE, 
            0, 
            parachute_action, 
            0, 0, 0, 0, 0, 0)
        self.vehicle.send_mavlink(msg)
        
    def takeoff(self, altitude):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
            0, 0, 0, 0, 0, 0, 0, altitude)

        self.vehicle.send_mavlink(msg)

    def return_to_launch(self):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 
            0, 0, 0, 0, 0, 0, 0, 0)

        self.vehicle.send_mavlink(msg)

    def land(self, landing_mode):
        if landing_mode == 'normal landing':
            landing_instance = 0
        elif landing_mode == 'precision landing':
            landing_instance = 2 

        current_latitude = self.vehicle.location.lat
        current_longitude = self.vehicle.location.lon  
        ground_level = self.get_elevation(current_latitude, current_longitude)

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_NAV_LAND, 
            0, 0, 
            landing_instance, 
            0, 0, 
            current_latitude,
            current_longitude,
            ground_level)

        self.vehicle.send_mavlink(msg)
        
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)      
            if self.vehicle.location.global_relative_frame.alt >= ground_level * 0.95:
                print("Reached ground level.")
                break

            time.sleep(1)

    def mount_control(self, gimbal_pitch, gimbal_roll, gimbal_yaw):
        self.vehicle.gimbal.rotate(gimbal_pitch, gimbal_roll, gimbal_yaw)

    def yaw(self, yaw_degree):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            yaw_degree, 
            0, 1, 1, 0, 0, 0)

        self.vehicle.send_mavlink(msg)
        
    def direction(self, x_velocity, y_velocity, z_velocity):
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, 
            0, 0, 0, 
            x_velocity, 
            y_velocity, 
            z_velocity,
            0, 0, 0, 0, 0)

        self.vehicle.send_mavlink(msg)

    def send_gripper_controls(self, all_keys):
        delivery_trigger = all_keys.get('altitude_values',{}).get('Absolute ABS_HAT0Y')  
        if delivery_trigger < 0:
            self.send_delivery_controls(all_keys)

    def send_delivery_controls(self, all_keys):
        gear = 0
        descend = all_keys.get('trigger_values',{}).get('Absolute ABS_RZ')
        ascend = all_keys.get('trigger_values',{}).get('Absolute ABS_Z')

        winch_release_rate = ((descend + ascend) \
            / (2 * self.TRIGGER_ABSOLUTE_MAX)) * gear

        cargo_release = all_keys.get('boolean_values',{}).get('Key BTN_WEST')
         
        self.gripper_control(cargo_release)
        self.winch_control(2, winch_release_rate)

    def send_cam_controls(self, all_keys, preferences):
        cam_off = all_keys.get('incremental_keys',{}).get('Absolute ABS_HAT0X') 

        if cam_off == 0:
            return 

        trigger = all_keys.get('boolean_keys',{}).get('Key BTN_WEST')  

        if preferences.get('camera_mode') == 'photo':
            self.send_photo_controls(trigger)
            if trigger != 0:
                self.take_pictures(is_on = True)
            else:
                self.take_pictures(is_on = False)

        if preferences.get('camera_mode') == 'video':
            self.send_photo_controls(trigger)   
            if trigger != 0:
                self.video_capture(is_on = True)
            else:
                self.video_capture(is_on = False)        

    def send_mount_controls(self, all_keys):
        mount_dependance = all_keys.get('angular_values',{}).get('Key BTN_THUMBR') 

        if mount_dependance == 0:
            rs_yaxis = all_keys.get('angular_values',{}).get('Absolute ABS_RY')
            gimbal_pitch = self.sticks_percentage(rs_yaxis) * 180

            self.mount_control(gimbal_pitch, 0, 0)
            return

        rs_xaxis = all_keys.get('angular_values',{}).get('Absolute ABS_RX')
        gimbal_yaw = self.sticks_percentage(rs_xaxis) * 360

        rs_yaxis = all_keys.get('angular_values',{}).get('Absolute ABS_RY')
        gimbal_pitch = self.sticks_percentage(rs_yaxis) * 180

        self.mount_control(gimbal_pitch, 0, gimbal_yaw)

    def send_yaw(self, all_keys):
        gear = 0

        mount_dependance = all_keys.get('angular_values',{}).get('Key BTN_THUMBR') 
        if mount_dependance == 1:
            return
        
        rs_xaxis = all_keys.get('angular_values',{}).get('Absolute ABS_RX')
        yaw_degree = self.sticks_percentage(rs_xaxis) * 360

        self.yaw(yaw_degree)

    def send_directions(self, all_keys):
        gear = 0

        ls_xaxis = all_keys.get('precision_values',{}).get('Absolute ABS_X')
        x_velocity = self.sticks_percentage(ls_xaxis) * gear

        ls_yaxis = all_keys.get('precision_values',{}).get('Absolute ABS_Y')
        y_velocity = self.sticks_percentage(ls_yaxis) * gear

        right_bumper = all_keys.get('altitude_values',{}).get('Key BTN_TR')
        left_bumper = all_keys.get('altitude_values',{}).get('Key BTN_TL 1')
        z_velocity = right_bumper + left_bumper

        self.direction(x_velocity, y_velocity, z_velocity)

    def sticks_percentage(self, value):
        if value < 0:
            value = value / self.JOYSTICK_ABSOLUTE_MIN
        elif value > 0:
            value = value / self.JOYSTICK_ABSOLUTE_MAX
        else:
            value = 0
        return value

    def vibrate(gamepad=None):
        if not gamepad:
            gamepad = inputs.devices.gamepads[0]

        # # Vibrate left
        # gamepad.set_vibration(1, 0, 1000)
        # # Vibrate right
        # gamepad.set_vibration(0, 1, 1000)

        # Vibrate Both
        gamepad.set_vibration(1, 1, 2000)
        
        time.sleep(2)

    def get_gamepad_inputs(self, event):
        key = event.code
        value = event.state
        type = event.type

        return key, value, type

    def send_inputs(self, all_keys, preferences):
        self.send_directions(all_keys)
        self.send_yaw(all_keys)
        self.send_mount_controls(all_keys)
        self.send_cam_controls(all_keys, preferences)
        self.send_gripper_controls(all_keys)
        self.send_delivery_controls(self, all_keys)