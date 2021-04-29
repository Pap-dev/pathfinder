from dronekit import connect, VehicleMode
from preflight import Check
from guided_mode_controls import Controller
from dronekit import connect
from inputs import get_gamepad

class GuidedMode(object):

    @staticmethod
    def identify_gamepad_type():
        all_keys =[]
        return all_keys

    def controller_inputs_manager(self, all_keys, preferences):

        if not bool(all_keys):
            all_keys = {
                'precision_values' : {
                    'Absolute ABS_X' : 0, # LS X-axis 
                    'Absolute ABS_Y' : 0, # LS Y-axis 
                    },
                'trigger_values' : {
                    'Absolute ABS_Z' : 0, # LT
                    'Absolute ABS_RZ' : 0 # RT
                    },
                'altitude_values' : {
                    'Key BTN_TR' : 0, # RB
                    'Key BTN_TL 1' : 0 # LB
                    },
                'angular_values' : {
                    'Absolute ABS_RY' : 0, # RS Y-axis 
                    'Absolute ABS_RX' : 0, # RS X-axis 
                    },
                'boolean_values' : {
                    'Key BTN_THUMBL' : 0, # LS press
                    'Key BTN_START 1' : 0, # SELECT
                    'Key BTN_SELECT' : 0, # START
                    'Key BTN_THUMBR' : 0, # RS press
                    'Key BTN_WEST' : 0, # X
                    'Key BTN_SOUTH' : 0, # A
                    'Key BTN_EAST' : 0, # B
                    'Key BTN_NORTH' : 0 # Y
                    },
                'incremental_keys' : {        
                    'Absolute ABS_HAT0X' : 0, # DPAD X-axis
                    'Absolute ABS_HAT0Y' : 0 # DPAD Y-axis
                    }
            }

        if not bool(preferences):
            preferences = {
                'camera_mode' : 'photo',
                'parachute_setting' : 'enable auto-release',
                'landing_mode': 'normal landing'
            } # default preferences

        while 1:
            events = get_gamepad()

            for event in events:
                gcs_gamepad = Controller()
                input_name, input_value, input_type = gcs_gamepad.get_gamepad_inputs(event)

                for nested_dict in all_keys.items():
                    nested_dict.update((key, input_value) for key in nested_dict.items() \
                        if key == input_name)

                gcs_gamepad.send_inputs(all_keys, preferences)

    def run_guided_mode(self, gcs_credentials, connection_string, preferences):

        print("Connecting to vehicle...")
        vehicle = connect(connection_string, wait_ready=True)
        vehicle.mode = VehicleMode("GUIDED")

        print("Running preflight checks...")
        vehicle.run_check(gcs_credentials, connection_string)

        print("Running preflight checks...")
        all_keys = self.identify_gamepad_type()

        print("Initiate")
        vehicle.controller_inputs_manager(all_keys, preferences)