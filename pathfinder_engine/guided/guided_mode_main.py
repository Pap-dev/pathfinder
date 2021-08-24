from dronekit import connect, VehicleMode
from shared.preflight import Check
from guided_mode_controls import VehicleCommands
from dronekit import connect
from inputs import get_gamepad
from shared.callbacks import wildcard_callback, analyze_callbacks

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

        vehicle_attributes = {
            'version' : '', 
            'location.capabilities' : '', 
            'location.global_frame' : '',
            'location.global_relative_frame' : '', 
            'location.local_frame' : '', 
            'attitude' : '', 
            'velocity' : '', 
            'gps_0' : '', 
            'gimbal' : '',
            'battery' : '', 
            'rangefinder' : '', 
            'ekf_ok' : '', 
            'last_heartbeat' : '', 
            'home_location' : '', 
            'system_status' : '', 
            'heading' : '', 
            'is_armable' : '', 
            'airspeed' : '', 
            'groundspeed' : '', 
            'armed' : '', 
            'mode' : ''
        }   

        while 1:
            events = get_gamepad()
            gcs_gamepad = VehicleCommands()
            
            attribute_name, value = self.vehicle.add_attribute_listener('*', wildcard_callback)

            for attribute in list(vehicle_attributes):
                if attribute != attribute_name:
                    continue
                
                vehicle_attributes[attribute_name] = value

            alerts = self.analyze_callbacks(vehicle_attributes)
    
            if len(alerts) > 0:
                for alert in alerts:
                    print(alert)

            for event in events:
                
                input_name, input_value = gcs_gamepad.get_gamepad_inputs(event)

                for nested_dict in all_keys.items():
                    nested_dict.update((key, input_value) for key in nested_dict.items() \
                        if key == input_name)

                landing_instruction =  int(all_keys.get('Key BTN_START 1')) + int(all_keys.get('Key BTN_SELECT'))
                
                if landing_instruction < 2:
                    gcs_gamepad.send_inputs(all_keys, preferences)
                else:
                    landing_mode = preferences.get('landing_mode')
                    gcs_gamepad.land(landing_mode)

    def guided_mode_entrypoint(self, gcs_credentials, connection_string, preferences):

        print("Initializing controller...")
        all_keys = self.identify_gamepad_type()

        print("Connecting to vehicle...")
        vehicle = connect(connection_string, wait_ready=True)
        vehicle.mode = VehicleMode("GUIDED")

        print("Running preflight checks...")
        preval = Check()
        preval.run_check(gcs_credentials, connection_string)

        print("Initiate.")
        vehicle.controller_inputs_manager(all_keys, preferences)

    @staticmethod
    def get_info():
    
        preferences = {
            'camera_mode' : 'photo',
            'parachute_setting' : 'enable auto-release',
            'landing_mode': 'normal landing'
        }

        gcs_credentials = input("Pilot's credentials: ")
        connection_string = input("UAV connection string: ")

        camera_mode = input("Preferences - camera mode ? (photo/video)")
        parachute_setting = input("Preferences - parachute settings ?")
        landing_mode = input("Preferences - landing mode ?")

        preferences('camera_mode') = camera_mode
        preferences('parachute_setting') = parachute_setting
        preferences('landing_mode') = landing_mode

        return gcs_credentials, connection_string, preferences


if __name__ == "__main__":

    uav = GuidedMode()
    gcs_credentials, connection_string, preferences = uav.get_info()
    uav.guided_mode_entrypoint(gcs_credentials, connection_string, preferences)
