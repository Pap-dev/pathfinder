import sys

from requests.auth import HTTPBasicAuth
from indicators import check_internet_connection, flight_restriction, weather_conditions

# https://github.com/public-apis/public-apis/blob/master/README.md#weather
class Check(object):

    def run_check(self, gcs_credentials, connection_string):
        preflight_data = {}

        internet_connection = check_internet_connection()
        if not internet_connection:
            print('Cannot proceed without internet connection.')
            return

        print("Connecting to vehicle...")
        vehicle = self.connect(connection_string, wait_ready=True)

        print("Collecting basic infos")
        preflight_data["GCS credentials"] = gcs_credentials
        preflight_data["UDP"] = connection_string
        preflight_data["system status"] = vehicle.system_status.state

        print("Collecting home location")
        preflight_data["home location local frame"] = vehicle.location.local_frame
        preflight_data["home global relative altitude"] = vehicle.location.global_relative_frame
        preflight_data["home location global frame"] = vehicle.location.global_frame    

        print("Checking flight restrictions in the vehicle zone")
        is_restricted = self.flight_restriction()
        if is_restricted:
            print("Cannot proceed, vehicle in no-fly zone!")
            sys.exit()

        print("Vehicle's location clear for takeoff")

        print("Checking weather conditions")
        preflight_data["weather conditions before takeoff"] = weather_conditions(vehicle.location.global_frame)

        print("Collecting vehicle's preflight attributes...")
        preflight_data["firmware version"] = vehicle.version()
        preflight_data["firmware major version number"] = vehicle.version.major()
        preflight_data["firmware minor version number"] = vehicle.version.minor()
        preflight_data["firmware release type"] = vehicle.version.release_type()
        preflight_data["firmware release version"] = vehicle.version.release_version()
        preflight_data["firmware stable release"] = vehicle.version.is_stable()
        preflight_data["hardware last heartbeat"] = vehicle.last_heartbeat
        preflight_data["hardware GPS status"] = vehicle.gps_0
        preflight_data["hardware gimbal status"] = vehicle.gimbal
        preflight_data["hardware battery status"] = vehicle.battery
        preflight_data["hardware EKF status"] = vehicle.ekf_ok
        preflight_data["hardware rangefinder status"] = vehicle.rangefinder
        preflight_data["hardware rangefinder distance"] = vehicle.rangefinder.distance
        preflight_data["hardware rangefinder voltage"] = vehicle.rangefinder.voltage

        print("Checking supports")
        preflight_data["MISSION_FLOAT messages support"] = vehicle.capabilities.mission_float
        preflight_data["PARAM_FLOAT messages support"] = vehicle.capabilities.param_float
        preflight_data["MISSION_INT messages support"] = vehicle.capabilities.mission_int
        preflight_data["COMMAND_INT messages support"] = vehicle.capabilities.command_int
        preflight_data["PARAM_UNION messages support"] = vehicle.capabilities.param_union
        preflight_data["FTP file transfers support"] = vehicle.capabilities.ftp
        preflight_data["attitude offboard command support"] = vehicle.capabilities.set_attitude_target
        preflight_data["set_attitude_target_local_ned command support"] = vehicle.capabilities.set_attitude_target_local_ned
        preflight_data["set_altitude_target_global_int command support"] = vehicle.capabilities.set_altitude_target_global_int
        preflight_data["terrain protocol data handling support"] = vehicle.capabilities.terrain
        preflight_data["direct actuator control support"] = vehicle.capabilities.set_actuator_target
        preflight_data["flight termination command support"] = vehicle.capabilities.flight_termination
        preflight_data["onboard compass calibration support"] = vehicle.capabilities.compass_calibration

        print("Initializing vehicle...")
        print("Request accelerometer calibration.")
        try:
            vehicle.send_calibrate_accelerometer(simple=True)
            print("Accelerometer calibration successful.")
            preflight_data["accelerometer calibration"] = True
        except:
            print("Accelerometer calibration failed.")
            preflight_data["accelerometer calibration"] = False

        print("Request barometer calibration.")
        try:
            vehicle.send_calibrate_barometer()
            print("Barometer calibration successful.")
            preflight_data["barometer calibration"] = True
        except:
            print("Barometer calibration failed.")
            preflight_data["barometer calibration"] = False

        print("Request gyroscope calibration.")
        try:
            vehicle.send_calibrate_gyro()
            print("Gyroscope calibration successful.")
            preflight_data["gyroscope calibration"] = True
        except:
            print("Gyroscope calibration failed.")
            preflight_data["gyroscope calibration"] = False

        print("Request magnetometer calibration.")
        try:
            vehicle.send_calibrate_magnetometer()
            print("Magnetometer calibration successful.")
            preflight_data["magnetometer calibration"] = True
        except:
            print("Magnetometer calibration failed.")
            preflight_data["magnetometer calibration"] = False

        print("Request vehicle level (accelerometer trim) calibration.")
        try:
            vehicle.send_calibrate_vehicle_level()
            print("Vehicle level calibration successful.")
            preflight_data["vehicle level calibration"] = True
        except:
            print("Vehicle level calibration failed.")
            preflight_data["vehicle level calibration"] = False
    