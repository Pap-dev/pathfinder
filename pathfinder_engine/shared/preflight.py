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

        print("Collecting basic infos")
        preflight_data["GCS credentials"] = gcs_credentials
        preflight_data["UDP"] = connection_string
        preflight_data["system status"] = self.vehicle.system_status.state

        print("Collecting home location")
        preflight_data["home location local frame"] = self.vehicle.location.local_frame
        preflight_data["home global relative altitude"] = self.vehicle.location.global_relative_frame
        preflight_data["home location global frame"] = self.vehicle.location.global_frame    

        print("Checking flight restrictions in the vehicle zone")
        is_restricted = self.flight_restriction()
        if is_restricted:
            print("Cannot proceed, vehicle in no-fly zone!")
            sys.exit()

        print("Vehicle's location clear for takeoff")

        print("Checking weather conditions")
        preflight_data["weather conditions before takeoff"] = weather_conditions(self.vehicle.location.global_frame)

        print("Collecting vehicle's preflight attributes...")
        preflight_data["firmware version"] = self.vehicle.version()
        preflight_data["firmware major version number"] = self.vehicle.version.major()
        preflight_data["firmware minor version number"] = self.vehicle.version.minor()
        preflight_data["firmware release type"] = self.vehicle.version.release_type()
        preflight_data["firmware release version"] = self.vehicle.version.release_version()
        preflight_data["firmware stable release"] = self.vehicle.version.is_stable()
        preflight_data["hardware last heartbeat"] = self.vehicle.last_heartbeat
        preflight_data["hardware GPS status"] = self.vehicle.gps_0
        preflight_data["hardware gimbal status"] = self.vehicle.gimbal
        preflight_data["hardware battery status"] = self.vehicle.battery
        preflight_data["hardware EKF status"] = self.vehicle.ekf_ok
        preflight_data["hardware rangefinder status"] = self.vehicle.rangefinder
        preflight_data["hardware rangefinder distance"] = self.vehicle.rangefinder.distance
        preflight_data["hardware rangefinder voltage"] = self.vehicle.rangefinder.voltage

        print("Checking supports")
        preflight_data["MISSION_FLOAT messages support"] = self.vehicle.capabilities.mission_float
        preflight_data["PARAM_FLOAT messages support"] = self.vehicle.capabilities.param_float
        preflight_data["MISSION_INT messages support"] = self.vehicle.capabilities.mission_int
        preflight_data["COMMAND_INT messages support"] = self.vehicle.capabilities.command_int
        preflight_data["PARAM_UNION messages support"] = self.vehicle.capabilities.param_union
        preflight_data["FTP file transfers support"] = self.vehicle.capabilities.ftp
        preflight_data["attitude offboard command support"] = self.vehicle.capabilities.set_attitude_target
        preflight_data["set_attitude_target_local_ned command support"] = self.vehicle.capabilities.set_attitude_target_local_ned
        preflight_data["set_altitude_target_global_int command support"] = self.vehicle.capabilities.set_altitude_target_global_int
        preflight_data["terrain protocol data handling support"] = self.vehicle.capabilities.terrain
        preflight_data["direct actuator control support"] = self.vehicle.capabilities.set_actuator_target
        preflight_data["flight termination command support"] = self.vehicle.capabilities.flight_termination
        preflight_data["onboard compass calibration support"] = self.vehicle.capabilities.compass_calibration

        print("Initializing vehicle...")
        print("Request accelerometer calibration.")
        try:
            self.vehicle.send_calibrate_accelerometer(simple=True)
            print("Accelerometer calibration successful.")
            preflight_data["accelerometer calibration"] = True
        except:
            print("Accelerometer calibration failed.")
            preflight_data["accelerometer calibration"] = False

        print("Request barometer calibration.")
        try:
            self.vehicle.send_calibrate_barometer()
            print("Barometer calibration successful.")
            preflight_data["barometer calibration"] = True
        except:
            print("Barometer calibration failed.")
            preflight_data["barometer calibration"] = False

        print("Request gyroscope calibration.")
        try:
            self.vehicle.send_calibrate_gyro()
            print("Gyroscope calibration successful.")
            preflight_data["gyroscope calibration"] = True
        except:
            print("Gyroscope calibration failed.")
            preflight_data["gyroscope calibration"] = False

        print("Request magnetometer calibration.")
        try:
            self.vehicle.send_calibrate_magnetometer()
            print("Magnetometer calibration successful.")
            preflight_data["magnetometer calibration"] = True
        except:
            print("Magnetometer calibration failed.")
            preflight_data["magnetometer calibration"] = False

        print("Request vehicle level (accelerometer trim) calibration.")
        try:
            self.vehicle.send_calibrate_vehicle_level()
            print("Vehicle level calibration successful.")
            preflight_data["vehicle level calibration"] = True
        except:
            print("Vehicle level calibration failed.")
            preflight_data["vehicle level calibration"] = False

        return preflight_data
    