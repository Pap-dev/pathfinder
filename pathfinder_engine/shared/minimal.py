from pymavlink import mavutil

autopilot_generic = mavutil.mavlink.MAV_AUTOPILOT_GENERIC
autopilot_generic_value = 0
autopilot_generic_description = "Generic autopilot, full support for everything"

autopilot_reserved = mavutil.mavlink.MAV_AUTOPILOT_RESERVED
autopilot_reserved_value = 1
autopilot_reserved_description = "Reserved for future use."

autopilot_slugs = mavutil.mavlink.MAV_AUTOPILOT_SLUGS
autopilot_slugs_value = 2
autopilot_slugs_description = "SLUGS autopilot, http://slugsuav.soe.ucsc.edu"

autopilot_ardupilotmega = mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
autopilot_ardupilotmega_value = 3
autopilot_ardupilotmega_description = "ArduPilot - Plane/Copter/Rover/Sub/Tracker, https://ardupilot.org"

autopilot_openpilot = mavutil.mavlink.MAV_AUTOPILOT_OPENPILOT
autopilot_openpilot_value = 4
autopilot_openpilot_description = "OpenPilot, http://openpilot.org"

autopilot_generic_waypoints_only = mavutil.mavlink.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY
autopilot_generic_waypoints_only_value = 5
autopilot_generic_waypoints_only_description = "Generic autopilot only supporting simple waypoints"

autopilot_generic_waypoints_and_simple_navigation_only = mavutil.mavlink.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY
autopilot_generic_waypoints_and_simple_navigation_only_value = 6
autopilot_generic_waypoints_and_simple_navigation_only_description = "Generic autopilot supporting waypoints and other simple navigation commands"

autopilot_generic_mission_full = mavutil.mavlink.MAV_AUTOPILOT_GENERIC_MISSION_FULL
autopilot_generic_mission_full_value = 7
autopilot_generic_mission_full_description = "Generic autopilot supporting the full mission command set"

autopilot_invalid = mavutil.mavlink.MAV_AUTOPILOT_INVALID
autopilot_invalid_value = 8
autopilot_invalid_description = "No valid autopilot, e.g. a GCS or other MAVLink component"

autopilot_ppz = mavutil.mavlink.MAV_AUTOPILOT_PPZ
autopilot_ppz_value = 9
autopilot_ppz_description = "PPZ UAV - http://nongnu.org/paparazzi"

autopilot_udb = mavutil.mavlink.MAV_AUTOPILOT_UDB
autopilot_udb_value = 10
autopilot_udb_description = "UAV Dev Board"

autopilot_fp = mavutil.mavlink.MAV_AUTOPILOT_FP
autopilot_fp_value = 11
autopilot_fp_description = "FlexiPilot"

autopilot_px4 = mavutil.mavlink.MAV_AUTOPILOT_PX4
autopilot_px4_value = 12
autopilot_px4_description = "PX4 Autopilot - http://px4.io/"

autopilot_smaccmpilot = mavutil.mavlink.MAV_AUTOPILOT_SMACCMPILOT
autopilot_smaccmpilot_value = 13
autopilot_smaccmpilot_description = "SMACCMPilot - http://smaccmpilot.org"

autopilot_autoquad = mavutil.mavlink.MAV_AUTOPILOT_AUTOQUAD
autopilot_autoquad_value = 14
autopilot_autoquad_description = "AutoQuad -- http://autoquad.org"

autopilot_armazila = mavutil.mavlink.MAV_AUTOPILOT_ARMAZILA
autopilot_armazila_value = 15
autopilot_armazila_description = "Armazila -- http://armazila.com"

autopilot_aerob = mavutil.mavlink.MAV_AUTOPILOT_AEROB
autopilot_aerob_value = 16
autopilot_aerob_description = "Aerob -- http://aerob.ru"

autopilot_asluav = mavutil.mavlink.MAV_AUTOPILOT_ASLUAV
autopilot_asluav_value = 17
autopilot_asluav_description = "ASLUAV autopilot -- http://www.asl.ethz.ch"

autopilot_smartap = mavutil.mavlink.MAV_AUTOPILOT_SMARTAP
autopilot_smartap_value = 18
autopilot_smartap_description = "SmartAP Autopilot - http://sky-drones.com"

autopilot_airrails = mavutil.mavlink.MAV_AUTOPILOT_AIRRAILS
autopilot_airrails_value = 19
autopilot_airrails_description = "AirRails - http://uaventure.com"

type_generic = mavutil.mavlink.MAV_TYPE_GENERIC
type_generic_value = 0
type_generic_description = "Generic micro air vehicle"

type_fixed_wing = mavutil.mavlink.MAV_TYPE_FIXED_WING
type_fixed_wing_value = 1
type_fixed_wing_description = "Fixed wing aircraft."

type_quadrotor = mavutil.mavlink.MAV_TYPE_QUADROTOR
type_quadrotor_value = 2
type_quadrotor_description = "Quadrotor"

type_coaxial = mavutil.mavlink.MAV_TYPE_COAXIAL
type_coaxial_value = 3
type_coaxial_description = "Coaxial helicopter"

type_helicopter = mavutil.mavlink.MAV_TYPE_HELICOPTER
type_helicopter_value = 4
type_helicopter_description = "Normal helicopter with tail rotor."

type_antenna_tracker = mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER
type_antenna_tracker_value = 5
type_antenna_tracker_description = "Ground installation"

type_gcs = mavutil.mavlink.MAV_TYPE_GCS
type_gcs_value = 6
type_gcs_description = "Operator control unit / ground control station"

type_airship = mavutil.mavlink.MAV_TYPE_AIRSHIP
type_airship_value = 7
type_airship_description = "Airship, controlled"

type_free_balloon = mavutil.mavlink.MAV_TYPE_FREE_BALLOON
type_free_balloon_value = 8
type_free_balloon_description = "Free balloon, uncontrolled"

type_rocket = mavutil.mavlink.MAV_TYPE_ROCKET
type_rocket_value = 9
type_rocket_description = "Rocket"

type_ground_rover = mavutil.mavlink.MAV_TYPE_GROUND_ROVER
type_ground_rover_value = 10
type_ground_rover_description = "Ground rover"

type_surface_boat = mavutil.mavlink.MAV_TYPE_SURFACE_BOAT
type_surface_boat_value = 11
type_surface_boat_description = "Surface vessel, boat, ship"

type_submarine = mavutil.mavlink.MAV_TYPE_SUBMARINE
type_submarine_value = 12
type_submarine_description = "Submarine"

type_hexarotor = mavutil.mavlink.MAV_TYPE_HEXAROTOR
type_hexarotor_value = 13
type_hexarotor_description = "Hexarotor"

type_octorotor = mavutil.mavlink.MAV_TYPE_OCTOROTOR
type_octorotor_value = 14
type_octorotor_description = "Octorotor"

type_tricopter = mavutil.mavlink.MAV_TYPE_TRICOPTER
type_tricopter_value = 15
type_tricopter_description = "Tricopter"

type_flapping_wing = mavutil.mavlink.MAV_TYPE_FLAPPING_WING
type_flapping_wing_value = 16
type_flapping_wing_description = "Flapping wing"

type_kite = mavutil.mavlink.MAV_TYPE_KITE
type_kite_value = 17
type_kite_description = "Kite"

type_onboard_controller = mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER
type_onboard_controller_value = 18
type_onboard_controller_description = "Onboard companion controller"

type_vtol_duorotor = mavutil.mavlink.MAV_TYPE_VTOL_DUOROTOR
type_vtol_duorotor_value = 19
type_vtol_duorotor_description = "Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter."

type_vtol_quadrotor = mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR
type_vtol_quadrotor_value = 20
type_vtol_quadrotor_description = "Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter."

type_vtol_tiltrotor = mavutil.mavlink.MAV_TYPE_VTOL_TILTROTOR
type_vtol_tiltrotor_value = 21
type_vtol_tiltrotor_description = "Tiltrotor VTOL"

type_vtol_reserved2 = mavutil.mavlink.MAV_TYPE_VTOL_RESERVED2
type_vtol_reserved2_value = 22
type_vtol_reserved2_description = "VTOL reserved 2"

type_vtol_reserved3 = mavutil.mavlink.MAV_TYPE_VTOL_RESERVED3
type_vtol_reserved3_value = 23
type_vtol_reserved3_description = "VTOL reserved 3"

type_vtol_reserved4 = mavutil.mavlink.MAV_TYPE_VTOL_RESERVED4
type_vtol_reserved4_value = 24
type_vtol_reserved4_description = "VTOL reserved 4"

type_vtol_reserved5 = mavutil.mavlink.MAV_TYPE_VTOL_RESERVED5
type_vtol_reserved5_value = 25
type_vtol_reserved5_description = "VTOL reserved 5"

type_gimbal = mavutil.mavlink.MAV_TYPE_GIMBAL
type_gimbal_value = 26
type_gimbal_description = "Gimbal"

type_adsb = mavutil.mavlink.MAV_TYPE_ADSB
type_adsb_value = 27
type_adsb_description = "ADSB system"

type_parafoil = mavutil.mavlink.MAV_TYPE_PARAFOIL
type_parafoil_value = 28
type_parafoil_description = "Steerable, nonrigid airfoil"

type_dodecarotor = mavutil.mavlink.MAV_TYPE_DODECAROTOR
type_dodecarotor_value = 29
type_dodecarotor_description = "Dodecarotor"

type_camera = mavutil.mavlink.MAV_TYPE_CAMERA
type_camera_value = 30
type_camera_description = "Camera"

type_charging_station = mavutil.mavlink.MAV_TYPE_CHARGING_STATION
type_charging_station_value = 31
type_charging_station_description = "Charging station"

type_flarm = mavutil.mavlink.MAV_TYPE_FLARM
type_flarm_value = 32
type_flarm_description = "FLARM collision avoidance system"

type_servo = mavutil.mavlink.MAV_TYPE_SERVO
type_servo_value = 33
type_servo_description = "Servo"

type_odid = mavutil.mavlink.MAV_TYPE_ODID
type_odid_value = 34
type_odid_description = "Open Drone ID. See https://mavlink.io/en/services/opendroneid.html."

type_decarotor = mavutil.mavlink.MAV_TYPE_DECAROTOR
type_decarotor_value = 35
type_decarotor_description = "Decarotor"

type_battery = mavutil.mavlink.MAV_TYPE_BATTERY
type_battery_value = 36
type_battery_description = "Battery"

type_parachute = mavutil.mavlink.MAV_TYPE_PARACHUTE
type_parachute_value = 37
type_parachute_description = "Parachute"

mode_flag_safety_armed = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
mode_flag_safety_armed_value = 128
mode_flag_safety_armed_description = "0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state."

mode_flag_manual_input_enabled = mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
mode_flag_manual_input_enabled_value = 64
mode_flag_manual_input_enabled_description = "0b01000000 remote control input is enabled."

mode_flag_hil_enabled = mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED
mode_flag_hil_enabled_value = 32
mode_flag_hil_enabled_description = "0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational."

mode_flag_stabilize_enabled = mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED
mode_flag_stabilize_enabled_value = 16
mode_flag_stabilize_enabled_description = "0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around."

mode_flag_guided_enabled = mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
mode_flag_guided_enabled_value = 8
mode_flag_guided_enabled_description = "0b00001000 guided mode enabled, system flies waypoints / mission items."

mode_flag_auto_enabled = mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED
mode_flag_auto_enabled_value = 4
mode_flag_auto_enabled_description = "0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation."

mode_flag_test_enabled = mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED
mode_flag_test_enabled_value = 2
mode_flag_test_enabled_description = "0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations."

mode_flag_custom_mode_enabled = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
mode_flag_custom_mode_enabled_value = 1
mode_flag_custom_mode_enabled_description = "0b00000001 Reserved for future use."

mode_flag_decode_position_safety = mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY
mode_flag_decode_position_safety_value = 128
mode_flag_decode_position_safety_description = "First bit:  10000000"

mode_flag_decode_position_manual = mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_MANUAL
mode_flag_decode_position_manual_value = 64
mode_flag_decode_position_manual_description = "Second bit: 01000000"

mode_flag_decode_position_hil = mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_HIL
mode_flag_decode_position_hil_value = 32
mode_flag_decode_position_hil_description = "Third bit:  00100000"

mode_flag_decode_position_stabilize = mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_STABILIZE
mode_flag_decode_position_stabilize_value = 16
mode_flag_decode_position_stabilize_description = "Fourth bit: 00010000"

mode_flag_decode_position_guided = mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_GUIDED
mode_flag_decode_position_guided_value = 8
mode_flag_decode_position_guided_description = "Fifth bit:  00001000"

mode_flag_decode_position_auto = mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_AUTO
mode_flag_decode_position_auto_value = 4
mode_flag_decode_position_auto_description = "Sixth bit:   00000100"

mode_flag_decode_position_test = mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_TEST
mode_flag_decode_position_test_value = 2
mode_flag_decode_position_test_description = "Seventh bit: 00000010"

mode_flag_decode_position_custom_mode = mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE
mode_flag_decode_position_custom_mode_value = 1
mode_flag_decode_position_custom_mode_description = "Eighth bit: 00000001"

state_uninit = mavutil.mavlink.MAV_STATE_UNINIT
state_uninit_value = 0
state_uninit_description = "Uninitialized system, state is unknown."

state_boot = mavutil.mavlink.MAV_STATE_BOOT
state_boot_value = 1
state_boot_description = "System is booting up."

state_calibrating = mavutil.mavlink.MAV_STATE_CALIBRATING
state_calibrating_value = 2
state_calibrating_description = "System is calibrating and not flight-ready."

state_standby = mavutil.mavlink.MAV_STATE_STANDBY
state_standby_value = 3
state_standby_description = "System is grounded and on standby. It can be launched any time."

state_active = mavutil.mavlink.MAV_STATE_ACTIVE
state_active_value = 4
state_active_description = "System is active and might be already airborne. Motors are engaged."

state_critical = mavutil.mavlink.MAV_STATE_CRITICAL
state_critical_value = 5
state_critical_description = "System is in a non-normal flight mode. It can however still navigate."

state_emergency = mavutil.mavlink.MAV_STATE_EMERGENCY
state_emergency_value = 6
state_emergency_description = "System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down."

state_poweroff = mavutil.mavlink.MAV_STATE_POWEROFF
state_poweroff_value = 7
state_poweroff_description = "System just initialized its power-down sequence, will shut down now."

state_flight_termination = mavutil.mavlink.MAV_STATE_FLIGHT_TERMINATION
state_flight_termination_value = 8
state_flight_termination_description = "System is terminating itself."

comp_id_all = mavutil.mavlink.MAV_COMP_ID_ALL
comp_id_all_value = 0
comp_id_all_description = "Target id (target_component) used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces. Note: This is not a valid *source* component id for a message."

comp_id_autopilot1 = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
comp_id_autopilot1_value = 1
comp_id_autopilot1_description = "System flight controller component ('autopilot'). Only one autopilot is expected in a particular system."

comp_id_user1 = mavutil.mavlink.MAV_COMP_ID_USER1
comp_id_user1_value = 25
comp_id_user1_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user2 = mavutil.mavlink.MAV_COMP_ID_USER2
comp_id_user2_value = 26
comp_id_user2_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user3 = mavutil.mavlink.MAV_COMP_ID_USER3
comp_id_user3_value = 27
comp_id_user3_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user4 = mavutil.mavlink.MAV_COMP_ID_USER4
comp_id_user4_value = 28
comp_id_user4_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user5 = mavutil.mavlink.MAV_COMP_ID_USER5
comp_id_user5_value = 29
comp_id_user5_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user6 = mavutil.mavlink.MAV_COMP_ID_USER6
comp_id_user6_value = 30
comp_id_user6_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user7 = mavutil.mavlink.MAV_COMP_ID_USER7
comp_id_user7_value = 31
comp_id_user7_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user8 = mavutil.mavlink.MAV_COMP_ID_USER8
comp_id_user8_value = 32
comp_id_user8_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user9 = mavutil.mavlink.MAV_COMP_ID_USER9
comp_id_user9_value = 33
comp_id_user9_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user10 = mavutil.mavlink.MAV_COMP_ID_USER10
comp_id_user10_value = 34
comp_id_user10_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user11 = mavutil.mavlink.MAV_COMP_ID_USER11
comp_id_user11_value = 35
comp_id_user11_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user12 = mavutil.mavlink.MAV_COMP_ID_USER12
comp_id_user12_value = 36
comp_id_user12_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user13 = mavutil.mavlink.MAV_COMP_ID_USER13
comp_id_user13_value = 37
comp_id_user13_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user14 = mavutil.mavlink.MAV_COMP_ID_USER14
comp_id_user14_value = 38
comp_id_user14_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user15 = mavutil.mavlink.MAV_COMP_ID_USER15
comp_id_user15_value = 39
comp_id_user15_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user16 = mavutil.mavlink.MAV_COMP_ID_USER16
comp_id_user16_value = 40
comp_id_user16_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user17 = mavutil.mavlink.MAV_COMP_ID_USER17
comp_id_user17_value = 41
comp_id_user17_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user18 = mavutil.mavlink.MAV_COMP_ID_USER18
comp_id_user18_value = 42
comp_id_user18_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user19 = mavutil.mavlink.MAV_COMP_ID_USER19
comp_id_user19_value = 43
comp_id_user19_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user20 = mavutil.mavlink.MAV_COMP_ID_USER20
comp_id_user20_value = 44
comp_id_user20_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user21 = mavutil.mavlink.MAV_COMP_ID_USER21
comp_id_user21_value = 45
comp_id_user21_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user22 = mavutil.mavlink.MAV_COMP_ID_USER22
comp_id_user22_value = 46
comp_id_user22_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user23 = mavutil.mavlink.MAV_COMP_ID_USER23
comp_id_user23_value = 47
comp_id_user23_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user24 = mavutil.mavlink.MAV_COMP_ID_USER24
comp_id_user24_value = 48
comp_id_user24_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user25 = mavutil.mavlink.MAV_COMP_ID_USER25
comp_id_user25_value = 49
comp_id_user25_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user26 = mavutil.mavlink.MAV_COMP_ID_USER26
comp_id_user26_value = 50
comp_id_user26_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user27 = mavutil.mavlink.MAV_COMP_ID_USER27
comp_id_user27_value = 51
comp_id_user27_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user28 = mavutil.mavlink.MAV_COMP_ID_USER28
comp_id_user28_value = 52
comp_id_user28_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user29 = mavutil.mavlink.MAV_COMP_ID_USER29
comp_id_user29_value = 53
comp_id_user29_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user30 = mavutil.mavlink.MAV_COMP_ID_USER30
comp_id_user30_value = 54
comp_id_user30_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user31 = mavutil.mavlink.MAV_COMP_ID_USER31
comp_id_user31_value = 55
comp_id_user31_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user32 = mavutil.mavlink.MAV_COMP_ID_USER32
comp_id_user32_value = 56
comp_id_user32_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user33 = mavutil.mavlink.MAV_COMP_ID_USER33
comp_id_user33_value = 57
comp_id_user33_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user34 = mavutil.mavlink.MAV_COMP_ID_USER34
comp_id_user34_value = 58
comp_id_user34_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user35 = mavutil.mavlink.MAV_COMP_ID_USER35
comp_id_user35_value = 59
comp_id_user35_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user36 = mavutil.mavlink.MAV_COMP_ID_USER36
comp_id_user36_value = 60
comp_id_user36_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user37 = mavutil.mavlink.MAV_COMP_ID_USER37
comp_id_user37_value = 61
comp_id_user37_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user38 = mavutil.mavlink.MAV_COMP_ID_USER38
comp_id_user38_value = 62
comp_id_user38_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user39 = mavutil.mavlink.MAV_COMP_ID_USER39
comp_id_user39_value = 63
comp_id_user39_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user40 = mavutil.mavlink.MAV_COMP_ID_USER40
comp_id_user40_value = 64
comp_id_user40_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user41 = mavutil.mavlink.MAV_COMP_ID_USER41
comp_id_user41_value = 65
comp_id_user41_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user42 = mavutil.mavlink.MAV_COMP_ID_USER42
comp_id_user42_value = 66
comp_id_user42_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user43 = mavutil.mavlink.MAV_COMP_ID_USER43
comp_id_user43_value = 67
comp_id_user43_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_telemetry_radio = mavutil.mavlink.MAV_COMP_ID_TELEMETRY_RADIO
comp_id_telemetry_radio_value = 68
comp_id_telemetry_radio_description = "Telemetry radio (e.g. SiK radio, or other component that emits RADIO_STATUS messages)."

comp_id_user45 = mavutil.mavlink.MAV_COMP_ID_USER45
comp_id_user45_value = 69
comp_id_user45_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user46 = mavutil.mavlink.MAV_COMP_ID_USER46
comp_id_user46_value = 70
comp_id_user46_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user47 = mavutil.mavlink.MAV_COMP_ID_USER47
comp_id_user47_value = 71
comp_id_user47_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user48 = mavutil.mavlink.MAV_COMP_ID_USER48
comp_id_user48_value = 72
comp_id_user48_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user49 = mavutil.mavlink.MAV_COMP_ID_USER49
comp_id_user49_value = 73
comp_id_user49_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user50 = mavutil.mavlink.MAV_COMP_ID_USER50
comp_id_user50_value = 74
comp_id_user50_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user51 = mavutil.mavlink.MAV_COMP_ID_USER51
comp_id_user51_value = 75
comp_id_user51_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user52 = mavutil.mavlink.MAV_COMP_ID_USER52
comp_id_user52_value = 76
comp_id_user52_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user53 = mavutil.mavlink.MAV_COMP_ID_USER53
comp_id_user53_value = 77
comp_id_user53_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user54 = mavutil.mavlink.MAV_COMP_ID_USER54
comp_id_user54_value = 78
comp_id_user54_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user55 = mavutil.mavlink.MAV_COMP_ID_USER55
comp_id_user55_value = 79
comp_id_user55_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user56 = mavutil.mavlink.MAV_COMP_ID_USER56
comp_id_user56_value = 80
comp_id_user56_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user57 = mavutil.mavlink.MAV_COMP_ID_USER57
comp_id_user57_value = 81
comp_id_user57_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user58 = mavutil.mavlink.MAV_COMP_ID_USER58
comp_id_user58_value = 82
comp_id_user58_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user59 = mavutil.mavlink.MAV_COMP_ID_USER59
comp_id_user59_value = 83
comp_id_user59_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user60 = mavutil.mavlink.MAV_COMP_ID_USER60
comp_id_user60_value = 84
comp_id_user60_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user61 = mavutil.mavlink.MAV_COMP_ID_USER61
comp_id_user61_value = 85
comp_id_user61_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user62 = mavutil.mavlink.MAV_COMP_ID_USER62
comp_id_user62_value = 86
comp_id_user62_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user63 = mavutil.mavlink.MAV_COMP_ID_USER63
comp_id_user63_value = 87
comp_id_user63_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user64 = mavutil.mavlink.MAV_COMP_ID_USER64
comp_id_user64_value = 88
comp_id_user64_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user65 = mavutil.mavlink.MAV_COMP_ID_USER65
comp_id_user65_value = 89
comp_id_user65_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user66 = mavutil.mavlink.MAV_COMP_ID_USER66
comp_id_user66_value = 90
comp_id_user66_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user67 = mavutil.mavlink.MAV_COMP_ID_USER67
comp_id_user67_value = 91
comp_id_user67_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user68 = mavutil.mavlink.MAV_COMP_ID_USER68
comp_id_user68_value = 92
comp_id_user68_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user69 = mavutil.mavlink.MAV_COMP_ID_USER69
comp_id_user69_value = 93
comp_id_user69_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user70 = mavutil.mavlink.MAV_COMP_ID_USER70
comp_id_user70_value = 94
comp_id_user70_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user71 = mavutil.mavlink.MAV_COMP_ID_USER71
comp_id_user71_value = 95
comp_id_user71_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user72 = mavutil.mavlink.MAV_COMP_ID_USER72
comp_id_user72_value = 96
comp_id_user72_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user73 = mavutil.mavlink.MAV_COMP_ID_USER73
comp_id_user73_value = 97
comp_id_user73_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user74 = mavutil.mavlink.MAV_COMP_ID_USER74
comp_id_user74_value = 98
comp_id_user74_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_user75 = mavutil.mavlink.MAV_COMP_ID_USER75
comp_id_user75_value = 99
comp_id_user75_description = "Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network."

comp_id_camera = mavutil.mavlink.MAV_COMP_ID_CAMERA
comp_id_camera_value = 100
comp_id_camera_description = "Camera #1."

comp_id_camera2 = mavutil.mavlink.MAV_COMP_ID_CAMERA2
comp_id_camera2_value = 101
comp_id_camera2_description = "Camera #2."

comp_id_camera3 = mavutil.mavlink.MAV_COMP_ID_CAMERA3
comp_id_camera3_value = 102
comp_id_camera3_description = "Camera #3."

comp_id_camera4 = mavutil.mavlink.MAV_COMP_ID_CAMERA4
comp_id_camera4_value = 103
comp_id_camera4_description = "Camera #4."

comp_id_camera5 = mavutil.mavlink.MAV_COMP_ID_CAMERA5
comp_id_camera5_value = 104
comp_id_camera5_description = "Camera #5."

comp_id_camera6 = mavutil.mavlink.MAV_COMP_ID_CAMERA6
comp_id_camera6_value = 105
comp_id_camera6_description = "Camera #6."

comp_id_servo1 = mavutil.mavlink.MAV_COMP_ID_SERVO1
comp_id_servo1_value = 140
comp_id_servo1_description = "Servo #1."

comp_id_servo2 = mavutil.mavlink.MAV_COMP_ID_SERVO2
comp_id_servo2_value = 141
comp_id_servo2_description = "Servo #2."

comp_id_servo3 = mavutil.mavlink.MAV_COMP_ID_SERVO3
comp_id_servo3_value = 142
comp_id_servo3_description = "Servo #3."

comp_id_servo4 = mavutil.mavlink.MAV_COMP_ID_SERVO4
comp_id_servo4_value = 143
comp_id_servo4_description = "Servo #4."

comp_id_servo5 = mavutil.mavlink.MAV_COMP_ID_SERVO5
comp_id_servo5_value = 144
comp_id_servo5_description = "Servo #5."

comp_id_servo6 = mavutil.mavlink.MAV_COMP_ID_SERVO6
comp_id_servo6_value = 145
comp_id_servo6_description = "Servo #6."

comp_id_servo7 = mavutil.mavlink.MAV_COMP_ID_SERVO7
comp_id_servo7_value = 146
comp_id_servo7_description = "Servo #7."

comp_id_servo8 = mavutil.mavlink.MAV_COMP_ID_SERVO8
comp_id_servo8_value = 147
comp_id_servo8_description = "Servo #8."

comp_id_servo9 = mavutil.mavlink.MAV_COMP_ID_SERVO9
comp_id_servo9_value = 148
comp_id_servo9_description = "Servo #9."

comp_id_servo10 = mavutil.mavlink.MAV_COMP_ID_SERVO10
comp_id_servo10_value = 149
comp_id_servo10_description = "Servo #10."

comp_id_servo11 = mavutil.mavlink.MAV_COMP_ID_SERVO11
comp_id_servo11_value = 150
comp_id_servo11_description = "Servo #11."

comp_id_servo12 = mavutil.mavlink.MAV_COMP_ID_SERVO12
comp_id_servo12_value = 151
comp_id_servo12_description = "Servo #12."

comp_id_servo13 = mavutil.mavlink.MAV_COMP_ID_SERVO13
comp_id_servo13_value = 152
comp_id_servo13_description = "Servo #13."

comp_id_servo14 = mavutil.mavlink.MAV_COMP_ID_SERVO14
comp_id_servo14_value = 153
comp_id_servo14_description = "Servo #14."

comp_id_gimbal = mavutil.mavlink.MAV_COMP_ID_GIMBAL
comp_id_gimbal_value = 154
comp_id_gimbal_description = "Gimbal #1."

comp_id_log = mavutil.mavlink.MAV_COMP_ID_LOG
comp_id_log_value = 155
comp_id_log_description = "Logging component."

comp_id_adsb = mavutil.mavlink.MAV_COMP_ID_ADSB
comp_id_adsb_value = 156
comp_id_adsb_description = "Automatic Dependent Surveillance-Broadcast (ADS-B) component."

comp_id_osd = mavutil.mavlink.MAV_COMP_ID_OSD
comp_id_osd_value = 157
comp_id_osd_description = "On Screen Display (OSD) devices for video links."

comp_id_peripheral = mavutil.mavlink.MAV_COMP_ID_PERIPHERAL
comp_id_peripheral_value = 158
comp_id_peripheral_description = "Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice."

comp_id_qx1_gimbal = mavutil.mavlink.MAV_COMP_ID_QX1_GIMBAL
# Deprecated since 2018-11 and replaced by MAV_COMP_ID_GIMBAL
comp_id_qx1_gimbal_value = 159
comp_id_qx1_gimbal_description = "Gimbal ID for QX1."

comp_id_flarm = mavutil.mavlink.MAV_COMP_ID_FLARM
comp_id_flarm_value = 160
comp_id_flarm_description = "FLARM collision alert component."

comp_id_parachute = mavutil.mavlink.MAV_COMP_ID_PARACHUTE
comp_id_parachute_value = 161
comp_id_parachute_description = "Parachute component."

comp_id_gimbal2 = mavutil.mavlink.MAV_COMP_ID_GIMBAL2
comp_id_gimbal2_value = 171
comp_id_gimbal2_description = "Gimbal #2."

comp_id_gimbal3 = mavutil.mavlink.MAV_COMP_ID_GIMBAL3
comp_id_gimbal3_value = 172
comp_id_gimbal3_description = "Gimbal #3."

comp_id_gimbal4 = mavutil.mavlink.MAV_COMP_ID_GIMBAL4
comp_id_gimbal4_value = 173
comp_id_gimbal4_description = "Gimbal #4"

comp_id_gimbal5 = mavutil.mavlink.MAV_COMP_ID_GIMBAL5
comp_id_gimbal5_value = 174
comp_id_gimbal5_description = "Gimbal #5."

comp_id_gimbal6 = mavutil.mavlink.MAV_COMP_ID_GIMBAL6
comp_id_gimbal6_value = 175
comp_id_gimbal6_description = "Gimbal #6."

comp_id_battery = mavutil.mavlink.MAV_COMP_ID_BATTERY
comp_id_battery_value = 180
comp_id_battery_description = "Battery #1."

comp_id_battery2 = mavutil.mavlink.MAV_COMP_ID_BATTERY2
comp_id_battery2_value = 181
comp_id_battery2_description = "Battery #2."

comp_id_missionplanner = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
comp_id_missionplanner_value = 190
comp_id_missionplanner_description = "Component that can generate/supply a mission flight plan (e.g. GCS or developer API)."

comp_id_onboard_computer = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
comp_id_onboard_computer_value = 191
comp_id_onboard_computer_description = "Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on."

comp_id_onboard_computer2 = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER2
comp_id_onboard_computer2_value = 192
comp_id_onboard_computer2_description = "Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on."

comp_id_onboard_computer3 = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER3
comp_id_onboard_computer3_value = 193
comp_id_onboard_computer3_description = "Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on."

comp_id_onboard_computer4 = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER4
comp_id_onboard_computer4_value = 194
comp_id_onboard_computer4_description = "Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on."

comp_id_pathplanner = mavutil.mavlink.MAV_COMP_ID_PATHPLANNER
comp_id_pathplanner_value = 195
comp_id_pathplanner_description = "Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap, shortest path, cost, etc.)."

comp_id_obstacle_avoidance = mavutil.mavlink.MAV_COMP_ID_OBSTACLE_AVOIDANCE
comp_id_obstacle_avoidance_value = 196
comp_id_obstacle_avoidance_description = "Component that plans a collision free path between two points."

comp_id_visual_inertial_odometry = mavutil.mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
comp_id_visual_inertial_odometry_value = 197
comp_id_visual_inertial_odometry_description = "Component that provides position estimates using VIO techniques."

comp_id_pairing_manager = mavutil.mavlink.MAV_COMP_ID_PAIRING_MANAGER
comp_id_pairing_manager_value = 198
comp_id_pairing_manager_description = "Component that manages pairing of vehicle and GCS."

comp_id_imu = mavutil.mavlink.MAV_COMP_ID_IMU
comp_id_imu_value = 200
comp_id_imu_description = "Inertial Measurement Unit (IMU) #1."

comp_id_imu_2 = mavutil.mavlink.MAV_COMP_ID_IMU_2
comp_id_imu_2_value = 201
comp_id_imu_2_description = "Inertial Measurement Unit (IMU) #2."

comp_id_imu_3 = mavutil.mavlink.MAV_COMP_ID_IMU_3
comp_id_imu_3_value = 202
comp_id_imu_3_description = "Inertial Measurement Unit (IMU) #3."

comp_id_gps = mavutil.mavlink.MAV_COMP_ID_GPS
comp_id_gps_value = 220
comp_id_gps_description = "GPS #1."

comp_id_gps2 = mavutil.mavlink.MAV_COMP_ID_GPS2
comp_id_gps2_value = 221
comp_id_gps2_description = "GPS #2."

comp_id_odid_txrx_1 = mavutil.mavlink.MAV_COMP_ID_ODID_TXRX_1
comp_id_odid_txrx_1_value = 236
comp_id_odid_txrx_1_description = "Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet)."

comp_id_odid_txrx_2 = mavutil.mavlink.MAV_COMP_ID_ODID_TXRX_2
comp_id_odid_txrx_2_value = 237
comp_id_odid_txrx_2_description = "Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet)."

comp_id_odid_txrx_3 = mavutil.mavlink.MAV_COMP_ID_ODID_TXRX_3
comp_id_odid_txrx_3_value = 238
comp_id_odid_txrx_3_description = "Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet)."

comp_id_udp_bridge = mavutil.mavlink.MAV_COMP_ID_UDP_BRIDGE
comp_id_udp_bridge_value = 240
comp_id_udp_bridge_description = "Component to bridge MAVLink to UDP (i.e. from a UART)."

comp_id_uart_bridge = mavutil.mavlink.MAV_COMP_ID_UART_BRIDGE
comp_id_uart_bridge_value = 241
comp_id_uart_bridge_description = "Component to bridge to UART (i.e. from UDP)."

comp_id_tunnel_node = mavutil.mavlink.MAV_COMP_ID_TUNNEL_NODE
comp_id_tunnel_node_value = 242
comp_id_tunnel_node_description = "Component handling TUNNEL messages (e.g. vendor specific GUI of a component)."

comp_id_system_control = mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL
# Deprecated since 2018-11 and replaced by MAV_COMP_ID_ALL
comp_id_system_control_value = 250
comp_id_system_control_description = "Component for handling system messages (e.g. to ARM, takeoff, etc.)."

