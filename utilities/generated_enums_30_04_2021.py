from pymavlink import mavutil

mav_autopilot = mavutil.mavlink.MAV_AUTOPILOT
# name: MAV_AUTOPILOT_GENERIC
# value: 0
# description: Generic autopilot, full support for everything

# name: MAV_AUTOPILOT_RESERVED
# value: 1
# description: Reserved for future use.

# name: MAV_AUTOPILOT_SLUGS
# value: 2
# description: SLUGS autopilot, http://slugsuav.soe.ucsc.edu

# name: MAV_AUTOPILOT_ARDUPILOTMEGA
# value: 3
# description: ArduPilot - Plane/Copter/Rover/Sub/Tracker, https://ardupilot.org

# name: MAV_AUTOPILOT_OPENPILOT
# value: 4
# description: OpenPilot, http://openpilot.org

# name: MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY
# value: 5
# description: Generic autopilot only supporting simple waypoints

# name: MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY
# value: 6
# description: Generic autopilot supporting waypoints and other simple navigation commands

# name: MAV_AUTOPILOT_GENERIC_MISSION_FULL
# value: 7
# description: Generic autopilot supporting the full mission command set

# name: MAV_AUTOPILOT_INVALID
# value: 8
# description: No valid autopilot, e.g. a GCS or other MAVLink component

# name: MAV_AUTOPILOT_PPZ
# value: 9
# description: PPZ UAV - http://nongnu.org/paparazzi

# name: MAV_AUTOPILOT_UDB
# value: 10
# description: UAV Dev Board

# name: MAV_AUTOPILOT_FP
# value: 11
# description: FlexiPilot

# name: MAV_AUTOPILOT_PX4
# value: 12
# description: PX4 Autopilot - http://px4.io/

# name: MAV_AUTOPILOT_SMACCMPILOT
# value: 13
# description: SMACCMPilot - http://smaccmpilot.org

# name: MAV_AUTOPILOT_AUTOQUAD
# value: 14
# description: AutoQuad -- http://autoquad.org

# name: MAV_AUTOPILOT_ARMAZILA
# value: 15
# description: Armazila -- http://armazila.com

# name: MAV_AUTOPILOT_AEROB
# value: 16
# description: Aerob -- http://aerob.ru

# name: MAV_AUTOPILOT_ASLUAV
# value: 17
# description: ASLUAV autopilot -- http://www.asl.ethz.ch

# name: MAV_AUTOPILOT_SMARTAP
# value: 18
# description: SmartAP Autopilot - http://sky-drones.com

# name: MAV_AUTOPILOT_AIRRAILS
# value: 19
# description: AirRails - http://uaventure.com

 
mav_type = mavutil.mavlink.MAV_TYPE
# name: MAV_TYPE_GENERIC
# value: 0
# description: Generic micro air vehicle

# name: MAV_TYPE_FIXED_WING
# value: 1
# description: Fixed wing aircraft.

# name: MAV_TYPE_QUADROTOR
# value: 2
# description: Quadrotor

# name: MAV_TYPE_COAXIAL
# value: 3
# description: Coaxial helicopter

# name: MAV_TYPE_HELICOPTER
# value: 4
# description: Normal helicopter with tail rotor.

# name: MAV_TYPE_ANTENNA_TRACKER
# value: 5
# description: Ground installation

# name: MAV_TYPE_GCS
# value: 6
# description: Operator control unit / ground control station

# name: MAV_TYPE_AIRSHIP
# value: 7
# description: Airship, controlled

# name: MAV_TYPE_FREE_BALLOON
# value: 8
# description: Free balloon, uncontrolled

# name: MAV_TYPE_ROCKET
# value: 9
# description: Rocket

# name: MAV_TYPE_GROUND_ROVER
# value: 10
# description: Ground rover

# name: MAV_TYPE_SURFACE_BOAT
# value: 11
# description: Surface vessel, boat, ship

# name: MAV_TYPE_SUBMARINE
# value: 12
# description: Submarine

# name: MAV_TYPE_HEXAROTOR
# value: 13
# description: Hexarotor

# name: MAV_TYPE_OCTOROTOR
# value: 14
# description: Octorotor

# name: MAV_TYPE_TRICOPTER
# value: 15
# description: Tricopter

# name: MAV_TYPE_FLAPPING_WING
# value: 16
# description: Flapping wing

# name: MAV_TYPE_KITE
# value: 17
# description: Kite

# name: MAV_TYPE_ONBOARD_CONTROLLER
# value: 18
# description: Onboard companion controller

# name: MAV_TYPE_VTOL_DUOROTOR
# value: 19
# description: Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.

# name: MAV_TYPE_VTOL_QUADROTOR
# value: 20
# description: Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.

# name: MAV_TYPE_VTOL_TILTROTOR
# value: 21
# description: Tiltrotor VTOL

# name: MAV_TYPE_VTOL_RESERVED2
# value: 22
# description: VTOL reserved 2

# name: MAV_TYPE_VTOL_RESERVED3
# value: 23
# description: VTOL reserved 3

# name: MAV_TYPE_VTOL_RESERVED4
# value: 24
# description: VTOL reserved 4

# name: MAV_TYPE_VTOL_RESERVED5
# value: 25
# description: VTOL reserved 5

# name: MAV_TYPE_GIMBAL
# value: 26
# description: Gimbal

# name: MAV_TYPE_ADSB
# value: 27
# description: ADSB system

# name: MAV_TYPE_PARAFOIL
# value: 28
# description: Steerable, nonrigid airfoil

# name: MAV_TYPE_DODECAROTOR
# value: 29
# description: Dodecarotor

# name: MAV_TYPE_CAMERA
# value: 30
# description: Camera

# name: MAV_TYPE_CHARGING_STATION
# value: 31
# description: Charging station

# name: MAV_TYPE_FLARM
# value: 32
# description: FLARM collision avoidance system

# name: MAV_TYPE_SERVO
# value: 33
# description: Servo

# name: MAV_TYPE_ODID
# value: 34
# description: Open Drone ID. See https://mavlink.io/en/services/opendroneid.html.

# name: MAV_TYPE_DECAROTOR
# value: 35
# description: Decarotor

# name: MAV_TYPE_BATTERY
# value: 36
# description: Battery

 
mav_mode_flag = mavutil.mavlink.MAV_MODE_FLAG
# name: MAV_MODE_FLAG_SAFETY_ARMED
# value: 128
# description: 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state.

# name: MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
# value: 64
# description: 0b01000000 remote control input is enabled.

# name: MAV_MODE_FLAG_HIL_ENABLED
# value: 32
# description: 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.

# name: MAV_MODE_FLAG_STABILIZE_ENABLED
# value: 16
# description: 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.

# name: MAV_MODE_FLAG_GUIDED_ENABLED
# value: 8
# description: 0b00001000 guided mode enabled, system flies waypoints / mission items.

# name: MAV_MODE_FLAG_AUTO_ENABLED
# value: 4
# description: 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.

# name: MAV_MODE_FLAG_TEST_ENABLED
# value: 2
# description: 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.

# name: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
# value: 1
# description: 0b00000001 Reserved for future use.

 
mav_mode_flag_decode_position = mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION
# name: MAV_MODE_FLAG_DECODE_POSITION_SAFETY
# value: 128
# description: First bit:  10000000

# name: MAV_MODE_FLAG_DECODE_POSITION_MANUAL
# value: 64
# description: Second bit: 01000000

# name: MAV_MODE_FLAG_DECODE_POSITION_HIL
# value: 32
# description: Third bit:  00100000

# name: MAV_MODE_FLAG_DECODE_POSITION_STABILIZE
# value: 16
# description: Fourth bit: 00010000

# name: MAV_MODE_FLAG_DECODE_POSITION_GUIDED
# value: 8
# description: Fifth bit:  00001000

# name: MAV_MODE_FLAG_DECODE_POSITION_AUTO
# value: 4
# description: Sixth bit:   00000100

# name: MAV_MODE_FLAG_DECODE_POSITION_TEST
# value: 2
# description: Seventh bit: 00000010

# name: MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE
# value: 1
# description: Eighth bit: 00000001

 
mav_state = mavutil.mavlink.MAV_STATE
# name: MAV_STATE_UNINIT
# value: 0
# description: Uninitialized system, state is unknown.

# name: MAV_STATE_BOOT
# value: 1
# description: System is booting up.

# name: MAV_STATE_CALIBRATING
# value: 2
# description: System is calibrating and not flight-ready.

# name: MAV_STATE_STANDBY
# value: 3
# description: System is grounded and on standby. It can be launched any time.

# name: MAV_STATE_ACTIVE
# value: 4
# description: System is active and might be already airborne. Motors are engaged.

# name: MAV_STATE_CRITICAL
# value: 5
# description: System is in a non-normal flight mode. It can however still navigate.

# name: MAV_STATE_EMERGENCY
# value: 6
# description: System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.

# name: MAV_STATE_POWEROFF
# value: 7
# description: System just initialized its power-down sequence, will shut down now.

# name: MAV_STATE_FLIGHT_TERMINATION
# value: 8
# description: System is terminating itself.

 
mav_component = mavutil.mavlink.MAV_COMPONENT
# Deprecated since 2018-11 and replaced by MAV_COMP_ID_ALL# Description # name: MAV_COMP_ID_ALL
# value: 0
# description: Target id (target_component) used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces. Note: This is not a valid *source* component id for a message.

# name: MAV_COMP_ID_AUTOPILOT1
# value: 1
# description: System flight controller component ("autopilot"). Only one autopilot is expected in a particular system.

# name: MAV_COMP_ID_USER1
# value: 25
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER2
# value: 26
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER3
# value: 27
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER4
# value: 28
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER5
# value: 29
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER6
# value: 30
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER7
# value: 31
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER8
# value: 32
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER9
# value: 33
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER10
# value: 34
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER11
# value: 35
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER12
# value: 36
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER13
# value: 37
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER14
# value: 38
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER15
# value: 39
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER16
# value: 40
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER17
# value: 41
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER18
# value: 42
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER19
# value: 43
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER20
# value: 44
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER21
# value: 45
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER22
# value: 46
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER23
# value: 47
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER24
# value: 48
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER25
# value: 49
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER26
# value: 50
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER27
# value: 51
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER28
# value: 52
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER29
# value: 53
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER30
# value: 54
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER31
# value: 55
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER32
# value: 56
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER33
# value: 57
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER34
# value: 58
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER35
# value: 59
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER36
# value: 60
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER37
# value: 61
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER38
# value: 62
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER39
# value: 63
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER40
# value: 64
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER41
# value: 65
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER42
# value: 66
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER43
# value: 67
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_TELEMETRY_RADIO
# value: 68
# description: Telemetry radio (e.g. SiK radio, or other component that emits RADIO_STATUS messages).

# name: MAV_COMP_ID_USER45
# value: 69
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER46
# value: 70
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER47
# value: 71
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER48
# value: 72
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER49
# value: 73
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER50
# value: 74
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER51
# value: 75
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER52
# value: 76
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER53
# value: 77
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER54
# value: 78
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER55
# value: 79
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER56
# value: 80
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER57
# value: 81
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER58
# value: 82
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER59
# value: 83
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER60
# value: 84
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER61
# value: 85
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER62
# value: 86
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER63
# value: 87
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER64
# value: 88
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER65
# value: 89
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER66
# value: 90
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER67
# value: 91
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER68
# value: 92
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER69
# value: 93
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER70
# value: 94
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER71
# value: 95
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER72
# value: 96
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER73
# value: 97
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER74
# value: 98
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_USER75
# value: 99
# description: Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.

# name: MAV_COMP_ID_CAMERA
# value: 100
# description: Camera #1.

# name: MAV_COMP_ID_CAMERA2
# value: 101
# description: Camera #2.

# name: MAV_COMP_ID_CAMERA3
# value: 102
# description: Camera #3.

# name: MAV_COMP_ID_CAMERA4
# value: 103
# description: Camera #4.

# name: MAV_COMP_ID_CAMERA5
# value: 104
# description: Camera #5.

# name: MAV_COMP_ID_CAMERA6
# value: 105
# description: Camera #6.

# name: MAV_COMP_ID_SERVO1
# value: 140
# description: Servo #1.

# name: MAV_COMP_ID_SERVO2
# value: 141
# description: Servo #2.

# name: MAV_COMP_ID_SERVO3
# value: 142
# description: Servo #3.

# name: MAV_COMP_ID_SERVO4
# value: 143
# description: Servo #4.

# name: MAV_COMP_ID_SERVO5
# value: 144
# description: Servo #5.

# name: MAV_COMP_ID_SERVO6
# value: 145
# description: Servo #6.

# name: MAV_COMP_ID_SERVO7
# value: 146
# description: Servo #7.

# name: MAV_COMP_ID_SERVO8
# value: 147
# description: Servo #8.

# name: MAV_COMP_ID_SERVO9
# value: 148
# description: Servo #9.

# name: MAV_COMP_ID_SERVO10
# value: 149
# description: Servo #10.

# name: MAV_COMP_ID_SERVO11
# value: 150
# description: Servo #11.

# name: MAV_COMP_ID_SERVO12
# value: 151
# description: Servo #12.

# name: MAV_COMP_ID_SERVO13
# value: 152
# description: Servo #13.

# name: MAV_COMP_ID_SERVO14
# value: 153
# description: Servo #14.

# name: MAV_COMP_ID_GIMBAL
# value: 154
# description: Gimbal #1.

# name: MAV_COMP_ID_LOG
# value: 155
# description: Logging component.

# name: MAV_COMP_ID_ADSB
# value: 156
# description: Automatic Dependent Surveillance-Broadcast (ADS-B) component.

# name: MAV_COMP_ID_OSD
# value: 157
# description: On Screen Display (OSD) devices for video links.

# name: MAV_COMP_ID_PERIPHERAL
# value: 158
# description: Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice.

# name: MAV_COMP_ID_QX1_GIMBAL
# value: 159
# description: Gimbal ID for QX1.

# name: MAV_COMP_ID_FLARM
# value: 160
# description: FLARM collision alert component.

# name: MAV_COMP_ID_GIMBAL2
# value: 171
# description: Gimbal #2.

# name: MAV_COMP_ID_GIMBAL3
# value: 172
# description: Gimbal #3.

# name: MAV_COMP_ID_GIMBAL4
# value: 173
# description: Gimbal #4

# name: MAV_COMP_ID_GIMBAL5
# value: 174
# description: Gimbal #5.

# name: MAV_COMP_ID_GIMBAL6
# value: 175
# description: Gimbal #6.

# name: MAV_COMP_ID_BATTERY
# value: 180
# description: Battery #1.

# name: MAV_COMP_ID_BATTERY2
# value: 181
# description: Battery #2.

# name: MAV_COMP_ID_MISSIONPLANNER
# value: 190
# description: Component that can generate/supply a mission flight plan (e.g. GCS or developer API).

# name: MAV_COMP_ID_ONBOARD_COMPUTER
# value: 191
# description: Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on.

# name: MAV_COMP_ID_PATHPLANNER
# value: 195
# description: Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap, shortest path, cost, etc.).

# name: MAV_COMP_ID_OBSTACLE_AVOIDANCE
# value: 196
# description: Component that plans a collision free path between two points.

# name: MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
# value: 197
# description: Component that provides position estimates using VIO techniques.

# name: MAV_COMP_ID_PAIRING_MANAGER
# value: 198
# description: Component that manages pairing of vehicle and GCS.

# name: MAV_COMP_ID_IMU
# value: 200
# description: Inertial Measurement Unit (IMU) #1.

# name: MAV_COMP_ID_IMU_2
# value: 201
# description: Inertial Measurement Unit (IMU) #2.

# name: MAV_COMP_ID_IMU_3
# value: 202
# description: Inertial Measurement Unit (IMU) #3.

# name: MAV_COMP_ID_GPS
# value: 220
# description: GPS #1.

# name: MAV_COMP_ID_GPS2
# value: 221
# description: GPS #2.

# name: MAV_COMP_ID_ODID_TXRX_1
# value: 236
# description: Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).

# name: MAV_COMP_ID_ODID_TXRX_2
# value: 237
# description: Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).

# name: MAV_COMP_ID_ODID_TXRX_3
# value: 238
# description: Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).

# name: MAV_COMP_ID_UDP_BRIDGE
# value: 240
# description: Component to bridge MAVLink to UDP (i.e. from a UART).

# name: MAV_COMP_ID_UART_BRIDGE
# value: 241
# description: Component to bridge to UART (i.e. from UDP).

# name: MAV_COMP_ID_TUNNEL_NODE
# value: 242
# description: Component handling TUNNEL messages (e.g. vendor specific GUI of a component).

# name: MAV_COMP_ID_SYSTEM_CONTROL
# value: 250
# description: Component for handling system messages (e.g. to ARM, takeoff, etc.).

 
