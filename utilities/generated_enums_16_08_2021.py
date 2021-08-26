from pymavlink import mavutil

firmware_version_type = mavutil.mavlink.FIRMWARE_VERSION_TYPE
# name: FIRMWARE_VERSION_TYPE_DEV
# value: 0
# description: development release

# name: FIRMWARE_VERSION_TYPE_ALPHA
# value: 64
# description: alpha release

# name: FIRMWARE_VERSION_TYPE_BETA
# value: 128
# description: beta release

# name: FIRMWARE_VERSION_TYPE_RC
# value: 192
# description: release candidate

# name: FIRMWARE_VERSION_TYPE_OFFICIAL
# value: 255
# description: official stable release

 
hl_failure_flag = mavutil.mavlink.HL_FAILURE_FLAG
# name: HL_FAILURE_FLAG_GPS
# value: 1
# description: GPS failure.

# name: HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE
# value: 2
# description: Differential pressure sensor failure.

# name: HL_FAILURE_FLAG_ABSOLUTE_PRESSURE
# value: 4
# description: Absolute pressure sensor failure.

# name: HL_FAILURE_FLAG_3D_ACCEL
# value: 8
# description: Accelerometer sensor failure.

# name: HL_FAILURE_FLAG_3D_GYRO
# value: 16
# description: Gyroscope sensor failure.

# name: HL_FAILURE_FLAG_3D_MAG
# value: 32
# description: Magnetometer sensor failure.

# name: HL_FAILURE_FLAG_TERRAIN
# value: 64
# description: Terrain subsystem failure.

# name: HL_FAILURE_FLAG_BATTERY
# value: 128
# description: Battery failure/critical low battery.

# name: HL_FAILURE_FLAG_RC_RECEIVER
# value: 256
# description: RC receiver failure/no rc connection.

# name: HL_FAILURE_FLAG_OFFBOARD_LINK
# value: 512
# description: Offboard link failure.

# name: HL_FAILURE_FLAG_ENGINE
# value: 1024
# description: Engine failure.

# name: HL_FAILURE_FLAG_GEOFENCE
# value: 2048
# description: Geofence violation.

# name: HL_FAILURE_FLAG_ESTIMATOR
# value: 4096
# description: Estimator failure, for example measurement rejection or large variances.

# name: HL_FAILURE_FLAG_MISSION
# value: 8192
# description: Mission failure.

 
mav_goto = mavutil.mavlink.MAV_GOTO
# name: MAV_GOTO_DO_HOLD
# value: 0
# description: Hold at the current position.

# name: MAV_GOTO_DO_CONTINUE
# value: 1
# description: Continue with the next item in mission execution.

# name: MAV_GOTO_HOLD_AT_CURRENT_POSITION
# value: 2
# description: Hold at the current position of the system

# name: MAV_GOTO_HOLD_AT_SPECIFIED_POSITION
# value: 3
# description: Hold at the position specified in the parameters of the DO_HOLD action

 
mav_mode = mavutil.mavlink.MAV_MODE
# name: MAV_MODE_PREFLIGHT
# value: 0
# description: System is not ready to fly, booting, calibrating, etc. No flag is set.

# name: MAV_MODE_STABILIZE_DISARMED
# value: 80
# description: System is allowed to be active, under assisted RC control.

# name: MAV_MODE_STABILIZE_ARMED
# value: 208
# description: System is allowed to be active, under assisted RC control.

# name: MAV_MODE_MANUAL_DISARMED
# value: 64
# description: System is allowed to be active, under manual (RC) control, no stabilization

# name: MAV_MODE_MANUAL_ARMED
# value: 192
# description: System is allowed to be active, under manual (RC) control, no stabilization

# name: MAV_MODE_GUIDED_DISARMED
# value: 88
# description: System is allowed to be active, under autonomous control, manual setpoint

# name: MAV_MODE_GUIDED_ARMED
# value: 216
# description: System is allowed to be active, under autonomous control, manual setpoint

# name: MAV_MODE_AUTO_DISARMED
# value: 92
# description: System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)

# name: MAV_MODE_AUTO_ARMED
# value: 220
# description: System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)

# name: MAV_MODE_TEST_DISARMED
# value: 66
# description: UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.

# name: MAV_MODE_TEST_ARMED
# value: 194
# description: UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.

 
mav_sys_status_sensor = mavutil.mavlink.MAV_SYS_STATUS_SENSOR
# name: MAV_SYS_STATUS_SENSOR_3D_GYRO
# value: 1
# description: 0x01 3D gyro

# name: MAV_SYS_STATUS_SENSOR_3D_ACCEL
# value: 2
# description: 0x02 3D accelerometer

# name: MAV_SYS_STATUS_SENSOR_3D_MAG
# value: 4
# description: 0x04 3D magnetometer

# name: MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
# value: 8
# description: 0x08 absolute pressure

# name: MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE
# value: 16
# description: 0x10 differential pressure

# name: MAV_SYS_STATUS_SENSOR_GPS
# value: 32
# description: 0x20 GPS

# name: MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW
# value: 64
# description: 0x40 optical flow

# name: MAV_SYS_STATUS_SENSOR_VISION_POSITION
# value: 128
# description: 0x80 computer vision position

# name: MAV_SYS_STATUS_SENSOR_LASER_POSITION
# value: 256
# description: 0x100 laser based position

# name: MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH
# value: 512
# description: 0x200 external ground truth (Vicon or Leica)

# name: MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL
# value: 1024
# description: 0x400 3D angular rate control

# name: MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION
# value: 2048
# description: 0x800 attitude stabilization

# name: MAV_SYS_STATUS_SENSOR_YAW_POSITION
# value: 4096
# description: 0x1000 yaw position

# name: MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL
# value: 8192
# description: 0x2000 z/altitude control

# name: MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL
# value: 16384
# description: 0x4000 x/y position control

# name: MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS
# value: 32768
# description: 0x8000 motor outputs / control

# name: MAV_SYS_STATUS_SENSOR_RC_RECEIVER
# value: 65536
# description: 0x10000 rc receiver

# name: MAV_SYS_STATUS_SENSOR_3D_GYRO2
# value: 131072
# description: 0x20000 2nd 3D gyro

# name: MAV_SYS_STATUS_SENSOR_3D_ACCEL2
# value: 262144
# description: 0x40000 2nd 3D accelerometer

# name: MAV_SYS_STATUS_SENSOR_3D_MAG2
# value: 524288
# description: 0x80000 2nd 3D magnetometer

# name: MAV_SYS_STATUS_GEOFENCE
# value: 1048576
# description: 0x100000 geofence

# name: MAV_SYS_STATUS_AHRS
# value: 2097152
# description: 0x200000 AHRS subsystem health

# name: MAV_SYS_STATUS_TERRAIN
# value: 4194304
# description: 0x400000 Terrain subsystem health

# name: MAV_SYS_STATUS_REVERSE_MOTOR
# value: 8388608
# description: 0x800000 Motors are reversed

# name: MAV_SYS_STATUS_LOGGING
# value: 16777216
# description: 0x1000000 Logging

# name: MAV_SYS_STATUS_SENSOR_BATTERY
# value: 33554432
# description: 0x2000000 Battery

# name: MAV_SYS_STATUS_SENSOR_PROXIMITY
# value: 67108864
# description: 0x4000000 Proximity

# name: MAV_SYS_STATUS_SENSOR_SATCOM
# value: 134217728
# description: 0x8000000 Satellite Communication 

# name: MAV_SYS_STATUS_PREARM_CHECK
# value: 268435456
# description: 0x10000000 pre-arm check status. Always healthy when armed

# name: MAV_SYS_STATUS_OBSTACLE_AVOIDANCE
# value: 536870912
# description: 0x20000000 Avoidance/collision prevention

# name: MAV_SYS_STATUS_SENSOR_PROPULSION
# value: 1073741824
# description: 0x40000000 propulsion (actuator, esc, motor or propellor)

 
mav_frame = mavutil.mavlink.MAV_FRAME
# name: MAV_FRAME_GLOBAL
# value: 0
# description: Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL).

# name: MAV_FRAME_LOCAL_NED
# value: 1
# description: Local coordinate frame, Z-down (x: North, y: East, z: Down).

# name: MAV_FRAME_MISSION
# value: 2
# description: NOT a coordinate frame, indicates a mission command.

# name: MAV_FRAME_GLOBAL_RELATIVE_ALT
# value: 3
# description: Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.

# name: MAV_FRAME_LOCAL_ENU
# value: 4
# description: Local coordinate frame, Z-up (x: East, y: North, z: Up).

# name: MAV_FRAME_GLOBAL_INT
# value: 5
# description: Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL).

# name: MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
# value: 6
# description: Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location.

# name: MAV_FRAME_LOCAL_OFFSET_NED
# value: 7
# description: Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.

# name: MAV_FRAME_BODY_NED
# value: 8
# description: Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.

# name: MAV_FRAME_BODY_OFFSET_NED
# value: 9
# description: Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.

# name: MAV_FRAME_GLOBAL_TERRAIN_ALT
# value: 10
# description: Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.

# name: MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
# value: 11
# description: Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.

# name: MAV_FRAME_BODY_FRD
# value: 12
# description: Body fixed frame of reference, Z-down (x: Forward, y: Right, z: Down).

# name: MAV_FRAME_RESERVED_13
# value: 13
# description: MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up).

# name: MAV_FRAME_RESERVED_14
# value: 14
# description: MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system, Z-down (x: North, y: East, z: Down).

# name: MAV_FRAME_RESERVED_15
# value: 15
# description: MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up (x: East, y: North, z: Up).

# name: MAV_FRAME_RESERVED_16
# value: 16
# description: MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: North, y: East, z: Down).

# name: MAV_FRAME_RESERVED_17
# value: 17
# description: MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: East, y: North, z: Up).

# name: MAV_FRAME_RESERVED_18
# value: 18
# description: MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: North, y: East, z: Down).

# name: MAV_FRAME_RESERVED_19
# value: 19
# description: MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: East, y: North, z: Up).

# name: MAV_FRAME_LOCAL_FRD
# value: 20
# description: Forward, Right, Down coordinate frame. This is a local frame with Z-down and arbitrary F/R alignment (i.e. not aligned with NED/earth frame).

# name: MAV_FRAME_LOCAL_FLU
# value: 21
# description: Forward, Left, Up coordinate frame. This is a local frame with Z-up and arbitrary F/L alignment (i.e. not aligned with ENU/earth frame).

 
mavlink_data_stream_type = mavutil.mavlink.MAVLINK_DATA_STREAM_TYPE
# name: MAVLINK_DATA_STREAM_IMG_JPEG
# value: 0
# description: 

# name: MAVLINK_DATA_STREAM_IMG_BMP
# value: 1
# description: 

# name: MAVLINK_DATA_STREAM_IMG_RAW8U
# value: 2
# description: 

# name: MAVLINK_DATA_STREAM_IMG_RAW32U
# value: 3
# description: 

# name: MAVLINK_DATA_STREAM_IMG_PGM
# value: 4
# description: 

# name: MAVLINK_DATA_STREAM_IMG_PNG
# value: 5
# description: 

 
fence_action = mavutil.mavlink.FENCE_ACTION
# name: FENCE_ACTION_NONE
# value: 0
# description: Disable fenced mode. If used in a plan this would mean the next fence is disabled.

# name: FENCE_ACTION_GUIDED
# value: 1
# description: Fly to geofence MAV_CMD_NAV_FENCE_RETURN_POINT in GUIDED mode. Note: This action is only supported by ArduPlane, and may not be supported in all versions.

# name: FENCE_ACTION_REPORT
# value: 2
# description: Report fence breach, but don't take action

# name: FENCE_ACTION_GUIDED_THR_PASS
# value: 3
# description: Fly to geofence MAV_CMD_NAV_FENCE_RETURN_POINT with manual throttle control in GUIDED mode. Note: This action is only supported by ArduPlane, and may not be supported in all versions.

# name: FENCE_ACTION_RTL
# value: 4
# description: Return/RTL mode.

# name: FENCE_ACTION_HOLD
# value: 5
# description: Hold at current location.

# name: FENCE_ACTION_TERMINATE
# value: 6
# description: Termination failsafe. Motors are shut down (some flight stacks may trigger other failsafe actions).

# name: FENCE_ACTION_LAND
# value: 7
# description: Land at current location.

 
fence_breach = mavutil.mavlink.FENCE_BREACH
# name: FENCE_BREACH_NONE
# value: 0
# description: No last fence breach

# name: FENCE_BREACH_MINALT
# value: 1
# description: Breached minimum altitude

# name: FENCE_BREACH_MAXALT
# value: 2
# description: Breached maximum altitude

# name: FENCE_BREACH_BOUNDARY
# value: 3
# description: Breached fence boundary

 
fence_mitigate = mavutil.mavlink.FENCE_MITIGATE
# name: FENCE_MITIGATE_UNKNOWN
# value: 0
# description: Unknown

# name: FENCE_MITIGATE_NONE
# value: 1
# description: No actions being taken

# name: FENCE_MITIGATE_VEL_LIMIT
# value: 2
# description: Velocity limiting active to prevent breach

 
mav_mount_mode = mavutil.mavlink.MAV_MOUNT_MODE
# name: MAV_MOUNT_MODE_RETRACT
# value: 0
# description: Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization

# name: MAV_MOUNT_MODE_NEUTRAL
# value: 1
# description: Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.

# name: MAV_MOUNT_MODE_MAVLINK_TARGETING
# value: 2
# description: Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization

# name: MAV_MOUNT_MODE_RC_TARGETING
# value: 3
# description: Load neutral position and start RC Roll,Pitch,Yaw control with stabilization

# name: MAV_MOUNT_MODE_GPS_POINT
# value: 4
# description: Load neutral position and start to point to Lat,Lon,Alt

# name: MAV_MOUNT_MODE_SYSID_TARGET
# value: 5
# description: Gimbal tracks system with specified system ID

# name: MAV_MOUNT_MODE_HOME_LOCATION
# value: 6
# description: Gimbal tracks home location

 
gimbal_device_cap_flags = mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS
# name: GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT
# value: 1
# description: Gimbal device supports a retracted position

# name: GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL
# value: 2
# description: Gimbal device supports a horizontal, forward looking position, stabilized

# name: GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS
# value: 4
# description: Gimbal device supports rotating around roll axis.

# name: GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW
# value: 8
# description: Gimbal device supports to follow a roll angle relative to the vehicle

# name: GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK
# value: 16
# description: Gimbal device supports locking to an roll angle (generally that's the default with roll stabilized)

# name: GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS
# value: 32
# description: Gimbal device supports rotating around pitch axis.

# name: GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW
# value: 64
# description: Gimbal device supports to follow a pitch angle relative to the vehicle

# name: GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK
# value: 128
# description: Gimbal device supports locking to an pitch angle (generally that's the default with pitch stabilized)

# name: GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS
# value: 256
# description: Gimbal device supports rotating around yaw axis.

# name: GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW
# value: 512
# description: Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default)

# name: GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK
# value: 1024
# description: Gimbal device supports locking to an absolute heading (often this is an option available)

# name: GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW
# value: 2048
# description: Gimbal device supports yawing/panning infinetely (e.g. using slip disk).

 
gimbal_manager_cap_flags = mavutil.mavlink.GIMBAL_MANAGER_CAP_FLAGS
# name: GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT
# value: 1
# description: Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT.

# name: GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL
# value: 2
# description: Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL.

# name: GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS
# value: 4
# description: Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS.

# name: GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW
# value: 8
# description: Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW.

# name: GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK
# value: 16
# description: Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK.

# name: GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS
# value: 32
# description: Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS.

# name: GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW
# value: 64
# description: Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW.

# name: GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK
# value: 128
# description: Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK.

# name: GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS
# value: 256
# description: Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS.

# name: GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW
# value: 512
# description: Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW.

# name: GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK
# value: 1024
# description: Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK.

# name: GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW
# value: 2048
# description: Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW.

# name: GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL
# value: 65536
# description: Gimbal manager supports to point to a local position.

# name: GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL
# value: 131072
# description: Gimbal manager supports to point to a global latitude, longitude, altitude position.

 
gimbal_device_flags = mavutil.mavlink.GIMBAL_DEVICE_FLAGS
# name: GIMBAL_DEVICE_FLAGS_RETRACT
# value: 1
# description: Set to retracted safe position (no stabilization), takes presedence over all other flags.

# name: GIMBAL_DEVICE_FLAGS_NEUTRAL
# value: 2
# description: Set to neutral position (horizontal, forward looking, with stabiliziation), takes presedence over all other flags except RETRACT.

# name: GIMBAL_DEVICE_FLAGS_ROLL_LOCK
# value: 4
# description: Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal.

# name: GIMBAL_DEVICE_FLAGS_PITCH_LOCK
# value: 8
# description: Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default.

# name: GIMBAL_DEVICE_FLAGS_YAW_LOCK
# value: 16
# description: Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle).

 
gimbal_manager_flags = mavutil.mavlink.GIMBAL_MANAGER_FLAGS
# name: GIMBAL_MANAGER_FLAGS_RETRACT
# value: 1
# description: Based on GIMBAL_DEVICE_FLAGS_RETRACT

# name: GIMBAL_MANAGER_FLAGS_NEUTRAL
# value: 2
# description: Based on GIMBAL_DEVICE_FLAGS_NEUTRAL

# name: GIMBAL_MANAGER_FLAGS_ROLL_LOCK
# value: 4
# description: Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK

# name: GIMBAL_MANAGER_FLAGS_PITCH_LOCK
# value: 8
# description: Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK

# name: GIMBAL_MANAGER_FLAGS_YAW_LOCK
# value: 16
# description: Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK

 
gimbal_device_error_flags = mavutil.mavlink.GIMBAL_DEVICE_ERROR_FLAGS
# name: GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT
# value: 1
# description: Gimbal device is limited by hardware roll limit.

# name: GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT
# value: 2
# description: Gimbal device is limited by hardware pitch limit.

# name: GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT
# value: 4
# description: Gimbal device is limited by hardware yaw limit.

# name: GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR
# value: 8
# description: There is an error with the gimbal encoders.

# name: GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR
# value: 16
# description: There is an error with the gimbal power source.

# name: GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR
# value: 32
# description: There is an error with the gimbal motor's.

# name: GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR
# value: 64
# description: There is an error with the gimbal's software.

# name: GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR
# value: 128
# description: There is an error with the gimbal's communication.

# name: GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING
# value: 256
# description: Gimbal is currently calibrating.

 
gripper_actions = mavutil.mavlink.GRIPPER_ACTIONS
# name: GRIPPER_ACTION_RELEASE
# value: 0
# description: Gripper release cargo.

# name: GRIPPER_ACTION_GRAB
# value: 1
# description: Gripper grab onto cargo.

 
winch_actions = mavutil.mavlink.WINCH_ACTIONS
# name: WINCH_RELAXED
# value: 0
# description: Relax winch.

# name: WINCH_RELATIVE_LENGTH_CONTROL
# value: 1
# description: Wind or unwind specified length of cable, optionally using specified rate.

# name: WINCH_RATE_CONTROL
# value: 2
# description: Wind or unwind cable at specified rate.

 
uavcan_node_health = mavutil.mavlink.UAVCAN_NODE_HEALTH
# name: UAVCAN_NODE_HEALTH_OK
# value: 0
# description: The node is functioning properly.

# name: UAVCAN_NODE_HEALTH_WARNING
# value: 1
# description: A critical parameter went out of range or the node has encountered a minor failure.

# name: UAVCAN_NODE_HEALTH_ERROR
# value: 2
# description: The node has encountered a major failure.

# name: UAVCAN_NODE_HEALTH_CRITICAL
# value: 3
# description: The node has suffered a fatal malfunction.

 
uavcan_node_mode = mavutil.mavlink.UAVCAN_NODE_MODE
# name: UAVCAN_NODE_MODE_OPERATIONAL
# value: 0
# description: The node is performing its primary functions.

# name: UAVCAN_NODE_MODE_INITIALIZATION
# value: 1
# description: The node is initializing; this mode is entered immediately after startup.

# name: UAVCAN_NODE_MODE_MAINTENANCE
# value: 2
# description: The node is under maintenance.

# name: UAVCAN_NODE_MODE_SOFTWARE_UPDATE
# value: 3
# description: The node is in the process of updating its software.

# name: UAVCAN_NODE_MODE_OFFLINE
# value: 7
# description: The node is no longer available online.

 
esc_connection_type = mavutil.mavlink.ESC_CONNECTION_TYPE
# name: ESC_CONNECTION_TYPE_PPM
# value: 0
# description: Traditional PPM ESC.

# name: ESC_CONNECTION_TYPE_SERIAL
# value: 1
# description: Serial Bus connected ESC.

# name: ESC_CONNECTION_TYPE_ONESHOT
# value: 2
# description: One Shot PPM ESC.

# name: ESC_CONNECTION_TYPE_I2C
# value: 3
# description: I2C ESC.

# name: ESC_CONNECTION_TYPE_CAN
# value: 4
# description: CAN-Bus ESC.

# name: ESC_CONNECTION_TYPE_DSHOT
# value: 5
# description: DShot ESC.

 
esc_failure_flags = mavutil.mavlink.ESC_FAILURE_FLAGS
# name: ESC_FAILURE_NONE
# value: 0
# description: No ESC failure.

# name: ESC_FAILURE_OVER_CURRENT
# value: 1
# description: Over current failure.

# name: ESC_FAILURE_OVER_VOLTAGE
# value: 2
# description: Over voltage failure.

# name: ESC_FAILURE_OVER_TEMPERATURE
# value: 4
# description: Over temperature failure.

# name: ESC_FAILURE_OVER_RPM
# value: 8
# description: Over RPM failure.

# name: ESC_FAILURE_INCONSISTENT_CMD
# value: 16
# description: Inconsistent command failure i.e. out of bounds.

# name: ESC_FAILURE_MOTOR_STUCK
# value: 32
# description: Motor stuck failure.

# name: ESC_FAILURE_GENERIC
# value: 64
# description: Generic ESC failure.

 
storage_status = mavutil.mavlink.STORAGE_STATUS
# name: STORAGE_STATUS_EMPTY
# value: 0
# description: Storage is missing (no microSD card loaded for example.)

# name: STORAGE_STATUS_UNFORMATTED
# value: 1
# description: Storage present but unformatted.

# name: STORAGE_STATUS_READY
# value: 2
# description: Storage present and ready.

# name: STORAGE_STATUS_NOT_SUPPORTED
# value: 3
# description: Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION fields will be ignored.

 
storage_type = mavutil.mavlink.STORAGE_TYPE
# name: STORAGE_TYPE_UNKNOWN
# value: 0
# description: Storage type is not known.

# name: STORAGE_TYPE_USB_STICK
# value: 1
# description: Storage type is USB device.

# name: STORAGE_TYPE_SD
# value: 2
# description: Storage type is SD card.

# name: STORAGE_TYPE_MICROSD
# value: 3
# description: Storage type is microSD card.

# name: STORAGE_TYPE_CF
# value: 4
# description: Storage type is CFast.

# name: STORAGE_TYPE_CFE
# value: 5
# description: Storage type is CFexpress.

# name: STORAGE_TYPE_XQD
# value: 6
# description: Storage type is XQD.

# name: STORAGE_TYPE_HD
# value: 7
# description: Storage type is HD mass storage type.

# name: STORAGE_TYPE_OTHER
# value: 254
# description: Storage type is other, not listed type.

 
orbit_yaw_behaviour = mavutil.mavlink.ORBIT_YAW_BEHAVIOUR
# name: ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER
# value: 0
# description: Vehicle front points to the center (default).

# name: ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING
# value: 1
# description: Vehicle front holds heading when message received.

# name: ORBIT_YAW_BEHAVIOUR_UNCONTROLLED
# value: 2
# description: Yaw uncontrolled.

# name: ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE
# value: 3
# description: Vehicle front follows flight path (tangential to circle).

# name: ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED
# value: 4
# description: Yaw controlled by RC input.

 
wifi_config_ap_response = mavutil.mavlink.WIFI_CONFIG_AP_RESPONSE
# name: WIFI_CONFIG_AP_RESPONSE_UNDEFINED
# value: 0
# description: Undefined response. Likely an indicative of a system that doesn't support this request.

# name: WIFI_CONFIG_AP_RESPONSE_ACCEPTED
# value: 1
# description: Changes accepted.

# name: WIFI_CONFIG_AP_RESPONSE_REJECTED
# value: 2
# description: Changes rejected.

# name: WIFI_CONFIG_AP_RESPONSE_MODE_ERROR
# value: 3
# description: Invalid Mode.

# name: WIFI_CONFIG_AP_RESPONSE_SSID_ERROR
# value: 4
# description: Invalid SSID.

# name: WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR
# value: 5
# description: Invalid Password.

 
cellular_config_response = mavutil.mavlink.CELLULAR_CONFIG_RESPONSE
# name: CELLULAR_CONFIG_RESPONSE_ACCEPTED
# value: 0
# description: Changes accepted.

# name: CELLULAR_CONFIG_RESPONSE_APN_ERROR
# value: 1
# description: Invalid APN.

# name: CELLULAR_CONFIG_RESPONSE_PIN_ERROR
# value: 2
# description: Invalid PIN.

# name: CELLULAR_CONFIG_RESPONSE_REJECTED
# value: 3
# description: Changes rejected.

# name: CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED
# value: 4
# description: PUK is required to unblock SIM card.

 
wifi_config_ap_mode = mavutil.mavlink.WIFI_CONFIG_AP_MODE
# name: WIFI_CONFIG_AP_MODE_UNDEFINED
# value: 0
# description: WiFi mode is undefined.

# name: WIFI_CONFIG_AP_MODE_AP
# value: 1
# description: WiFi configured as an access point.

# name: WIFI_CONFIG_AP_MODE_STATION
# value: 2
# description: WiFi configured as a station connected to an existing local WiFi network.

# name: WIFI_CONFIG_AP_MODE_DISABLED
# value: 3
# description: WiFi disabled.

 
comp_metadata_type = mavutil.mavlink.COMP_METADATA_TYPE
# name: COMP_METADATA_TYPE_GENERAL
# value: 0
# description: General information about the component. General metadata includes information about other COMP_METADATA_TYPEs supported by the component. This type must be supported and must be downloadable from vehicle.

# name: COMP_METADATA_TYPE_PARAMETER
# value: 1
# description: Parameter meta data.

# name: COMP_METADATA_TYPE_COMMANDS
# value: 2
# description: Meta data that specifies which commands and command parameters the vehicle supports. (WIP)

# name: COMP_METADATA_TYPE_PERIPHERALS
# value: 3
# description: Meta data that specifies external non-MAVLink peripherals.

# name: COMP_METADATA_TYPE_EVENTS
# value: 4
# description: Meta data for the events interface.

 
param_transaction_transport = mavutil.mavlink.PARAM_TRANSACTION_TRANSPORT
# name: PARAM_TRANSACTION_TRANSPORT_PARAM
# value: 0
# description: Transaction over param transport.

# name: PARAM_TRANSACTION_TRANSPORT_PARAM_EXT
# value: 1
# description: Transaction over param_ext transport.

 
param_transaction_action = mavutil.mavlink.PARAM_TRANSACTION_ACTION
# name: PARAM_TRANSACTION_ACTION_START
# value: 0
# description: Commit the current parameter transaction.

# name: PARAM_TRANSACTION_ACTION_COMMIT
# value: 1
# description: Commit the current parameter transaction.

# name: PARAM_TRANSACTION_ACTION_CANCEL
# value: 2
# description: Cancel the current parameter transaction.

 
param_transaction_action = mavutil.mavlink.PARAM_TRANSACTION_ACTION
# name: PARAM_TRANSACTION_ACTION_START
# value: 0
# description: Commit the current parameter transaction.

# name: PARAM_TRANSACTION_ACTION_COMMIT
# value: 1
# description: Commit the current parameter transaction.

# name: PARAM_TRANSACTION_ACTION_CANCEL
# value: 2
# description: Cancel the current parameter transaction.

 
mav_data_stream = mavutil.mavlink.MAV_DATA_STREAM
# name: MAV_DATA_STREAM_ALL
# value: 0
# description: Enable all data streams

# name: MAV_DATA_STREAM_RAW_SENSORS
# value: 1
# description: Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.

# name: MAV_DATA_STREAM_EXTENDED_STATUS
# value: 2
# description: Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS

# name: MAV_DATA_STREAM_RC_CHANNELS
# value: 3
# description: Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW

# name: MAV_DATA_STREAM_RAW_CONTROLLER
# value: 4
# description: Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.

# name: MAV_DATA_STREAM_POSITION
# value: 6
# description: Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.

# name: MAV_DATA_STREAM_EXTRA1
# value: 10
# description: Dependent on the autopilot

# name: MAV_DATA_STREAM_EXTRA2
# value: 11
# description: Dependent on the autopilot

# name: MAV_DATA_STREAM_EXTRA3
# value: 12
# description: Dependent on the autopilot

 
mav_roi = mavutil.mavlink.MAV_ROI
# name: MAV_ROI_NONE
# value: 0
# description: No region of interest.

# name: MAV_ROI_WPNEXT
# value: 1
# description: Point toward next waypoint, with optional pitch/roll/yaw offset.

# name: MAV_ROI_WPINDEX
# value: 2
# description: Point toward given waypoint.

# name: MAV_ROI_LOCATION
# value: 3
# description: Point toward fixed location.

# name: MAV_ROI_TARGET
# value: 4
# description: Point toward of given id.

 
mav_cmd_ack = mavutil.mavlink.MAV_CMD_ACK
# name: MAV_CMD_ACK_OK
# value: 0
# description: Command / mission item is ok.

# name: MAV_CMD_ACK_ERR_FAIL
# value: 1
# description: Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.

# name: MAV_CMD_ACK_ERR_ACCESS_DENIED
# value: 2
# description: The system is refusing to accept this command from this source / communication partner.

# name: MAV_CMD_ACK_ERR_NOT_SUPPORTED
# value: 3
# description: Command or mission item is not supported, other commands would be accepted.

# name: MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED
# value: 4
# description: The coordinate frame of this command / mission item is not supported.

# name: MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE
# value: 5
# description: The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible.

# name: MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE
# value: 6
# description: The X or latitude value is out of range.

# name: MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE
# value: 7
# description: The Y or longitude value is out of range.

# name: MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE
# value: 8
# description: The Z or altitude value is out of range.

 
mav_param_type = mavutil.mavlink.MAV_PARAM_TYPE
# name: MAV_PARAM_TYPE_UINT8
# value: 1
# description: 8-bit unsigned integer

# name: MAV_PARAM_TYPE_INT8
# value: 2
# description: 8-bit signed integer

# name: MAV_PARAM_TYPE_UINT16
# value: 3
# description: 16-bit unsigned integer

# name: MAV_PARAM_TYPE_INT16
# value: 4
# description: 16-bit signed integer

# name: MAV_PARAM_TYPE_UINT32
# value: 5
# description: 32-bit unsigned integer

# name: MAV_PARAM_TYPE_INT32
# value: 6
# description: 32-bit signed integer

# name: MAV_PARAM_TYPE_UINT64
# value: 7
# description: 64-bit unsigned integer

# name: MAV_PARAM_TYPE_INT64
# value: 8
# description: 64-bit signed integer

# name: MAV_PARAM_TYPE_REAL32
# value: 9
# description: 32-bit floating-point

# name: MAV_PARAM_TYPE_REAL64
# value: 10
# description: 64-bit floating-point

 
mav_param_ext_type = mavutil.mavlink.MAV_PARAM_EXT_TYPE
# name: MAV_PARAM_EXT_TYPE_UINT8
# value: 1
# description: 8-bit unsigned integer

# name: MAV_PARAM_EXT_TYPE_INT8
# value: 2
# description: 8-bit signed integer

# name: MAV_PARAM_EXT_TYPE_UINT16
# value: 3
# description: 16-bit unsigned integer

# name: MAV_PARAM_EXT_TYPE_INT16
# value: 4
# description: 16-bit signed integer

# name: MAV_PARAM_EXT_TYPE_UINT32
# value: 5
# description: 32-bit unsigned integer

# name: MAV_PARAM_EXT_TYPE_INT32
# value: 6
# description: 32-bit signed integer

# name: MAV_PARAM_EXT_TYPE_UINT64
# value: 7
# description: 64-bit unsigned integer

# name: MAV_PARAM_EXT_TYPE_INT64
# value: 8
# description: 64-bit signed integer

# name: MAV_PARAM_EXT_TYPE_REAL32
# value: 9
# description: 32-bit floating-point

# name: MAV_PARAM_EXT_TYPE_REAL64
# value: 10
# description: 64-bit floating-point

# name: MAV_PARAM_EXT_TYPE_CUSTOM
# value: 11
# description: Custom Type

 
mav_result = mavutil.mavlink.MAV_RESULT
# name: MAV_RESULT_ACCEPTED
# value: 0
# description: Command is valid (is supported and has valid parameters), and was executed.

# name: MAV_RESULT_TEMPORARILY_REJECTED
# value: 1
# description: Command is valid, but cannot be executed at this time. This is used to indicate a problem that should be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS lock, etc.). Retrying later should work.

# name: MAV_RESULT_DENIED
# value: 2
# description: Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will not work.

# name: MAV_RESULT_UNSUPPORTED
# value: 3
# description: Command is not supported (unknown).

# name: MAV_RESULT_FAILED
# value: 4
# description: Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting to write a file when out of memory, attempting to arm when sensors are not calibrated, etc.

# name: MAV_RESULT_IN_PROGRESS
# value: 5
# description: Command is valid and is being executed. This will be followed by further progress updates, i.e. the component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the operation.

# name: MAV_RESULT_CANCELLED
# value: 6
# description: Command has been cancelled (as a result of receiving a COMMAND_CANCEL message).

 
mav_mission_result = mavutil.mavlink.MAV_MISSION_RESULT
# name: MAV_MISSION_ACCEPTED
# value: 0
# description: mission accepted OK

# name: MAV_MISSION_ERROR
# value: 1
# description: Generic error / not accepting mission commands at all right now.

# name: MAV_MISSION_UNSUPPORTED_FRAME
# value: 2
# description: Coordinate frame is not supported.

# name: MAV_MISSION_UNSUPPORTED
# value: 3
# description: Command is not supported.

# name: MAV_MISSION_NO_SPACE
# value: 4
# description: Mission items exceed storage space.

# name: MAV_MISSION_INVALID
# value: 5
# description: One of the parameters has an invalid value.

# name: MAV_MISSION_INVALID_PARAM1
# value: 6
# description: param1 has an invalid value.

# name: MAV_MISSION_INVALID_PARAM2
# value: 7
# description: param2 has an invalid value.

# name: MAV_MISSION_INVALID_PARAM3
# value: 8
# description: param3 has an invalid value.

# name: MAV_MISSION_INVALID_PARAM4
# value: 9
# description: param4 has an invalid value.

# name: MAV_MISSION_INVALID_PARAM5_X
# value: 10
# description: x / param5 has an invalid value.

# name: MAV_MISSION_INVALID_PARAM6_Y
# value: 11
# description: y / param6 has an invalid value.

# name: MAV_MISSION_INVALID_PARAM7
# value: 12
# description: z / param7 has an invalid value.

# name: MAV_MISSION_INVALID_SEQUENCE
# value: 13
# description: Mission item received out of sequence

# name: MAV_MISSION_DENIED
# value: 14
# description: Not accepting any mission commands from this communication partner.

# name: MAV_MISSION_OPERATION_CANCELLED
# value: 15
# description: Current mission operation cancelled (e.g. mission upload, mission download).

 
mav_severity = mavutil.mavlink.MAV_SEVERITY
# name: MAV_SEVERITY_EMERGENCY
# value: 0
# description: System is unusable. This is a "panic" condition.

# name: MAV_SEVERITY_ALERT
# value: 1
# description: Action should be taken immediately. Indicates error in non-critical systems.

# name: MAV_SEVERITY_CRITICAL
# value: 2
# description: Action must be taken immediately. Indicates failure in a primary system.

# name: MAV_SEVERITY_ERROR
# value: 3
# description: Indicates an error in secondary/redundant systems.

# name: MAV_SEVERITY_WARNING
# value: 4
# description: Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.

# name: MAV_SEVERITY_NOTICE
# value: 5
# description: An unusual event has occurred, though not an error condition. This should be investigated for the root cause.

# name: MAV_SEVERITY_INFO
# value: 6
# description: Normal operational messages. Useful for logging. No action is required for these messages.

# name: MAV_SEVERITY_DEBUG
# value: 7
# description: Useful non-operational messages that can assist in debugging. These should not occur during normal operation.

 
mav_power_status = mavutil.mavlink.MAV_POWER_STATUS
# name: MAV_POWER_STATUS_BRICK_VALID
# value: 1
# description: main brick power supply valid

# name: MAV_POWER_STATUS_SERVO_VALID
# value: 2
# description: main servo power supply valid for FMU

# name: MAV_POWER_STATUS_USB_CONNECTED
# value: 4
# description: USB power is connected

# name: MAV_POWER_STATUS_PERIPH_OVERCURRENT
# value: 8
# description: peripheral supply is in over-current state

# name: MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT
# value: 16
# description: hi-power peripheral supply is in over-current state

# name: MAV_POWER_STATUS_CHANGED
# value: 32
# description: Power status has changed since boot

 
serial_control_dev = mavutil.mavlink.SERIAL_CONTROL_DEV
# name: SERIAL_CONTROL_DEV_TELEM1
# value: 0
# description: First telemetry port

# name: SERIAL_CONTROL_DEV_TELEM2
# value: 1
# description: Second telemetry port

# name: SERIAL_CONTROL_DEV_GPS1
# value: 2
# description: First GPS port

# name: SERIAL_CONTROL_DEV_GPS2
# value: 3
# description: Second GPS port

# name: SERIAL_CONTROL_DEV_SHELL
# value: 10
# description: system shell

# name: SERIAL_CONTROL_SERIAL0
# value: 100
# description: SERIAL0

# name: SERIAL_CONTROL_SERIAL1
# value: 101
# description: SERIAL1

# name: SERIAL_CONTROL_SERIAL2
# value: 102
# description: SERIAL2

# name: SERIAL_CONTROL_SERIAL3
# value: 103
# description: SERIAL3

# name: SERIAL_CONTROL_SERIAL4
# value: 104
# description: SERIAL4

# name: SERIAL_CONTROL_SERIAL5
# value: 105
# description: SERIAL5

# name: SERIAL_CONTROL_SERIAL6
# value: 106
# description: SERIAL6

# name: SERIAL_CONTROL_SERIAL7
# value: 107
# description: SERIAL7

# name: SERIAL_CONTROL_SERIAL8
# value: 108
# description: SERIAL8

# name: SERIAL_CONTROL_SERIAL9
# value: 109
# description: SERIAL9

 
serial_control_flag = mavutil.mavlink.SERIAL_CONTROL_FLAG
# name: SERIAL_CONTROL_FLAG_REPLY
# value: 1
# description: Set if this is a reply

# name: SERIAL_CONTROL_FLAG_RESPOND
# value: 2
# description: Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message

# name: SERIAL_CONTROL_FLAG_EXCLUSIVE
# value: 4
# description: Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set

# name: SERIAL_CONTROL_FLAG_BLOCKING
# value: 8
# description: Block on writes to the serial port

# name: SERIAL_CONTROL_FLAG_MULTI
# value: 16
# description: Send multiple replies until port is drained

 
mav_distance_sensor = mavutil.mavlink.MAV_DISTANCE_SENSOR
# name: MAV_DISTANCE_SENSOR_LASER
# value: 0
# description: Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units

# name: MAV_DISTANCE_SENSOR_ULTRASOUND
# value: 1
# description: Ultrasound rangefinder, e.g. MaxBotix units

# name: MAV_DISTANCE_SENSOR_INFRARED
# value: 2
# description: Infrared rangefinder, e.g. Sharp units

# name: MAV_DISTANCE_SENSOR_RADAR
# value: 3
# description: Radar type, e.g. uLanding units

# name: MAV_DISTANCE_SENSOR_UNKNOWN
# value: 4
# description: Broken or unknown type, e.g. analog units

 
mav_sensor_orientation = mavutil.mavlink.MAV_SENSOR_ORIENTATION
# name: MAV_SENSOR_ROTATION_NONE
# value: 0
# description: Roll: 0, Pitch: 0, Yaw: 0

# name: MAV_SENSOR_ROTATION_YAW_45
# value: 1
# description: Roll: 0, Pitch: 0, Yaw: 45

# name: MAV_SENSOR_ROTATION_YAW_90
# value: 2
# description: Roll: 0, Pitch: 0, Yaw: 90

# name: MAV_SENSOR_ROTATION_YAW_135
# value: 3
# description: Roll: 0, Pitch: 0, Yaw: 135

# name: MAV_SENSOR_ROTATION_YAW_180
# value: 4
# description: Roll: 0, Pitch: 0, Yaw: 180

# name: MAV_SENSOR_ROTATION_YAW_225
# value: 5
# description: Roll: 0, Pitch: 0, Yaw: 225

# name: MAV_SENSOR_ROTATION_YAW_270
# value: 6
# description: Roll: 0, Pitch: 0, Yaw: 270

# name: MAV_SENSOR_ROTATION_YAW_315
# value: 7
# description: Roll: 0, Pitch: 0, Yaw: 315

# name: MAV_SENSOR_ROTATION_ROLL_180
# value: 8
# description: Roll: 180, Pitch: 0, Yaw: 0

# name: MAV_SENSOR_ROTATION_ROLL_180_YAW_45
# value: 9
# description: Roll: 180, Pitch: 0, Yaw: 45

# name: MAV_SENSOR_ROTATION_ROLL_180_YAW_90
# value: 10
# description: Roll: 180, Pitch: 0, Yaw: 90

# name: MAV_SENSOR_ROTATION_ROLL_180_YAW_135
# value: 11
# description: Roll: 180, Pitch: 0, Yaw: 135

# name: MAV_SENSOR_ROTATION_PITCH_180
# value: 12
# description: Roll: 0, Pitch: 180, Yaw: 0

# name: MAV_SENSOR_ROTATION_ROLL_180_YAW_225
# value: 13
# description: Roll: 180, Pitch: 0, Yaw: 225

# name: MAV_SENSOR_ROTATION_ROLL_180_YAW_270
# value: 14
# description: Roll: 180, Pitch: 0, Yaw: 270

# name: MAV_SENSOR_ROTATION_ROLL_180_YAW_315
# value: 15
# description: Roll: 180, Pitch: 0, Yaw: 315

# name: MAV_SENSOR_ROTATION_ROLL_90
# value: 16
# description: Roll: 90, Pitch: 0, Yaw: 0

# name: MAV_SENSOR_ROTATION_ROLL_90_YAW_45
# value: 17
# description: Roll: 90, Pitch: 0, Yaw: 45

# name: MAV_SENSOR_ROTATION_ROLL_90_YAW_90
# value: 18
# description: Roll: 90, Pitch: 0, Yaw: 90

# name: MAV_SENSOR_ROTATION_ROLL_90_YAW_135
# value: 19
# description: Roll: 90, Pitch: 0, Yaw: 135

# name: MAV_SENSOR_ROTATION_ROLL_270
# value: 20
# description: Roll: 270, Pitch: 0, Yaw: 0

# name: MAV_SENSOR_ROTATION_ROLL_270_YAW_45
# value: 21
# description: Roll: 270, Pitch: 0, Yaw: 45

# name: MAV_SENSOR_ROTATION_ROLL_270_YAW_90
# value: 22
# description: Roll: 270, Pitch: 0, Yaw: 90

# name: MAV_SENSOR_ROTATION_ROLL_270_YAW_135
# value: 23
# description: Roll: 270, Pitch: 0, Yaw: 135

# name: MAV_SENSOR_ROTATION_PITCH_90
# value: 24
# description: Roll: 0, Pitch: 90, Yaw: 0

# name: MAV_SENSOR_ROTATION_PITCH_270
# value: 25
# description: Roll: 0, Pitch: 270, Yaw: 0

# name: MAV_SENSOR_ROTATION_PITCH_180_YAW_90
# value: 26
# description: Roll: 0, Pitch: 180, Yaw: 90

# name: MAV_SENSOR_ROTATION_PITCH_180_YAW_270
# value: 27
# description: Roll: 0, Pitch: 180, Yaw: 270

# name: MAV_SENSOR_ROTATION_ROLL_90_PITCH_90
# value: 28
# description: Roll: 90, Pitch: 90, Yaw: 0

# name: MAV_SENSOR_ROTATION_ROLL_180_PITCH_90
# value: 29
# description: Roll: 180, Pitch: 90, Yaw: 0

# name: MAV_SENSOR_ROTATION_ROLL_270_PITCH_90
# value: 30
# description: Roll: 270, Pitch: 90, Yaw: 0

# name: MAV_SENSOR_ROTATION_ROLL_90_PITCH_180
# value: 31
# description: Roll: 90, Pitch: 180, Yaw: 0

# name: MAV_SENSOR_ROTATION_ROLL_270_PITCH_180
# value: 32
# description: Roll: 270, Pitch: 180, Yaw: 0

# name: MAV_SENSOR_ROTATION_ROLL_90_PITCH_270
# value: 33
# description: Roll: 90, Pitch: 270, Yaw: 0

# name: MAV_SENSOR_ROTATION_ROLL_180_PITCH_270
# value: 34
# description: Roll: 180, Pitch: 270, Yaw: 0

# name: MAV_SENSOR_ROTATION_ROLL_270_PITCH_270
# value: 35
# description: Roll: 270, Pitch: 270, Yaw: 0

# name: MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90
# value: 36
# description: Roll: 90, Pitch: 180, Yaw: 90

# name: MAV_SENSOR_ROTATION_ROLL_90_YAW_270
# value: 37
# description: Roll: 90, Pitch: 0, Yaw: 270

# name: MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293
# value: 38
# description: Roll: 90, Pitch: 68, Yaw: 293

# name: MAV_SENSOR_ROTATION_PITCH_315
# value: 39
# description: Pitch: 315

# name: MAV_SENSOR_ROTATION_ROLL_90_PITCH_315
# value: 40
# description: Roll: 90, Pitch: 315

# name: MAV_SENSOR_ROTATION_CUSTOM
# value: 100
# description: Custom orientation

 
mav_protocol_capability = mavutil.mavlink.MAV_PROTOCOL_CAPABILITY
# name: MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT
# value: 1
# description: Autopilot supports MISSION float message type.

# name: MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT
# value: 2
# description: Autopilot supports the new param float message type.

# name: MAV_PROTOCOL_CAPABILITY_MISSION_INT
# value: 4
# description: Autopilot supports MISSION_ITEM_INT scaled integer message type.

# name: MAV_PROTOCOL_CAPABILITY_COMMAND_INT
# value: 8
# description: Autopilot supports COMMAND_INT scaled integer message type.

# name: MAV_PROTOCOL_CAPABILITY_PARAM_UNION
# value: 16
# description: Autopilot supports the new param union message type.

# name: MAV_PROTOCOL_CAPABILITY_FTP
# value: 32
# description: Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.

# name: MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
# value: 64
# description: Autopilot supports commanding attitude offboard.

# name: MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED
# value: 128
# description: Autopilot supports commanding position and velocity targets in local NED frame.

# name: MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT
# value: 256
# description: Autopilot supports commanding position and velocity targets in global scaled integers.

# name: MAV_PROTOCOL_CAPABILITY_TERRAIN
# value: 512
# description: Autopilot supports terrain protocol / data handling.

# name: MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET
# value: 1024
# description: Autopilot supports direct actuator control.

# name: MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION
# value: 2048
# description: Autopilot supports the flight termination command.

# name: MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION
# value: 4096
# description: Autopilot supports onboard compass calibration.

# name: MAV_PROTOCOL_CAPABILITY_MAVLINK2
# value: 8192
# description: Autopilot supports MAVLink version 2.

# name: MAV_PROTOCOL_CAPABILITY_MISSION_FENCE
# value: 16384
# description: Autopilot supports mission fence protocol.

# name: MAV_PROTOCOL_CAPABILITY_MISSION_RALLY
# value: 32768
# description: Autopilot supports mission rally point protocol.

# name: MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION
# value: 65536
# description: Autopilot supports the flight information protocol.

 
mav_mission_type = mavutil.mavlink.MAV_MISSION_TYPE
# name: MAV_MISSION_TYPE_MISSION
# value: 0
# description: Items are mission commands for main mission.

# name: MAV_MISSION_TYPE_FENCE
# value: 1
# description: Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items.

# name: MAV_MISSION_TYPE_RALLY
# value: 2
# description: Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT rally point items.

# name: MAV_MISSION_TYPE_ALL
# value: 255
# description: Only used in MISSION_CLEAR_ALL to clear all mission types.

 
mav_estimator_type = mavutil.mavlink.MAV_ESTIMATOR_TYPE
# name: MAV_ESTIMATOR_TYPE_UNKNOWN
# value: 0
# description: Unknown type of the estimator.

# name: MAV_ESTIMATOR_TYPE_NAIVE
# value: 1
# description: This is a naive estimator without any real covariance feedback.

# name: MAV_ESTIMATOR_TYPE_VISION
# value: 2
# description: Computer vision based estimate. Might be up to scale.

# name: MAV_ESTIMATOR_TYPE_VIO
# value: 3
# description: Visual-inertial estimate.

# name: MAV_ESTIMATOR_TYPE_GPS
# value: 4
# description: Plain GPS estimate.

# name: MAV_ESTIMATOR_TYPE_GPS_INS
# value: 5
# description: Estimator integrating GPS and inertial sensing.

# name: MAV_ESTIMATOR_TYPE_MOCAP
# value: 6
# description: Estimate from external motion capturing system.

# name: MAV_ESTIMATOR_TYPE_LIDAR
# value: 7
# description: Estimator based on lidar sensor input.

# name: MAV_ESTIMATOR_TYPE_AUTOPILOT
# value: 8
# description: Estimator on autopilot.

 
mav_battery_type = mavutil.mavlink.MAV_BATTERY_TYPE
# name: MAV_BATTERY_TYPE_UNKNOWN
# value: 0
# description: Not specified.

# name: MAV_BATTERY_TYPE_LIPO
# value: 1
# description: Lithium polymer battery

# name: MAV_BATTERY_TYPE_LIFE
# value: 2
# description: Lithium-iron-phosphate battery

# name: MAV_BATTERY_TYPE_LION
# value: 3
# description: Lithium-ION battery

# name: MAV_BATTERY_TYPE_NIMH
# value: 4
# description: Nickel metal hydride battery

 
mav_battery_function = mavutil.mavlink.MAV_BATTERY_FUNCTION
# name: MAV_BATTERY_FUNCTION_UNKNOWN
# value: 0
# description: Battery function is unknown

# name: MAV_BATTERY_FUNCTION_ALL
# value: 1
# description: Battery supports all flight systems

# name: MAV_BATTERY_FUNCTION_PROPULSION
# value: 2
# description: Battery for the propulsion system

# name: MAV_BATTERY_FUNCTION_AVIONICS
# value: 3
# description: Avionics battery

# name: MAV_BATTERY_TYPE_PAYLOAD
# value: 4
# description: Payload battery

 
mav_battery_charge_state = mavutil.mavlink.MAV_BATTERY_CHARGE_STATE
# name: MAV_BATTERY_CHARGE_STATE_UNDEFINED
# value: 0
# description: Low battery state is not provided

# name: MAV_BATTERY_CHARGE_STATE_OK
# value: 1
# description: Battery is not in low state. Normal operation.

# name: MAV_BATTERY_CHARGE_STATE_LOW
# value: 2
# description: Battery state is low, warn and monitor close.

# name: MAV_BATTERY_CHARGE_STATE_CRITICAL
# value: 3
# description: Battery state is critical, return or abort immediately.

# name: MAV_BATTERY_CHARGE_STATE_EMERGENCY
# value: 4
# description: Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to prevent damage.

# name: MAV_BATTERY_CHARGE_STATE_FAILED
# value: 5
# description: Battery failed, damage unavoidable. Possible causes (faults) are listed in MAV_BATTERY_FAULT.

# name: MAV_BATTERY_CHARGE_STATE_UNHEALTHY
# value: 6
# description: Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in MAV_BATTERY_FAULT.

# name: MAV_BATTERY_CHARGE_STATE_CHARGING
# value: 7
# description: Battery is charging.

 
mav_battery_mode = mavutil.mavlink.MAV_BATTERY_MODE
# name: MAV_BATTERY_MODE_UNKNOWN
# value: 0
# description: Battery mode not supported/unknown battery mode/normal operation.

# name: MAV_BATTERY_MODE_AUTO_DISCHARGING
# value: 1
# description: Battery is auto discharging (towards storage level).

# name: MAV_BATTERY_MODE_HOT_SWAP
# value: 2
# description: Battery in hot-swap mode (current limited to prevent spikes that might damage sensitive electrical circuits).

 
mav_battery_fault = mavutil.mavlink.MAV_BATTERY_FAULT
# name: MAV_BATTERY_FAULT_DEEP_DISCHARGE
# value: 1
# description: Battery has deep discharged.

# name: MAV_BATTERY_FAULT_SPIKES
# value: 2
# description: Voltage spikes.

# name: MAV_BATTERY_FAULT_CELL_FAIL
# value: 4
# description: One or more cells have failed. Battery should also report MAV_BATTERY_CHARGE_STATE_FAILE (and should not be used).

# name: MAV_BATTERY_FAULT_OVER_CURRENT
# value: 8
# description: Over-current fault.

# name: MAV_BATTERY_FAULT_OVER_TEMPERATURE
# value: 16
# description: Over-temperature fault.

# name: MAV_BATTERY_FAULT_UNDER_TEMPERATURE
# value: 32
# description: Under-temperature fault.

# name: MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE
# value: 64
# description: Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage).

# name: MAV_BATTERY_FAULT_INCOMPATIBLE_FIRMWARE
# value: 128
# description: Battery firmware is not compatible with current autopilot firmware.

# name: BATTERY_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION
# value: 256
# description: Battery is not compatible due to cell configuration (e.g. 5s1p when vehicle requires 6s).

 
mav_generator_status_flag = mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG
# name: MAV_GENERATOR_STATUS_FLAG_OFF
# value: 1
# description: Generator is off.

# name: MAV_GENERATOR_STATUS_FLAG_READY
# value: 2
# description: Generator is ready to start generating power.

# name: MAV_GENERATOR_STATUS_FLAG_GENERATING
# value: 4
# description: Generator is generating power.

# name: MAV_GENERATOR_STATUS_FLAG_CHARGING
# value: 8
# description: Generator is charging the batteries (generating enough power to charge and provide the load).

# name: MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER
# value: 16
# description: Generator is operating at a reduced maximum power.

# name: MAV_GENERATOR_STATUS_FLAG_MAXPOWER
# value: 32
# description: Generator is providing the maximum output.

# name: MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING
# value: 64
# description: Generator is near the maximum operating temperature, cooling is insufficient.

# name: MAV_GENERATOR_STATUS_FLAG_OVERTEMP_FAULT
# value: 128
# description: Generator hit the maximum operating temperature and shutdown.

# name: MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING
# value: 256
# description: Power electronics are near the maximum operating temperature, cooling is insufficient.

# name: MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT
# value: 512
# description: Power electronics hit the maximum operating temperature and shutdown.

# name: MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT
# value: 1024
# description: Power electronics experienced a fault and shutdown.

# name: MAV_GENERATOR_STATUS_FLAG_POWERSOURCE_FAULT
# value: 2048
# description: The power source supplying the generator failed e.g. mechanical generator stopped, tether is no longer providing power, solar cell is in shade, hydrogen reaction no longer happening.

# name: MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING
# value: 4096
# description: Generator controller having communication problems.

# name: MAV_GENERATOR_STATUS_FLAG_COOLING_WARNING
# value: 8192
# description: Power electronic or generator cooling system error.

# name: MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT
# value: 16384
# description: Generator controller power rail experienced a fault.

# name: MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT
# value: 32768
# description: Generator controller exceeded the overcurrent threshold and shutdown to prevent damage.

# name: MAV_GENERATOR_STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT
# value: 65536
# description: Generator controller detected a high current going into the batteries and shutdown to prevent battery damage.

# name: MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT
# value: 131072
# description: Generator controller exceeded it's overvoltage threshold and shutdown to prevent it exceeding the voltage rating.

# name: MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT
# value: 262144
# description: Batteries are under voltage (generator will not start).

# name: MAV_GENERATOR_STATUS_FLAG_START_INHIBITED
# value: 524288
# description: Generator start is inhibited by e.g. a safety switch.

# name: MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED
# value: 1048576
# description: Generator requires maintenance.

# name: MAV_GENERATOR_STATUS_FLAG_WARMING_UP
# value: 2097152
# description: Generator is not ready to generate yet.

# name: MAV_GENERATOR_STATUS_FLAG_IDLE
# value: 4194304
# description: Generator is idle.

 
mav_vtol_state = mavutil.mavlink.MAV_VTOL_STATE
# name: MAV_VTOL_STATE_UNDEFINED
# value: 0
# description: MAV is not configured as VTOL

# name: MAV_VTOL_STATE_TRANSITION_TO_FW
# value: 1
# description: VTOL is in transition from multicopter to fixed-wing

# name: MAV_VTOL_STATE_TRANSITION_TO_MC
# value: 2
# description: VTOL is in transition from fixed-wing to multicopter

# name: MAV_VTOL_STATE_MC
# value: 3
# description: VTOL is in multicopter state

# name: MAV_VTOL_STATE_FW
# value: 4
# description: VTOL is in fixed-wing state

 
mav_landed_state = mavutil.mavlink.MAV_LANDED_STATE
# name: MAV_LANDED_STATE_UNDEFINED
# value: 0
# description: MAV landed state is unknown

# name: MAV_LANDED_STATE_ON_GROUND
# value: 1
# description: MAV is landed (on ground)

# name: MAV_LANDED_STATE_IN_AIR
# value: 2
# description: MAV is in air

# name: MAV_LANDED_STATE_TAKEOFF
# value: 3
# description: MAV currently taking off

# name: MAV_LANDED_STATE_LANDING
# value: 4
# description: MAV currently landing

 
adsb_altitude_type = mavutil.mavlink.ADSB_ALTITUDE_TYPE
# name: ADSB_ALTITUDE_TYPE_PRESSURE_QNH
# value: 0
# description: Altitude reported from a Baro source using QNH reference

# name: ADSB_ALTITUDE_TYPE_GEOMETRIC
# value: 1
# description: Altitude reported from a GNSS source

 
adsb_emitter_type = mavutil.mavlink.ADSB_EMITTER_TYPE
# name: ADSB_EMITTER_TYPE_NO_INFO
# value: 0
# description: 

# name: ADSB_EMITTER_TYPE_LIGHT
# value: 1
# description: 

# name: ADSB_EMITTER_TYPE_SMALL
# value: 2
# description: 

# name: ADSB_EMITTER_TYPE_LARGE
# value: 3
# description: 

# name: ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE
# value: 4
# description: 

# name: ADSB_EMITTER_TYPE_HEAVY
# value: 5
# description: 

# name: ADSB_EMITTER_TYPE_HIGHLY_MANUV
# value: 6
# description: 

# name: ADSB_EMITTER_TYPE_ROTOCRAFT
# value: 7
# description: 

# name: ADSB_EMITTER_TYPE_UNASSIGNED
# value: 8
# description: 

# name: ADSB_EMITTER_TYPE_GLIDER
# value: 9
# description: 

# name: ADSB_EMITTER_TYPE_LIGHTER_AIR
# value: 10
# description: 

# name: ADSB_EMITTER_TYPE_PARACHUTE
# value: 11
# description: 

# name: ADSB_EMITTER_TYPE_ULTRA_LIGHT
# value: 12
# description: 

# name: ADSB_EMITTER_TYPE_UNASSIGNED2
# value: 13
# description: 

# name: ADSB_EMITTER_TYPE_UAV
# value: 14
# description: 

# name: ADSB_EMITTER_TYPE_SPACE
# value: 15
# description: 

# name: ADSB_EMITTER_TYPE_UNASSGINED3
# value: 16
# description: 

# name: ADSB_EMITTER_TYPE_EMERGENCY_SURFACE
# value: 17
# description: 

# name: ADSB_EMITTER_TYPE_SERVICE_SURFACE
# value: 18
# description: 

# name: ADSB_EMITTER_TYPE_POINT_OBSTACLE
# value: 19
# description: 

 
adsb_flags = mavutil.mavlink.ADSB_FLAGS
# name: ADSB_FLAGS_VALID_COORDS
# value: 1
# description: 

# name: ADSB_FLAGS_VALID_ALTITUDE
# value: 2
# description: 

# name: ADSB_FLAGS_VALID_HEADING
# value: 4
# description: 

# name: ADSB_FLAGS_VALID_VELOCITY
# value: 8
# description: 

# name: ADSB_FLAGS_VALID_CALLSIGN
# value: 16
# description: 

# name: ADSB_FLAGS_VALID_SQUAWK
# value: 32
# description: 

# name: ADSB_FLAGS_SIMULATED
# value: 64
# description: 

# name: ADSB_FLAGS_VERTICAL_VELOCITY_VALID
# value: 128
# description: 

# name: ADSB_FLAGS_BARO_VALID
# value: 256
# description: 

# name: ADSB_FLAGS_SOURCE_UAT
# value: 32768
# description: 

 
mav_do_reposition_flags = mavutil.mavlink.MAV_DO_REPOSITION_FLAGS
# name: MAV_DO_REPOSITION_FLAGS_CHANGE_MODE
# value: 1
# description: The aircraft should immediately transition into guided. This should not be set for follow me applications

 
estimator_status_flags = mavutil.mavlink.ESTIMATOR_STATUS_FLAGS
# name: ESTIMATOR_ATTITUDE
# value: 1
# description: True if the attitude estimate is good

# name: ESTIMATOR_VELOCITY_HORIZ
# value: 2
# description: True if the horizontal velocity estimate is good

# name: ESTIMATOR_VELOCITY_VERT
# value: 4
# description: True if the  vertical velocity estimate is good

# name: ESTIMATOR_POS_HORIZ_REL
# value: 8
# description: True if the horizontal position (relative) estimate is good

# name: ESTIMATOR_POS_HORIZ_ABS
# value: 16
# description: True if the horizontal position (absolute) estimate is good

# name: ESTIMATOR_POS_VERT_ABS
# value: 32
# description: True if the vertical position (absolute) estimate is good

# name: ESTIMATOR_POS_VERT_AGL
# value: 64
# description: True if the vertical position (above ground) estimate is good

# name: ESTIMATOR_CONST_POS_MODE
# value: 128
# description: True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)

# name: ESTIMATOR_PRED_POS_HORIZ_REL
# value: 256
# description: True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate

# name: ESTIMATOR_PRED_POS_HORIZ_ABS
# value: 512
# description: True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate

# name: ESTIMATOR_GPS_GLITCH
# value: 1024
# description: True if the EKF has detected a GPS glitch

# name: ESTIMATOR_ACCEL_ERROR
# value: 2048
# description: True if the EKF has detected bad accelerometer data

 
motor_test_order = mavutil.mavlink.MOTOR_TEST_ORDER
# name: MOTOR_TEST_ORDER_DEFAULT
# value: 0
# description: Default autopilot motor test method.

# name: MOTOR_TEST_ORDER_SEQUENCE
# value: 1
# description: Motor numbers are specified as their index in a predefined vehicle-specific sequence.

# name: MOTOR_TEST_ORDER_BOARD
# value: 2
# description: Motor numbers are specified as the output as labeled on the board.

 
motor_test_throttle_type = mavutil.mavlink.MOTOR_TEST_THROTTLE_TYPE
# name: MOTOR_TEST_THROTTLE_PERCENT
# value: 0
# description: Throttle as a percentage (0 ~ 100)

# name: MOTOR_TEST_THROTTLE_PWM
# value: 1
# description: Throttle as an absolute PWM value (normally in range of 1000~2000).

# name: MOTOR_TEST_THROTTLE_PILOT
# value: 2
# description: Throttle pass-through from pilot's transmitter.

# name: MOTOR_TEST_COMPASS_CAL
# value: 3
# description: Per-motor compass calibration test.

 
gps_input_ignore_flags = mavutil.mavlink.GPS_INPUT_IGNORE_FLAGS
# name: GPS_INPUT_IGNORE_FLAG_ALT
# value: 1
# description: ignore altitude field

# name: GPS_INPUT_IGNORE_FLAG_HDOP
# value: 2
# description: ignore hdop field

# name: GPS_INPUT_IGNORE_FLAG_VDOP
# value: 4
# description: ignore vdop field

# name: GPS_INPUT_IGNORE_FLAG_VEL_HORIZ
# value: 8
# description: ignore horizontal velocity field (vn and ve)

# name: GPS_INPUT_IGNORE_FLAG_VEL_VERT
# value: 16
# description: ignore vertical velocity field (vd)

# name: GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY
# value: 32
# description: ignore speed accuracy field

# name: GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY
# value: 64
# description: ignore horizontal accuracy field

# name: GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY
# value: 128
# description: ignore vertical accuracy field

 
mav_collision_action = mavutil.mavlink.MAV_COLLISION_ACTION
# name: MAV_COLLISION_ACTION_NONE
# value: 0
# description: Ignore any potential collisions

# name: MAV_COLLISION_ACTION_REPORT
# value: 1
# description: Report potential collision

# name: MAV_COLLISION_ACTION_ASCEND_OR_DESCEND
# value: 2
# description: Ascend or Descend to avoid threat

# name: MAV_COLLISION_ACTION_MOVE_HORIZONTALLY
# value: 3
# description: Move horizontally to avoid threat

# name: MAV_COLLISION_ACTION_MOVE_PERPENDICULAR
# value: 4
# description: Aircraft to move perpendicular to the collision's velocity vector

# name: MAV_COLLISION_ACTION_RTL
# value: 5
# description: Aircraft to fly directly back to its launch point

# name: MAV_COLLISION_ACTION_HOVER
# value: 6
# description: Aircraft to stop in place

 
mav_collision_threat_level = mavutil.mavlink.MAV_COLLISION_THREAT_LEVEL
# name: MAV_COLLISION_THREAT_LEVEL_NONE
# value: 0
# description: Not a threat

# name: MAV_COLLISION_THREAT_LEVEL_LOW
# value: 1
# description: Craft is mildly concerned about this threat

# name: MAV_COLLISION_THREAT_LEVEL_HIGH
# value: 2
# description: Craft is panicking, and may take actions to avoid threat

 
mav_collision_src = mavutil.mavlink.MAV_COLLISION_SRC
# name: MAV_COLLISION_SRC_ADSB
# value: 0
# description: ID field references ADSB_VEHICLE packets

# name: MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT
# value: 1
# description: ID field references MAVLink SRC ID

 
gps_fix_type = mavutil.mavlink.GPS_FIX_TYPE
# name: GPS_FIX_TYPE_NO_GPS
# value: 0
# description: No GPS connected

# name: GPS_FIX_TYPE_NO_FIX
# value: 1
# description: No position information, GPS is connected

# name: GPS_FIX_TYPE_2D_FIX
# value: 2
# description: 2D position

# name: GPS_FIX_TYPE_3D_FIX
# value: 3
# description: 3D position

# name: GPS_FIX_TYPE_DGPS
# value: 4
# description: DGPS/SBAS aided 3D position

# name: GPS_FIX_TYPE_RTK_FLOAT
# value: 5
# description: RTK float, 3D position

# name: GPS_FIX_TYPE_RTK_FIXED
# value: 6
# description: RTK Fixed, 3D position

# name: GPS_FIX_TYPE_STATIC
# value: 7
# description: Static fixed, typically used for base stations

# name: GPS_FIX_TYPE_PPP
# value: 8
# description: PPP, 3D position.

 
rtk_baseline_coordinate_system = mavutil.mavlink.RTK_BASELINE_COORDINATE_SYSTEM
# name: RTK_BASELINE_COORDINATE_SYSTEM_ECEF
# value: 0
# description: Earth-centered, Earth-fixed

# name: RTK_BASELINE_COORDINATE_SYSTEM_NED
# value: 1
# description: RTK basestation centered, north, east, down

 
landing_target_type = mavutil.mavlink.LANDING_TARGET_TYPE
# name: LANDING_TARGET_TYPE_LIGHT_BEACON
# value: 0
# description: Landing target signaled by light beacon (ex: IR-LOCK)

# name: LANDING_TARGET_TYPE_RADIO_BEACON
# value: 1
# description: Landing target signaled by radio beacon (ex: ILS, NDB)

# name: LANDING_TARGET_TYPE_VISION_FIDUCIAL
# value: 2
# description: Landing target represented by a fiducial marker (ex: ARTag)

# name: LANDING_TARGET_TYPE_VISION_OTHER
# value: 3
# description: Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)

 
vtol_transition_heading = mavutil.mavlink.VTOL_TRANSITION_HEADING
# name: VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT
# value: 0
# description: Respect the heading configuration of the vehicle.

# name: VTOL_TRANSITION_HEADING_NEXT_WAYPOINT
# value: 1
# description: Use the heading pointing towards the next waypoint.

# name: VTOL_TRANSITION_HEADING_TAKEOFF
# value: 2
# description: Use the heading on takeoff (while sitting on the ground).

# name: VTOL_TRANSITION_HEADING_SPECIFIED
# value: 3
# description: Use the specified heading in parameter 4.

# name: VTOL_TRANSITION_HEADING_ANY
# value: 4
# description: Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning is active).

 
camera_cap_flags = mavutil.mavlink.CAMERA_CAP_FLAGS
# name: CAMERA_CAP_FLAGS_CAPTURE_VIDEO
# value: 1
# description: Camera is able to record video

# name: CAMERA_CAP_FLAGS_CAPTURE_IMAGE
# value: 2
# description: Camera is able to capture images

# name: CAMERA_CAP_FLAGS_HAS_MODES
# value: 4
# description: Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)

# name: CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE
# value: 8
# description: Camera can capture images while in video mode

# name: CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE
# value: 16
# description: Camera can capture videos while in Photo/Image mode

# name: CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE
# value: 32
# description: Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)

# name: CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM
# value: 64
# description: Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM)

# name: CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS
# value: 128
# description: Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS)

# name: CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM
# value: 256
# description: Camera has video streaming capabilities (request VIDEO_STREAM_INFORMATION with MAV_CMD_REQUEST_MESSAGE for video streaming info)

# name: CAMERA_CAP_FLAGS_HAS_TRACKING_POINT
# value: 512
# description: Camera supports tracking of a point on the camera view.

# name: CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE
# value: 1024
# description: Camera supports tracking of a selection rectangle on the camera view.

# name: CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS
# value: 2048
# description: Camera supports tracking geo status (CAMERA_TRACKING_GEO_STATUS).

 
video_stream_status_flags = mavutil.mavlink.VIDEO_STREAM_STATUS_FLAGS
# name: VIDEO_STREAM_STATUS_FLAGS_RUNNING
# value: 1
# description: Stream is active (running)

# name: VIDEO_STREAM_STATUS_FLAGS_THERMAL
# value: 2
# description: Stream is thermal imaging

 
video_stream_type = mavutil.mavlink.VIDEO_STREAM_TYPE
# name: VIDEO_STREAM_TYPE_RTSP
# value: 0
# description: Stream is RTSP

# name: VIDEO_STREAM_TYPE_RTPUDP
# value: 1
# description: Stream is RTP UDP (URI gives the port number)

# name: VIDEO_STREAM_TYPE_TCP_MPEG
# value: 2
# description: Stream is MPEG on TCP

# name: VIDEO_STREAM_TYPE_MPEG_TS_H264
# value: 3
# description: Stream is h.264 on MPEG TS (URI gives the port number)

 
camera_tracking_status_flags = mavutil.mavlink.CAMERA_TRACKING_STATUS_FLAGS
# name: CAMERA_TRACKING_STATUS_FLAGS_IDLE
# value: 0
# description: Camera is not tracking

# name: CAMERA_TRACKING_STATUS_FLAGS_ACTIVE
# value: 1
# description: Camera is tracking

# name: CAMERA_TRACKING_STATUS_FLAGS_ERROR
# value: 2
# description: Camera tracking in error state

 
camera_tracking_mode = mavutil.mavlink.CAMERA_TRACKING_MODE
# name: CAMERA_TRACKING_MODE_NONE
# value: 0
# description: Not tracking

# name: CAMERA_TRACKING_MODE_POINT
# value: 1
# description: Target is a point

# name: CAMERA_TRACKING_MODE_RECTANGLE
# value: 2
# description: Target is a rectangle

 
camera_tracking_target_data = mavutil.mavlink.CAMERA_TRACKING_TARGET_DATA
# name: CAMERA_TRACKING_TARGET_DATA_NONE
# value: 0
# description: No target data

# name: CAMERA_TRACKING_TARGET_DATA_EMBEDDED
# value: 1
# description: Target data embedded in image data (proprietary)

# name: CAMERA_TRACKING_TARGET_DATA_RENDERED
# value: 2
# description: Target data rendered in image

# name: CAMERA_TRACKING_TARGET_DATA_IN_STATUS
# value: 4
# description: Target data within status message (Point or Rectangle)

 
camera_zoom_type = mavutil.mavlink.CAMERA_ZOOM_TYPE
# name: ZOOM_TYPE_STEP
# value: 0
# description: Zoom one step increment (-1 for wide, 1 for tele)

# name: ZOOM_TYPE_CONTINUOUS
# value: 1
# description: Continuous zoom up/down until stopped (-1 for wide, 1 for tele, 0 to stop zooming)

# name: ZOOM_TYPE_RANGE
# value: 2
# description: Zoom value as proportion of full camera range (a value between 0.0 and 100.0)

# name: ZOOM_TYPE_FOCAL_LENGTH
# value: 3
# description: Zoom value/variable focal length in milimetres. Note that there is no message to get the valid zoom range of the camera, so this can type can only be used for cameras where the zoom range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera)

 
set_focus_type = mavutil.mavlink.SET_FOCUS_TYPE
# name: FOCUS_TYPE_STEP
# value: 0
# description: Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity).

# name: FOCUS_TYPE_CONTINUOUS
# value: 1
# description: Continuous focus up/down until stopped (-1 for focusing in, 1 for focusing out towards infinity, 0 to stop focusing)

# name: FOCUS_TYPE_RANGE
# value: 2
# description: Focus value as proportion of full camera focus range (a value between 0.0 and 100.0)

# name: FOCUS_TYPE_METERS
# value: 3
# description: Focus value in metres. Note that there is no message to get the valid focus range of the camera, so this can type can only be used for cameras where the range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera).

 
param_ack = mavutil.mavlink.PARAM_ACK
# name: PARAM_ACK_ACCEPTED
# value: 0
# description: Parameter value ACCEPTED and SET

# name: PARAM_ACK_VALUE_UNSUPPORTED
# value: 1
# description: Parameter value UNKNOWN/UNSUPPORTED

# name: PARAM_ACK_FAILED
# value: 2
# description: Parameter failed to set

# name: PARAM_ACK_IN_PROGRESS
# value: 3
# description: Parameter value received but not yet set/accepted. A subsequent PARAM_ACK_TRANSACTION or PARAM_EXT_ACK with the final result will follow once operation is completed. This is returned immediately for parameters that take longer to set, indicating taht the the parameter was recieved and does not need to be resent.

 
camera_mode = mavutil.mavlink.CAMERA_MODE
# name: CAMERA_MODE_IMAGE
# value: 0
# description: Camera is in image/photo capture mode.

# name: CAMERA_MODE_VIDEO
# value: 1
# description: Camera is in video capture mode.

# name: CAMERA_MODE_IMAGE_SURVEY
# value: 2
# description: Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys.

 
mav_arm_auth_denied_reason = mavutil.mavlink.MAV_ARM_AUTH_DENIED_REASON
# name: MAV_ARM_AUTH_DENIED_REASON_GENERIC
# value: 0
# description: Not a specific reason

# name: MAV_ARM_AUTH_DENIED_REASON_NONE
# value: 1
# description: Authorizer will send the error as string to GCS

# name: MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT
# value: 2
# description: At least one waypoint have a invalid value

# name: MAV_ARM_AUTH_DENIED_REASON_TIMEOUT
# value: 3
# description: Timeout in the authorizer process(in case it depends on network)

# name: MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE
# value: 4
# description: Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that caused it to be denied.

# name: MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER
# value: 5
# description: Weather is not good to fly

 
rc_type = mavutil.mavlink.RC_TYPE
# name: RC_TYPE_SPEKTRUM_DSM2
# value: 0
# description: Spektrum DSM2

# name: RC_TYPE_SPEKTRUM_DSMX
# value: 1
# description: Spektrum DSMX

 
position_target_typemask = mavutil.mavlink.POSITION_TARGET_TYPEMASK
# name: POSITION_TARGET_TYPEMASK_X_IGNORE
# value: 1
# description: Ignore position x

# name: POSITION_TARGET_TYPEMASK_Y_IGNORE
# value: 2
# description: Ignore position y

# name: POSITION_TARGET_TYPEMASK_Z_IGNORE
# value: 4
# description: Ignore position z

# name: POSITION_TARGET_TYPEMASK_VX_IGNORE
# value: 8
# description: Ignore velocity x

# name: POSITION_TARGET_TYPEMASK_VY_IGNORE
# value: 16
# description: Ignore velocity y

# name: POSITION_TARGET_TYPEMASK_VZ_IGNORE
# value: 32
# description: Ignore velocity z

# name: POSITION_TARGET_TYPEMASK_AX_IGNORE
# value: 64
# description: Ignore acceleration x

# name: POSITION_TARGET_TYPEMASK_AY_IGNORE
# value: 128
# description: Ignore acceleration y

# name: POSITION_TARGET_TYPEMASK_AZ_IGNORE
# value: 256
# description: Ignore acceleration z

# name: POSITION_TARGET_TYPEMASK_FORCE_SET
# value: 512
# description: Use force instead of acceleration

# name: POSITION_TARGET_TYPEMASK_YAW_IGNORE
# value: 1024
# description: Ignore yaw

# name: POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
# value: 2048
# description: Ignore yaw rate

 
attitude_target_typemask = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK
# name: ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
# value: 1
# description: Ignore body roll rate

# name: ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
# value: 2
# description: Ignore body pitch rate

# name: ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
# value: 4
# description: Ignore body yaw rate

# name: ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET
# value: 32
# description: Use 3D body thrust setpoint instead of throttle

# name: ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE
# value: 64
# description: Ignore throttle

# name: ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
# value: 128
# description: Ignore attitude

 
utm_flight_state = mavutil.mavlink.UTM_FLIGHT_STATE
# name: UTM_FLIGHT_STATE_UNKNOWN
# value: 1
# description: The flight state can't be determined.

# name: UTM_FLIGHT_STATE_GROUND
# value: 2
# description: UAS on ground.

# name: UTM_FLIGHT_STATE_AIRBORNE
# value: 3
# description: UAS airborne.

# name: UTM_FLIGHT_STATE_EMERGENCY
# value: 16
# description: UAS is in an emergency flight state.

# name: UTM_FLIGHT_STATE_NOCTRL
# value: 32
# description: UAS has no active controls.

 
utm_data_avail_flags = mavutil.mavlink.UTM_DATA_AVAIL_FLAGS
# name: UTM_DATA_AVAIL_FLAGS_TIME_VALID
# value: 1
# description: The field time contains valid data.

# name: UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE
# value: 2
# description: The field uas_id contains valid data.

# name: UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE
# value: 4
# description: The fields lat, lon and h_acc contain valid data.

# name: UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE
# value: 8
# description: The fields alt and v_acc contain valid data.

# name: UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE
# value: 16
# description: The field relative_alt contains valid data.

# name: UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE
# value: 32
# description: The fields vx and vy contain valid data.

# name: UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE
# value: 64
# description: The field vz contains valid data.

# name: UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE
# value: 128
# description: The fields next_lat, next_lon and next_alt contain valid data.

 
cellular_network_radio_type = mavutil.mavlink.CELLULAR_NETWORK_RADIO_TYPE
# name: CELLULAR_NETWORK_RADIO_TYPE_NONE
# value: 0
# description: 

# name: CELLULAR_NETWORK_RADIO_TYPE_GSM
# value: 1
# description: 

# name: CELLULAR_NETWORK_RADIO_TYPE_CDMA
# value: 2
# description: 

# name: CELLULAR_NETWORK_RADIO_TYPE_WCDMA
# value: 3
# description: 

# name: CELLULAR_NETWORK_RADIO_TYPE_LTE
# value: 4
# description: 

 
cellular_status_flag = mavutil.mavlink.CELLULAR_STATUS_FLAG
# name: CELLULAR_STATUS_FLAG_UNKNOWN
# value: 0
# description: State unknown or not reportable.

# name: CELLULAR_STATUS_FLAG_FAILED
# value: 1
# description: Modem is unusable

# name: CELLULAR_STATUS_FLAG_INITIALIZING
# value: 2
# description: Modem is being initialized

# name: CELLULAR_STATUS_FLAG_LOCKED
# value: 3
# description: Modem is locked

# name: CELLULAR_STATUS_FLAG_DISABLED
# value: 4
# description: Modem is not enabled and is powered down

# name: CELLULAR_STATUS_FLAG_DISABLING
# value: 5
# description: Modem is currently transitioning to the CELLULAR_STATUS_FLAG_DISABLED state

# name: CELLULAR_STATUS_FLAG_ENABLING
# value: 6
# description: Modem is currently transitioning to the CELLULAR_STATUS_FLAG_ENABLED state

# name: CELLULAR_STATUS_FLAG_ENABLED
# value: 7
# description: Modem is enabled and powered on but not registered with a network provider and not available for data connections

# name: CELLULAR_STATUS_FLAG_SEARCHING
# value: 8
# description: Modem is searching for a network provider to register

# name: CELLULAR_STATUS_FLAG_REGISTERED
# value: 9
# description: Modem is registered with a network provider, and data connections and messaging may be available for use

# name: CELLULAR_STATUS_FLAG_DISCONNECTING
# value: 10
# description: Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated

# name: CELLULAR_STATUS_FLAG_CONNECTING
# value: 11
# description: Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered

# name: CELLULAR_STATUS_FLAG_CONNECTED
# value: 12
# description: One or more packet data bearers is active and connected

 
cellular_network_failed_reason = mavutil.mavlink.CELLULAR_NETWORK_FAILED_REASON
# name: CELLULAR_NETWORK_FAILED_REASON_NONE
# value: 0
# description: No error

# name: CELLULAR_NETWORK_FAILED_REASON_UNKNOWN
# value: 1
# description: Error state is unknown

# name: CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING
# value: 2
# description: SIM is required for the modem but missing

# name: CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR
# value: 3
# description: SIM is available, but not usuable for connection

 
precision_land_mode = mavutil.mavlink.PRECISION_LAND_MODE
# name: PRECISION_LAND_MODE_DISABLED
# value: 0
# description: Normal (non-precision) landing.

# name: PRECISION_LAND_MODE_OPPORTUNISTIC
# value: 1
# description: Use precision landing if beacon detected when land command accepted, otherwise land normally.

# name: PRECISION_LAND_MODE_REQUIRED
# value: 2
# description: Use precision landing, searching for beacon if not found when land command accepted (land normally if beacon cannot be found).

 
parachute_action = mavutil.mavlink.PARACHUTE_ACTION
# name: PARACHUTE_DISABLE
# value: 0
# description: Disable auto-release of parachute (i.e. release triggered by crash detectors).

# name: PARACHUTE_ENABLE
# value: 1
# description: Enable auto-release of parachute.

# name: PARACHUTE_RELEASE
# value: 2
# description: Release parachute and kill motors.

 
mav_tunnel_payload_type = mavutil.mavlink.MAV_TUNNEL_PAYLOAD_TYPE
# name: MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN
# value: 0
# description: Encoding of payload unknown.

# name: MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0
# value: 200
# description: Registered for STorM32 gimbal controller.

# name: MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1
# value: 201
# description: Registered for STorM32 gimbal controller.

# name: MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2
# value: 202
# description: Registered for STorM32 gimbal controller.

# name: MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3
# value: 203
# description: Registered for STorM32 gimbal controller.

# name: MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4
# value: 204
# description: Registered for STorM32 gimbal controller.

# name: MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5
# value: 205
# description: Registered for STorM32 gimbal controller.

# name: MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6
# value: 206
# description: Registered for STorM32 gimbal controller.

# name: MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7
# value: 207
# description: Registered for STorM32 gimbal controller.

# name: MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8
# value: 208
# description: Registered for STorM32 gimbal controller.

# name: MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9
# value: 209
# description: Registered for STorM32 gimbal controller.

 
mav_odid_id_type = mavutil.mavlink.MAV_ODID_ID_TYPE
# name: MAV_ODID_ID_TYPE_NONE
# value: 0
# description: No type defined.

# name: MAV_ODID_ID_TYPE_SERIAL_NUMBER
# value: 1
# description: Manufacturer Serial Number (ANSI/CTA-2063 format).

# name: MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID
# value: 2
# description: CAA (Civil Aviation Authority) registered ID. Format: [ICAO Country Code].[CAA Assigned ID].

# name: MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID
# value: 3
# description: UTM (Unmanned Traffic Management) assigned UUID (RFC4122).

 
mav_odid_ua_type = mavutil.mavlink.MAV_ODID_UA_TYPE
# name: MAV_ODID_UA_TYPE_NONE
# value: 0
# description: No UA (Unmanned Aircraft) type defined.

# name: MAV_ODID_UA_TYPE_AEROPLANE
# value: 1
# description: Aeroplane/Airplane. Fixed wing.

# name: MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR
# value: 2
# description: Helicopter or multirotor.

# name: MAV_ODID_UA_TYPE_GYROPLANE
# value: 3
# description: Gyroplane.

# name: MAV_ODID_UA_TYPE_HYBRID_LIFT
# value: 4
# description: VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically.

# name: MAV_ODID_UA_TYPE_ORNITHOPTER
# value: 5
# description: Ornithopter.

# name: MAV_ODID_UA_TYPE_GLIDER
# value: 6
# description: Glider.

# name: MAV_ODID_UA_TYPE_KITE
# value: 7
# description: Kite.

# name: MAV_ODID_UA_TYPE_FREE_BALLOON
# value: 8
# description: Free Balloon.

# name: MAV_ODID_UA_TYPE_CAPTIVE_BALLOON
# value: 9
# description: Captive Balloon.

# name: MAV_ODID_UA_TYPE_AIRSHIP
# value: 10
# description: Airship. E.g. a blimp.

# name: MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE
# value: 11
# description: Free Fall/Parachute (unpowered).

# name: MAV_ODID_UA_TYPE_ROCKET
# value: 12
# description: Rocket.

# name: MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT
# value: 13
# description: Tethered powered aircraft.

# name: MAV_ODID_UA_TYPE_GROUND_OBSTACLE
# value: 14
# description: Ground Obstacle.

# name: MAV_ODID_UA_TYPE_OTHER
# value: 15
# description: Other type of aircraft not listed earlier.

 
mav_odid_status = mavutil.mavlink.MAV_ODID_STATUS
# name: MAV_ODID_STATUS_UNDECLARED
# value: 0
# description: The status of the (UA) Unmanned Aircraft is undefined.

# name: MAV_ODID_STATUS_GROUND
# value: 1
# description: The UA is on the ground.

# name: MAV_ODID_STATUS_AIRBORNE
# value: 2
# description: The UA is in the air.

# name: MAV_ODID_STATUS_EMERGENCY
# value: 3
# description: The UA is having an emergency.

 
mav_odid_height_ref = mavutil.mavlink.MAV_ODID_HEIGHT_REF
# name: MAV_ODID_HEIGHT_REF_OVER_TAKEOFF
# value: 0
# description: The height field is relative to the take-off location.

# name: MAV_ODID_HEIGHT_REF_OVER_GROUND
# value: 1
# description: The height field is relative to ground.

 
mav_odid_hor_acc = mavutil.mavlink.MAV_ODID_HOR_ACC
# name: MAV_ODID_HOR_ACC_UNKNOWN
# value: 0
# description: The horizontal accuracy is unknown.

# name: MAV_ODID_HOR_ACC_10NM
# value: 1
# description: The horizontal accuracy is smaller than 10 Nautical Miles. 18.52 km.

# name: MAV_ODID_HOR_ACC_4NM
# value: 2
# description: The horizontal accuracy is smaller than 4 Nautical Miles. 7.408 km.

# name: MAV_ODID_HOR_ACC_2NM
# value: 3
# description: The horizontal accuracy is smaller than 2 Nautical Miles. 3.704 km.

# name: MAV_ODID_HOR_ACC_1NM
# value: 4
# description: The horizontal accuracy is smaller than 1 Nautical Miles. 1.852 km.

# name: MAV_ODID_HOR_ACC_0_5NM
# value: 5
# description: The horizontal accuracy is smaller than 0.5 Nautical Miles. 926 m.

# name: MAV_ODID_HOR_ACC_0_3NM
# value: 6
# description: The horizontal accuracy is smaller than 0.3 Nautical Miles. 555.6 m.

# name: MAV_ODID_HOR_ACC_0_1NM
# value: 7
# description: The horizontal accuracy is smaller than 0.1 Nautical Miles. 185.2 m.

# name: MAV_ODID_HOR_ACC_0_05NM
# value: 8
# description: The horizontal accuracy is smaller than 0.05 Nautical Miles. 92.6 m.

# name: MAV_ODID_HOR_ACC_30_METER
# value: 9
# description: The horizontal accuracy is smaller than 30 meter.

# name: MAV_ODID_HOR_ACC_10_METER
# value: 10
# description: The horizontal accuracy is smaller than 10 meter.

# name: MAV_ODID_HOR_ACC_3_METER
# value: 11
# description: The horizontal accuracy is smaller than 3 meter.

# name: MAV_ODID_HOR_ACC_1_METER
# value: 12
# description: The horizontal accuracy is smaller than 1 meter.

 
mav_odid_ver_acc = mavutil.mavlink.MAV_ODID_VER_ACC
# name: MAV_ODID_VER_ACC_UNKNOWN
# value: 0
# description: The vertical accuracy is unknown.

# name: MAV_ODID_VER_ACC_150_METER
# value: 1
# description: The vertical accuracy is smaller than 150 meter.

# name: MAV_ODID_VER_ACC_45_METER
# value: 2
# description: The vertical accuracy is smaller than 45 meter.

# name: MAV_ODID_VER_ACC_25_METER
# value: 3
# description: The vertical accuracy is smaller than 25 meter.

# name: MAV_ODID_VER_ACC_10_METER
# value: 4
# description: The vertical accuracy is smaller than 10 meter.

# name: MAV_ODID_VER_ACC_3_METER
# value: 5
# description: The vertical accuracy is smaller than 3 meter.

# name: MAV_ODID_VER_ACC_1_METER
# value: 6
# description: The vertical accuracy is smaller than 1 meter.

 
mav_odid_speed_acc = mavutil.mavlink.MAV_ODID_SPEED_ACC
# name: MAV_ODID_SPEED_ACC_UNKNOWN
# value: 0
# description: The speed accuracy is unknown.

# name: MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND
# value: 1
# description: The speed accuracy is smaller than 10 meters per second.

# name: MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND
# value: 2
# description: The speed accuracy is smaller than 3 meters per second.

# name: MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND
# value: 3
# description: The speed accuracy is smaller than 1 meters per second.

# name: MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND
# value: 4
# description: The speed accuracy is smaller than 0.3 meters per second.

 
mav_odid_time_acc = mavutil.mavlink.MAV_ODID_TIME_ACC
# name: MAV_ODID_TIME_ACC_UNKNOWN
# value: 0
# description: The timestamp accuracy is unknown.

# name: MAV_ODID_TIME_ACC_0_1_SECOND
# value: 1
# description: The timestamp accuracy is smaller than or equal to 0.1 second.

# name: MAV_ODID_TIME_ACC_0_2_SECOND
# value: 2
# description: The timestamp accuracy is smaller than or equal to 0.2 second.

# name: MAV_ODID_TIME_ACC_0_3_SECOND
# value: 3
# description: The timestamp accuracy is smaller than or equal to 0.3 second.

# name: MAV_ODID_TIME_ACC_0_4_SECOND
# value: 4
# description: The timestamp accuracy is smaller than or equal to 0.4 second.

# name: MAV_ODID_TIME_ACC_0_5_SECOND
# value: 5
# description: The timestamp accuracy is smaller than or equal to 0.5 second.

# name: MAV_ODID_TIME_ACC_0_6_SECOND
# value: 6
# description: The timestamp accuracy is smaller than or equal to 0.6 second.

# name: MAV_ODID_TIME_ACC_0_7_SECOND
# value: 7
# description: The timestamp accuracy is smaller than or equal to 0.7 second.

# name: MAV_ODID_TIME_ACC_0_8_SECOND
# value: 8
# description: The timestamp accuracy is smaller than or equal to 0.8 second.

# name: MAV_ODID_TIME_ACC_0_9_SECOND
# value: 9
# description: The timestamp accuracy is smaller than or equal to 0.9 second.

# name: MAV_ODID_TIME_ACC_1_0_SECOND
# value: 10
# description: The timestamp accuracy is smaller than or equal to 1.0 second.

# name: MAV_ODID_TIME_ACC_1_1_SECOND
# value: 11
# description: The timestamp accuracy is smaller than or equal to 1.1 second.

# name: MAV_ODID_TIME_ACC_1_2_SECOND
# value: 12
# description: The timestamp accuracy is smaller than or equal to 1.2 second.

# name: MAV_ODID_TIME_ACC_1_3_SECOND
# value: 13
# description: The timestamp accuracy is smaller than or equal to 1.3 second.

# name: MAV_ODID_TIME_ACC_1_4_SECOND
# value: 14
# description: The timestamp accuracy is smaller than or equal to 1.4 second.

# name: MAV_ODID_TIME_ACC_1_5_SECOND
# value: 15
# description: The timestamp accuracy is smaller than or equal to 1.5 second.

 
mav_odid_auth_type = mavutil.mavlink.MAV_ODID_AUTH_TYPE
# name: MAV_ODID_AUTH_TYPE_NONE
# value: 0
# description: No authentication type is specified.

# name: MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE
# value: 1
# description: Signature for the UAS (Unmanned Aircraft System) ID.

# name: MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE
# value: 2
# description: Signature for the Operator ID.

# name: MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE
# value: 3
# description: Signature for the entire message set.

# name: MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID
# value: 4
# description: Authentication is provided by Network Remote ID.

 
mav_odid_desc_type = mavutil.mavlink.MAV_ODID_DESC_TYPE
# name: MAV_ODID_DESC_TYPE_TEXT
# value: 0
# description: Free-form text description of the purpose of the flight.

 
mav_odid_operator_location_type = mavutil.mavlink.MAV_ODID_OPERATOR_LOCATION_TYPE
# name: MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF
# value: 0
# description: The location of the operator is the same as the take-off location.

# name: MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS
# value: 1
# description: The location of the operator is based on live GNSS data.

# name: MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED
# value: 2
# description: The location of the operator is a fixed location.

 
mav_odid_classification_type = mavutil.mavlink.MAV_ODID_CLASSIFICATION_TYPE
# name: MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED
# value: 0
# description: The classification type for the UA is undeclared.

# name: MAV_ODID_CLASSIFICATION_TYPE_EU
# value: 1
# description: The classification type for the UA follows EU (European Union) specifications.

 
mav_odid_category_eu = mavutil.mavlink.MAV_ODID_CATEGORY_EU
# name: MAV_ODID_CATEGORY_EU_UNDECLARED
# value: 0
# description: The category for the UA, according to the EU specification, is undeclared.

# name: MAV_ODID_CATEGORY_EU_OPEN
# value: 1
# description: The category for the UA, according to the EU specification, is the Open category.

# name: MAV_ODID_CATEGORY_EU_SPECIFIC
# value: 2
# description: The category for the UA, according to the EU specification, is the Specific category.

# name: MAV_ODID_CATEGORY_EU_CERTIFIED
# value: 3
# description: The category for the UA, according to the EU specification, is the Certified category.

 
mav_odid_class_eu = mavutil.mavlink.MAV_ODID_CLASS_EU
# name: MAV_ODID_CLASS_EU_UNDECLARED
# value: 0
# description: The class for the UA, according to the EU specification, is undeclared.

# name: MAV_ODID_CLASS_EU_CLASS_0
# value: 1
# description: The class for the UA, according to the EU specification, is Class 0.

# name: MAV_ODID_CLASS_EU_CLASS_1
# value: 2
# description: The class for the UA, according to the EU specification, is Class 1.

# name: MAV_ODID_CLASS_EU_CLASS_2
# value: 3
# description: The class for the UA, according to the EU specification, is Class 2.

# name: MAV_ODID_CLASS_EU_CLASS_3
# value: 4
# description: The class for the UA, according to the EU specification, is Class 3.

# name: MAV_ODID_CLASS_EU_CLASS_4
# value: 5
# description: The class for the UA, according to the EU specification, is Class 4.

# name: MAV_ODID_CLASS_EU_CLASS_5
# value: 6
# description: The class for the UA, according to the EU specification, is Class 5.

# name: MAV_ODID_CLASS_EU_CLASS_6
# value: 7
# description: The class for the UA, according to the EU specification, is Class 6.

 
mav_odid_operator_id_type = mavutil.mavlink.MAV_ODID_OPERATOR_ID_TYPE
# name: MAV_ODID_OPERATOR_ID_TYPE_CAA
# value: 0
# description: CAA (Civil Aviation Authority) registered operator ID.

 
tune_format = mavutil.mavlink.TUNE_FORMAT
# name: TUNE_FORMAT_QBASIC1_1
# value: 1
# description: Format is QBasic 1.1 Play: https://www.qbasic.net/en/reference/qb11/Statement/PLAY-006.htm.

# name: TUNE_FORMAT_MML_MODERN
# value: 2
# description: Format is Modern Music Markup Language (MML): https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML.

 
component_cap_flags = mavutil.mavlink.COMPONENT_CAP_FLAGS
# name: COMPONENT_CAP_FLAGS_PARAM
# value: 1
# description: Component has parameters, and supports the parameter protocol (PARAM messages).

# name: COMPONENT_CAP_FLAGS_PARAM_EXT
# value: 2
# description: Component has parameters, and supports the extended parameter protocol (PARAM_EXT messages).

 
ais_type = mavutil.mavlink.AIS_TYPE
# name: AIS_TYPE_UNKNOWN
# value: 0
# description: Not available (default).

# name: AIS_TYPE_RESERVED_1
# value: 1
# description: 

# name: AIS_TYPE_RESERVED_2
# value: 2
# description: 

# name: AIS_TYPE_RESERVED_3
# value: 3
# description: 

# name: AIS_TYPE_RESERVED_4
# value: 4
# description: 

# name: AIS_TYPE_RESERVED_5
# value: 5
# description: 

# name: AIS_TYPE_RESERVED_6
# value: 6
# description: 

# name: AIS_TYPE_RESERVED_7
# value: 7
# description: 

# name: AIS_TYPE_RESERVED_8
# value: 8
# description: 

# name: AIS_TYPE_RESERVED_9
# value: 9
# description: 

# name: AIS_TYPE_RESERVED_10
# value: 10
# description: 

# name: AIS_TYPE_RESERVED_11
# value: 11
# description: 

# name: AIS_TYPE_RESERVED_12
# value: 12
# description: 

# name: AIS_TYPE_RESERVED_13
# value: 13
# description: 

# name: AIS_TYPE_RESERVED_14
# value: 14
# description: 

# name: AIS_TYPE_RESERVED_15
# value: 15
# description: 

# name: AIS_TYPE_RESERVED_16
# value: 16
# description: 

# name: AIS_TYPE_RESERVED_17
# value: 17
# description: 

# name: AIS_TYPE_RESERVED_18
# value: 18
# description: 

# name: AIS_TYPE_RESERVED_19
# value: 19
# description: 

# name: AIS_TYPE_WIG
# value: 20
# description: Wing In Ground effect.

# name: AIS_TYPE_WIG_HAZARDOUS_A
# value: 21
# description: 

# name: AIS_TYPE_WIG_HAZARDOUS_B
# value: 22
# description: 

# name: AIS_TYPE_WIG_HAZARDOUS_C
# value: 23
# description: 

# name: AIS_TYPE_WIG_HAZARDOUS_D
# value: 24
# description: 

# name: AIS_TYPE_WIG_RESERVED_1
# value: 25
# description: 

# name: AIS_TYPE_WIG_RESERVED_2
# value: 26
# description: 

# name: AIS_TYPE_WIG_RESERVED_3
# value: 27
# description: 

# name: AIS_TYPE_WIG_RESERVED_4
# value: 28
# description: 

# name: AIS_TYPE_WIG_RESERVED_5
# value: 29
# description: 

# name: AIS_TYPE_FISHING
# value: 30
# description: 

# name: AIS_TYPE_TOWING
# value: 31
# description: 

# name: AIS_TYPE_TOWING_LARGE
# value: 32
# description: Towing: length exceeds 200m or breadth exceeds 25m.

# name: AIS_TYPE_DREDGING
# value: 33
# description: Dredging or other underwater ops.

# name: AIS_TYPE_DIVING
# value: 34
# description: 

# name: AIS_TYPE_MILITARY
# value: 35
# description: 

# name: AIS_TYPE_SAILING
# value: 36
# description: 

# name: AIS_TYPE_PLEASURE
# value: 37
# description: 

# name: AIS_TYPE_RESERVED_20
# value: 38
# description: 

# name: AIS_TYPE_RESERVED_21
# value: 39
# description: 

# name: AIS_TYPE_HSC
# value: 40
# description: High Speed Craft.

# name: AIS_TYPE_HSC_HAZARDOUS_A
# value: 41
# description: 

# name: AIS_TYPE_HSC_HAZARDOUS_B
# value: 42
# description: 

# name: AIS_TYPE_HSC_HAZARDOUS_C
# value: 43
# description: 

# name: AIS_TYPE_HSC_HAZARDOUS_D
# value: 44
# description: 

# name: AIS_TYPE_HSC_RESERVED_1
# value: 45
# description: 

# name: AIS_TYPE_HSC_RESERVED_2
# value: 46
# description: 

# name: AIS_TYPE_HSC_RESERVED_3
# value: 47
# description: 

# name: AIS_TYPE_HSC_RESERVED_4
# value: 48
# description: 

# name: AIS_TYPE_HSC_UNKNOWN
# value: 49
# description: 

# name: AIS_TYPE_PILOT
# value: 50
# description: 

# name: AIS_TYPE_SAR
# value: 51
# description: Search And Rescue vessel.

# name: AIS_TYPE_TUG
# value: 52
# description: 

# name: AIS_TYPE_PORT_TENDER
# value: 53
# description: 

# name: AIS_TYPE_ANTI_POLLUTION
# value: 54
# description: Anti-pollution equipment.

# name: AIS_TYPE_LAW_ENFORCEMENT
# value: 55
# description: 

# name: AIS_TYPE_SPARE_LOCAL_1
# value: 56
# description: 

# name: AIS_TYPE_SPARE_LOCAL_2
# value: 57
# description: 

# name: AIS_TYPE_MEDICAL_TRANSPORT
# value: 58
# description: 

# name: AIS_TYPE_NONECOMBATANT
# value: 59
# description: Noncombatant ship according to RR Resolution No. 18.

# name: AIS_TYPE_PASSENGER
# value: 60
# description: 

# name: AIS_TYPE_PASSENGER_HAZARDOUS_A
# value: 61
# description: 

# name: AIS_TYPE_PASSENGER_HAZARDOUS_B
# value: 62
# description: 

# name: AIS_TYPE_AIS_TYPE_PASSENGER_HAZARDOUS_C
# value: 63
# description: 

# name: AIS_TYPE_PASSENGER_HAZARDOUS_D
# value: 64
# description: 

# name: AIS_TYPE_PASSENGER_RESERVED_1
# value: 65
# description: 

# name: AIS_TYPE_PASSENGER_RESERVED_2
# value: 66
# description: 

# name: AIS_TYPE_PASSENGER_RESERVED_3
# value: 67
# description: 

# name: AIS_TYPE_AIS_TYPE_PASSENGER_RESERVED_4
# value: 68
# description: 

# name: AIS_TYPE_PASSENGER_UNKNOWN
# value: 69
# description: 

# name: AIS_TYPE_CARGO
# value: 70
# description: 

# name: AIS_TYPE_CARGO_HAZARDOUS_A
# value: 71
# description: 

# name: AIS_TYPE_CARGO_HAZARDOUS_B
# value: 72
# description: 

# name: AIS_TYPE_CARGO_HAZARDOUS_C
# value: 73
# description: 

# name: AIS_TYPE_CARGO_HAZARDOUS_D
# value: 74
# description: 

# name: AIS_TYPE_CARGO_RESERVED_1
# value: 75
# description: 

# name: AIS_TYPE_CARGO_RESERVED_2
# value: 76
# description: 

# name: AIS_TYPE_CARGO_RESERVED_3
# value: 77
# description: 

# name: AIS_TYPE_CARGO_RESERVED_4
# value: 78
# description: 

# name: AIS_TYPE_CARGO_UNKNOWN
# value: 79
# description: 

# name: AIS_TYPE_TANKER
# value: 80
# description: 

# name: AIS_TYPE_TANKER_HAZARDOUS_A
# value: 81
# description: 

# name: AIS_TYPE_TANKER_HAZARDOUS_B
# value: 82
# description: 

# name: AIS_TYPE_TANKER_HAZARDOUS_C
# value: 83
# description: 

# name: AIS_TYPE_TANKER_HAZARDOUS_D
# value: 84
# description: 

# name: AIS_TYPE_TANKER_RESERVED_1
# value: 85
# description: 

# name: AIS_TYPE_TANKER_RESERVED_2
# value: 86
# description: 

# name: AIS_TYPE_TANKER_RESERVED_3
# value: 87
# description: 

# name: AIS_TYPE_TANKER_RESERVED_4
# value: 88
# description: 

# name: AIS_TYPE_TANKER_UNKNOWN
# value: 89
# description: 

# name: AIS_TYPE_OTHER
# value: 90
# description: 

# name: AIS_TYPE_OTHER_HAZARDOUS_A
# value: 91
# description: 

# name: AIS_TYPE_OTHER_HAZARDOUS_B
# value: 92
# description: 

# name: AIS_TYPE_OTHER_HAZARDOUS_C
# value: 93
# description: 

# name: AIS_TYPE_OTHER_HAZARDOUS_D
# value: 94
# description: 

# name: AIS_TYPE_OTHER_RESERVED_1
# value: 95
# description: 

# name: AIS_TYPE_OTHER_RESERVED_2
# value: 96
# description: 

# name: AIS_TYPE_OTHER_RESERVED_3
# value: 97
# description: 

# name: AIS_TYPE_OTHER_RESERVED_4
# value: 98
# description: 

# name: AIS_TYPE_OTHER_UNKNOWN
# value: 99
# description: 

 
ais_nav_status = mavutil.mavlink.AIS_NAV_STATUS
# name: UNDER_WAY
# value: 0
# description: Under way using engine.

# name: AIS_NAV_ANCHORED
# value: 1
# description: 

# name: AIS_NAV_UN_COMMANDED
# value: 2
# description: 

# name: AIS_NAV_RESTRICTED_MANOEUVERABILITY
# value: 3
# description: 

# name: AIS_NAV_DRAUGHT_CONSTRAINED
# value: 4
# description: 

# name: AIS_NAV_MOORED
# value: 5
# description: 

# name: AIS_NAV_AGROUND
# value: 6
# description: 

# name: AIS_NAV_FISHING
# value: 7
# description: 

# name: AIS_NAV_SAILING
# value: 8
# description: 

# name: AIS_NAV_RESERVED_HSC
# value: 9
# description: 

# name: AIS_NAV_RESERVED_WIG
# value: 10
# description: 

# name: AIS_NAV_RESERVED_1
# value: 11
# description: 

# name: AIS_NAV_RESERVED_2
# value: 12
# description: 

# name: AIS_NAV_RESERVED_3
# value: 13
# description: 

# name: AIS_NAV_AIS_SART
# value: 14
# description: Search And Rescue Transponder.

# name: AIS_NAV_UNKNOWN
# value: 15
# description: Not available (default).

 
ais_flags = mavutil.mavlink.AIS_FLAGS
# name: AIS_FLAGS_POSITION_ACCURACY
# value: 1
# description: 1 = Position accuracy less than 10m, 0 = position accuracy greater than 10m.

# name: AIS_FLAGS_VALID_COG
# value: 2
# description: 

# name: AIS_FLAGS_VALID_VELOCITY
# value: 4
# description: 

# name: AIS_FLAGS_HIGH_VELOCITY
# value: 8
# description: 1 = Velocity over 52.5765m/s (102.2 knots)

# name: AIS_FLAGS_VALID_TURN_RATE
# value: 16
# description: 

# name: AIS_FLAGS_TURN_RATE_SIGN_ONLY
# value: 32
# description: Only the sign of the returned turn rate value is valid, either greater than 5deg/30s or less than -5deg/30s

# name: AIS_FLAGS_VALID_DIMENSIONS
# value: 64
# description: 

# name: AIS_FLAGS_LARGE_BOW_DIMENSION
# value: 128
# description: Distance to bow is larger than 511m

# name: AIS_FLAGS_LARGE_STERN_DIMENSION
# value: 256
# description: Distance to stern is larger than 511m

# name: AIS_FLAGS_LARGE_PORT_DIMENSION
# value: 512
# description: Distance to port side is larger than 63m

# name: AIS_FLAGS_LARGE_STARBOARD_DIMENSION
# value: 1024
# description: Distance to starboard side is larger than 63m

# name: AIS_FLAGS_VALID_CALLSIGN
# value: 2048
# description: 

# name: AIS_FLAGS_VALID_NAME
# value: 4096
# description: 

 
failure_unit = mavutil.mavlink.FAILURE_UNIT
# name: FAILURE_UNIT_SENSOR_GYRO
# value: 0
# description: 

# name: FAILURE_UNIT_SENSOR_ACCEL
# value: 1
# description: 

# name: FAILURE_UNIT_SENSOR_MAG
# value: 2
# description: 

# name: FAILURE_UNIT_SENSOR_BARO
# value: 3
# description: 

# name: FAILURE_UNIT_SENSOR_GPS
# value: 4
# description: 

# name: FAILURE_UNIT_SENSOR_OPTICAL_FLOW
# value: 5
# description: 

# name: FAILURE_UNIT_SENSOR_VIO
# value: 6
# description: 

# name: FAILURE_UNIT_SENSOR_DISTANCE_SENSOR
# value: 7
# description: 

# name: FAILURE_UNIT_SENSOR_AIRSPEED
# value: 8
# description: 

# name: FAILURE_UNIT_SYSTEM_BATTERY
# value: 100
# description: 

# name: FAILURE_UNIT_SYSTEM_MOTOR
# value: 101
# description: 

# name: FAILURE_UNIT_SYSTEM_SERVO
# value: 102
# description: 

# name: FAILURE_UNIT_SYSTEM_AVOIDANCE
# value: 103
# description: 

# name: FAILURE_UNIT_SYSTEM_RC_SIGNAL
# value: 104
# description: 

# name: FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL
# value: 105
# description: 

 
failure_type = mavutil.mavlink.FAILURE_TYPE
# name: FAILURE_TYPE_OK
# value: 0
# description: No failure injected, used to reset a previous failure.

# name: FAILURE_TYPE_OFF
# value: 1
# description: Sets unit off, so completely non-responsive.

# name: FAILURE_TYPE_STUCK
# value: 2
# description: Unit is stuck e.g. keeps reporting the same value.

# name: FAILURE_TYPE_GARBAGE
# value: 3
# description: Unit is reporting complete garbage.

# name: FAILURE_TYPE_WRONG
# value: 4
# description: Unit is consistently wrong.

# name: FAILURE_TYPE_SLOW
# value: 5
# description: Unit is slow, so e.g. reporting at slower than expected rate.

# name: FAILURE_TYPE_DELAYED
# value: 6
# description: Data of unit is delayed in time.

# name: FAILURE_TYPE_INTERMITTENT
# value: 7
# description: Unit is sometimes working, sometimes not.

 
nav_vtol_land_options = mavutil.mavlink.NAV_VTOL_LAND_OPTIONS
# name: NAV_VTOL_LAND_OPTIONS_DEFAULT
# value: 0
# description: Default autopilot landing behaviour.

# name: NAV_VTOL_LAND_OPTIONS_FW_DESCENT
# value: 1
# description: Descend in fixed wing mode, transitioning to multicopter mode for vertical landing when close to the ground.
          The fixed wing descent pattern is at the discretion of the vehicle (e.g. transition altitude, loiter direction, radius, and speed, etc.).        
        

# name: NAV_VTOL_LAND_OPTIONS_HOVER_DESCENT
# value: 2
# description: Land in multicopter mode on reaching the landing co-ordinates (the whole landing is by "hover descent").

 
mav_winch_status_flag = mavutil.mavlink.MAV_WINCH_STATUS_FLAG
# name: MAV_WINCH_STATUS_HEALTHY
# value: 1
# description: Winch is healthy

# name: MAV_WINCH_STATUS_FULLY_RETRACTED
# value: 2
# description: Winch thread is fully retracted

# name: MAV_WINCH_STATUS_MOVING
# value: 4
# description: Winch motor is moving

# name: MAV_WINCH_STATUS_CLUTCH_ENGAGED
# value: 8
# description: Winch clutch is engaged allowing motor to move freely

 
mag_cal_status = mavutil.mavlink.MAG_CAL_STATUS
# name: MAG_CAL_NOT_STARTED
# value: 0
# description: 

# name: MAG_CAL_WAITING_TO_START
# value: 1
# description: 

# name: MAG_CAL_RUNNING_STEP_ONE
# value: 2
# description: 

# name: MAG_CAL_RUNNING_STEP_TWO
# value: 3
# description: 

# name: MAG_CAL_SUCCESS
# value: 4
# description: 

# name: MAG_CAL_FAILED
# value: 5
# description: 

# name: MAG_CAL_BAD_ORIENTATION
# value: 6
# description: 

# name: MAG_CAL_BAD_RADIUS
# value: 7
# description: 

 
mav_event_error_reason = mavutil.mavlink.MAV_EVENT_ERROR_REASON
# name: MAV_EVENT_ERROR_REASON_UNAVAILABLE
# value: 0
# description: The requested event is not available (anymore).

 
mav_event_current_sequence_flags = mavutil.mavlink.MAV_EVENT_CURRENT_SEQUENCE_FLAGS
# name: MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET
# value: 1
# description: A sequence reset has happened (e.g. vehicle reboot).

 
