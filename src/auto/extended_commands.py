from dronekit import Command
from pymavlink import mavutil

def mav_cmd_nav_waypoint(hold, accept_radius, pass_radius, yaw, latitude, longitude, altitude):
	""" Navigate to waypoint.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
		0, 0,
		hold, # Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
		accept_radius, # Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)
		pass_radius, # 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
		yaw, # Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_nav_loiter_unlim(radius, yaw, latitude, longitude, altitude):
	""" Loiter around this waypoint an unlimited amount of time
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
		0, 0,
		0,
		0,
		radius, # Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise
		yaw, # Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_nav_loiter_turns(turns, heading_required, radius, xtrack_location, latitude, longitude, altitude):
	""" Loiter around this waypoint for X turns
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
		0, 0,
		turns, # Number of turns.
		heading_required, # Leave loiter circle only once heading towards the next waypoint (0 = False)
		radius, # Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise
		xtrack_location, # Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_nav_loiter_time(time, heading_required, radius, xtrack_location, latitude, longitude, altitude):
	""" Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter vehicles stop at the point (within a vehicle-specific acceptance radius). Forward-only moving vehicles (e.g. fixed-wing) circle the point with the specified radius/direction. If the Heading Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once heading towards the next waypoint.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
		0, 0,
		time, # Loiter time (only starts once Lat, Lon and Alt is reached).
		heading_required, # Leave loiter circle only once heading towards the next waypoint (0 = False)
		radius, # Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise.
		xtrack_location, # Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_nav_return_to_launch():
	""" Return to launch location
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_nav_land(abort_alt, land_mode, yaw_angle, latitude, longitude, altitude):
	""" Land at location.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_LAND,
		0, 0,
		abort_alt, # Minimum target altitude if landing is aborted (0 = undefined/use system default).
		land_mode, # Precision land mode.
		0,
		yaw_angle, # Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
		latitude, # Latitude.
		longitude, # Longitude.
		altitude) # Landing altitude (ground level in current frame).

	return cmd

def mav_cmd_nav_takeoff(pitch, yaw, latitude, longitude, altitude):
	""" Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
		0, 0,
		pitch, # Minimum pitch (if airspeed sensor present), desired pitch without sensor
		0,
		0,
		yaw, # Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_nav_land_local(target, offset, descend_rate, yaw, y_position, x_position, z_position):
	""" Land at local position (local frame only)
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_LAND_LOCAL,
		0, 0,
		target, # Landing target number (if available)
		offset, # Maximum accepted offset from desired landing position - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land
		descend_rate, # Landing descend rate
		yaw, # Desired yaw angle
		y_position, # Y-axis position
		x_position, # X-axis position
		z_position) # Z-axis / ground level position

	return cmd

def mav_cmd_nav_takeoff_local(pitch, ascend_rate, yaw, y_position, x_position, z_position):
	""" Takeoff from local position (local frame only)
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL,
		0, 0,
		pitch, # Minimum pitch (if airspeed sensor present), desired pitch without sensor
		0,
		ascend_rate, # Takeoff ascend rate
		yaw, # Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these
		y_position, # Y-axis position
		x_position, # X-axis position
		z_position) # Z-axis position

	return cmd

def mav_cmd_nav_follow(following, ground_speed, radius, yaw, latitude, longitude, altitude):
	""" Vehicle following, i.e. this waypoint represents the position of a moving vehicle
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_FOLLOW,
		0, 0,
		following, # Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation
		ground_speed, # Ground speed of vehicle to be followed
		radius, # Radius around waypoint. If positive loiter clockwise, else counter-clockwise
		yaw, # Desired yaw angle.
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_nav_continue_and_change_alt(action, altitude):
	""" Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,
		0, 0,
		action, # Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.
		0,
		0,
		0,
		0,
		0,
		altitude) # Desired altitude

	return cmd

def mav_cmd_nav_loiter_to_alt(heading_required, radius, xtrack_location, latitude, longitude, altitude):
	""" Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until heading toward the next waypoint.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
		0, 0,
		heading_required, # Leave loiter circle only once heading towards the next waypoint (0 = False)
		radius, # Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
		0,
		xtrack_location, # Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_do_follow(system_id, altitude_mode, altitude, time_to_land):
	""" Begin following a target
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_FOLLOW,
		0, 0,
		system_id, # System ID (of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode.
		0,
		0,
		altitude_mode, # Altitude mode: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home.
		altitude, # Altitude above home. (used if mode=2)
		0,
		time_to_land) # Time to land in which the MAV should go to the default position hold mode after a message RX timeout.

	return cmd

def mav_cmd_do_follow_reposition(camera_q1, camera_q2, camera_q3, camera_q4, altitude_offset, x_offset, y_offset):
	""" Reposition the MAV after a follow target command has been sent
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_FOLLOW_REPOSITION,
		0, 0,
		camera_q1, # Camera q1 (where 0 is on the ray from the camera to the tracking device)
		camera_q2, # Camera q2
		camera_q3, # Camera q3
		camera_q4, # Camera q4
		altitude_offset, # altitude offset from target
		x_offset, # X offset from target
		y_offset) # Y offset from target

	return cmd

def mav_cmd_do_orbit(radius, velocity, yaw_behavior, latitude_x, longitude_y, altitude_z):
	""" Start orbiting on the circumference of a circle defined by the parameters. Setting any value NaN results in using defaults.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_ORBIT,
		0, 0,
		radius, # Radius of the circle. positive: Orbit clockwise. negative: Orbit counter-clockwise.
		velocity, # Tangential Velocity. NaN: Vehicle configuration default.
		yaw_behavior, # Yaw behavior of the vehicle.
		0,
		latitude_x, # Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.
		longitude_y, # Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.
		altitude_z) # Center point altitude (MSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.

	return cmd

def mav_cmd_nav_roi(roi_mode, wp_index, roi_index, x, y, z):
	""" Deprecated since 2018-01 and replaced by MAV_CMD_DO_SET_ROI_*
		Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_ROI,
		0, 0,
		roi_mode, # Region of interest mode.
		wp_index, # Waypoint index/ target ID. (see MAV_ROI enum)
		roi_index, # ROI index (allows a vehicle to manage multiple ROI's)
		0,
		x, # x the location of the fixed ROI (see MAV_FRAME)
		y, # y
		z) # z

	return cmd

def mav_cmd_nav_pathplanning(local_ctrl, global_ctrl, yaw, latitude_x, longitude_y, altitude_z):
	""" Control autonomous path planning on the MAV.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_PATHPLANNING,
		0, 0,
		local_ctrl, # 0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning
		global_ctrl, # 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid
		0,
		yaw, # Yaw angle at goal
		latitude_x, # Latitude/X of goal
		longitude_y, # Longitude/Y of goal
		altitude_z) # Altitude/Z of goal

	return cmd

def mav_cmd_nav_spline_waypoint(hold, latitude_x, longitude_y, altitude_z):
	""" Navigate to waypoint using a spline path.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
		0, 0,
		hold, # Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
		0,
		0,
		0,
		latitude_x, # Latitude/X of goal
		longitude_y, # Longitude/Y of goal
		altitude_z) # Altitude/Z of goal

	return cmd

def mav_cmd_nav_vtol_takeoff(transition_heading, yaw_angle, latitude, longitude, altitude):
	""" Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
		0, 0,
		0,
		transition_heading, # Front transition heading.
		0,
		yaw_angle, # Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_nav_vtol_land(approach_altitude, yaw, latitude, longitude, ground_altitude):
	""" Land using VTOL mode
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND,
		0, 0,
		0,
		0,
		approach_altitude, # Approach altitude (with the same reference as the Altitude field). NaN if unspecified.
		yaw, # Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
		latitude, # Latitude
		longitude, # Longitude
		ground_altitude) # Altitude (ground level)

	return cmd

def mav_cmd_nav_guided_enable(enable):
	""" hand control over to an external controller
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE,
		0, 0,
		enable, # On / Off (> 0.5f on)
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_nav_delay(delay, hour, minute, second):
	""" Delay the next navigation command a number of seconds or until a specified time
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_DELAY,
		0, 0,
		delay, # Delay (-1 to enable time-of-day fields)
		hour, # hour (24h format, UTC, -1 to ignore)
		minute, # minute (24h format, UTC, -1 to ignore)
		second, # second (24h format, UTC, -1 to ignore)
		0,
		0,
		0)

	return cmd

def mav_cmd_nav_payload_place(max_descent, latitude, longitude, altitude):
	""" Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_PAYLOAD_PLACE,
		0, 0,
		max_descent, # Maximum distance to descend.
		0,
		0,
		0,
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_nav_last():
	""" NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_LAST,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_condition_delay(delay):
	""" Delay mission state machine.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_CONDITION_DELAY,
		0, 0,
		delay, # Delay
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_condition_change_alt(rate, altitude):
	""" Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude reached.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
		0, 0,
		rate, # Descent / Ascend rate.
		0,
		0,
		0,
		0,
		0,
		altitude) # Target Altitude

	return cmd

def mav_cmd_condition_distance(distance):
	""" Delay mission state machine until within desired distance of next NAV point.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_CONDITION_DISTANCE,
		0, 0,
		distance, # Distance.
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_condition_yaw(angle, angular_speed, direction, relative):
	""" Reach a certain target angle.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_CONDITION_YAW,
		0, 0,
		angle, # target angle, 0 is north
		angular_speed, # angular speed
		direction, # direction: -1: counter clockwise, 1: clockwise
		relative, # 0: absolute angle, 1: relative offset
		0,
		0,
		0)

	return cmd

def mav_cmd_condition_last():
	""" NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_CONDITION_LAST,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_set_mode(mode, custom_mode, custom_submode):
	""" Set system mode.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_MODE,
		0, 0,
		mode, # Mode
		custom_mode, # Custom mode - this is system specific, please refer to the individual autopilot specifications for details.
		custom_submode, # Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_jump(number, repeat):
	""" Jump to the desired command in the mission list.  Repeat this action only the specified number of times
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_JUMP,
		0, 0,
		number, # Sequence number
		repeat, # Repeat count
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_change_speed(speed_type, speed, throttle, relative):
	""" Change speed and/or throttle set points.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
		0, 0,
		speed_type, # Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
		speed, # Speed (-1 indicates no change)
		throttle, # Throttle (-1 indicates no change)
		relative, # 0: absolute, 1: relative
		0,
		0,
		0)

	return cmd

def mav_cmd_do_set_home(use_current, yaw, latitude, longitude, altitude):
	""" Changes the home location either to the current location or a specified location.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_HOME,
		0, 0,
		use_current, # Use current (1=use current location, 0=use specified location)
		0,
		0,
		yaw, # Yaw angle. NaN to use default heading
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_do_set_parameter(number, value):
	""" Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER,
		0, 0,
		number, # Parameter number
		value, # Parameter value
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_set_relay(instance, setting):
	""" Set a relay to a condition.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
		0, 0,
		instance, # Relay instance number.
		setting, # Setting. (1=on, 0=off, others possible depending on system hardware)
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_repeat_relay(instance, count, time):
	""" Cycle a relay on and off for a desired number of cycles with a desired period.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_REPEAT_RELAY,
		0, 0,
		instance, # Relay instance number.
		count, # Cycle count.
		time, # Cycle time.
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_set_servo(instance, pwm):
	""" Set a servo to a desired PWM value.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
		0, 0,
		instance, # Servo instance number.
		pwm, # Pulse Width Modulation.
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_repeat_servo(instance, pwm, count, time):
	""" Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO,
		0, 0,
		instance, # Servo instance number.
		pwm, # Pulse Width Modulation.
		count, # Cycle count.
		time, # Cycle time.
		0,
		0,
		0)

	return cmd

def mav_cmd_do_flighttermination(terminate):
	""" Terminate flight immediately
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
		0, 0,
		terminate, # Flight termination activated if > 0.5
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_change_altitude(altitude, frame):
	""" Change altitude set point.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
		0, 0,
		altitude, # Altitude.
		frame, # Frame of new altitude.
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_set_actuator(actuator_1, actuator_2, actuator_3, actuator_4, actuator_5, actuator_6, index):
	""" Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_ACTUATOR,
		0, 0,
		actuator_1, # Actuator 1 value, scaled from [-1 to 1]. NaN to ignore.
		actuator_2, # Actuator 2 value, scaled from [-1 to 1]. NaN to ignore.
		actuator_3, # Actuator 3 value, scaled from [-1 to 1]. NaN to ignore.
		actuator_4, # Actuator 4 value, scaled from [-1 to 1]. NaN to ignore.
		actuator_5, # Actuator 5 value, scaled from [-1 to 1]. NaN to ignore.
		actuator_6, # Actuator 6 value, scaled from [-1 to 1]. NaN to ignore.
		index) # Index of actuator set (i.e if set to 1, Actuator 1 becomes Actuator 7)

	return cmd

def mav_cmd_do_land_start(latitude, longitude):
	""" Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_LAND_START,
		0, 0,
		0,
		0,
		0,
		0,
		latitude, # Latitude
		longitude, # Longitude
		0)

	return cmd

def mav_cmd_do_rally_land(altitude, speed):
	""" Mission command to perform a landing from a rally point.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_RALLY_LAND,
		0, 0,
		altitude, # Break altitude
		speed, # Landing speed
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_go_around(altitude):
	""" Mission command to safely abort an autonomous landing.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_GO_AROUND,
		0, 0,
		altitude, # Altitude
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_reposition(speed, bitmask, yaw, latitude, longitude, altitude):
	""" Reposition the vehicle to a specific WGS84 global position.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_REPOSITION,
		0, 0,
		speed, # Ground speed, less than 0 (-1) for default
		bitmask, # Bitmask of option flags.
		0,
		yaw, # Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise)
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_do_pause_continue(continue):
	""" If in a GPS controlled position mode, hold the current position or continue.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
		0, 0,
		continue, # 0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_set_reverse(reverse):
	""" Set moving direction to forward or reverse.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_REVERSE,
		0, 0,
		reverse, # Direction (0=Forward, 1=Reverse)
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_set_roi_location(gimbal_device_id, latitude, longitude, altitude):
	""" Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this message.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
		0, 0,
		gimbal_device_id, # Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
		0,
		0,
		0,
		latitude, # Latitude of ROI location
		longitude, # Longitude of ROI location
		altitude) # Altitude of ROI location

	return cmd

def mav_cmd_do_set_roi_wpnext_offset(gimbal_device_id, pitch_offset, roll_offset, yaw_offset):
	""" Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET,
		0, 0,
		gimbal_device_id, # Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
		0,
		0,
		0,
		pitch_offset, # Pitch offset from next waypoint, positive pitching up
		roll_offset, # Roll offset from next waypoint, positive rolling to the right
		yaw_offset) # Yaw offset from next waypoint, positive yawing to the right

	return cmd

def mav_cmd_do_set_roi_none(gimbal_device_id):
	""" Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. After this command the gimbal manager should go back to manual input if available, and otherwise assume a neutral position.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE,
		0, 0,
		gimbal_device_id, # Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_set_roi_sysid(system_id, gimbal_device_id):
	""" Mount tracks system with specified system ID. Determination of target vehicle position may be done with GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_ROI_SYSID,
		0, 0,
		system_id, # System ID
		gimbal_device_id, # Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_control_video(id, transmission, interval, recording):
	""" Control onboard camera system.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_CONTROL_VIDEO,
		0, 0,
		id, # Camera ID (-1 for all)
		transmission, # Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
		interval, # Transmission mode: 0: video stream, >0: single images every n seconds
		recording, # Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
		0,
		0,
		0)

	return cmd

def mav_cmd_do_set_roi(roi_mode, wp_index, roi_index):
	""" Deprecated since 2018-01 and replaced by MAV_CMD_DO_SET_ROI_*
		Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_ROI,
		0, 0,
		roi_mode, # Region of interest mode.
		wp_index, # Waypoint index/ target ID (depends on param 1).
		roi_index, # Region of interest index. (allows a vehicle to manage multiple ROI's)
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_digicam_configure(mode, shutter_speed, aperture, iso, exposure, command_identity, engine_cutoff):
	""" Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE,
		0, 0,
		mode, # Modes: P, TV, AV, M, Etc.
		shutter_speed, # Shutter speed: Divisor number for one second.
		aperture, # Aperture: F stop number.
		iso, # ISO number e.g. 80, 100, 200, Etc.
		exposure, # Exposure type enumerator.
		command_identity, # Command Identity.
		engine_cutoff) # Main engine cut-off time before camera trigger. (0 means no cut-off)

	return cmd

def mav_cmd_do_digicam_control(session_control, zoom_absolute, zoom_relative, focus, shoot_command, command_identity, shot_id):
	""" Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
		0, 0,
		session_control, # Session control e.g. show/hide lens
		zoom_absolute, # Zoom's absolute position
		zoom_relative, # Zooming step value to offset zoom from the current position
		focus, # Focus Locking, Unlocking or Re-locking
		shoot_command, # Shooting Command
		command_identity, # Command Identity
		shot_id) # Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.

	return cmd

def mav_cmd_do_mount_configure(mode, stabilize_roll, stabilize_pitch, stabilize_yaw, roll_input_mode, pitch_input_mode, yaw_input_mode):
	""" Deprecated since 2020-01 and replaced by MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
		Mission command to configure a camera or antenna mount
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
		0, 0,
		mode, # Mount operation mode
		stabilize_roll, # stabilize roll? (1 = yes, 0 = no)
		stabilize_pitch, # stabilize pitch? (1 = yes, 0 = no)
		stabilize_yaw, # stabilize yaw? (1 = yes, 0 = no)
		roll_input_mode, # roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
		pitch_input_mode, # pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
		yaw_input_mode) # yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)

	return cmd

def mav_cmd_do_mount_control(pitch, roll, yaw, altitude, latitude, longitude, mode):
	""" Deprecated since 2020-01 and replaced by MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
		Mission command to control a camera or antenna mount
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
		0, 0,
		pitch, # pitch depending on mount mode (degrees or degrees/second depending on pitch input).
		roll, # roll depending on mount mode (degrees or degrees/second depending on roll input).
		yaw, # yaw depending on mount mode (degrees or degrees/second depending on yaw input).
		altitude, # altitude depending on mount mode.
		latitude, # latitude, set if appropriate mount mode.
		longitude, # longitude, set if appropriate mount mode.
		mode) # Mount mode.

	return cmd

def mav_cmd_do_set_cam_trigg_dist(distance, shutter, trigger):
	""" Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_CAM_TRIGG_DIST,
		0, 0,
		distance, # Camera trigger distance. 0 to stop triggering.
		shutter, # Camera shutter integration time. -1 or 0 to ignore
		trigger, # Trigger camera once immediately. (0 = no trigger, 1 = trigger)
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_fence_enable(enable):
	""" Mission command to enable the geofence
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
		0, 0,
		enable, # enable? (0=disable, 1=enable, 2=disable_floor_only)
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_parachute(action):
	""" Mission item/command to release a parachute or enable/disable auto release.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
		0, 0,
		action, # Action
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_motor_test(instance, throttle_type, throttle, timeout, motor_count, test_order):
	""" Command to perform motor test.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
		0, 0,
		instance, # Motor instance number (from 1 to max number of motors on the vehicle).
		throttle_type, # Throttle type (whether the Throttle Value in param3 is a percentage, PWM value, etc.)
		throttle, # Throttle value.
		timeout, # Timeout between tests that are run in sequence.
		motor_count, # Motor count. Number of motors to test in sequence: 0/1=one motor, 2= two motors, etc. The Timeout (param4) is used between tests.
		test_order, # Motor test order.
		0)

	return cmd

def mav_cmd_do_inverted_flight(inverted):
	""" Change to/from inverted flight.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_INVERTED_FLIGHT,
		0, 0,
		inverted, # Inverted flight. (0=normal, 1=inverted)
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_gripper(instance, action):
	""" Mission command to operate a gripper.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_GRIPPER,
		0, 0,
		instance, # Gripper instance number.
		action, # Gripper action to perform.
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_autotune_enable(enable):
	""" Enable/disable autotune.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_AUTOTUNE_ENABLE,
		0, 0,
		enable, # Enable (1: enable, 0:disable).
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_nav_set_yaw_speed(yaw, speed, angle):
	""" Sets a desired vehicle turn angle and speed change.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_SET_YAW_SPEED,
		0, 0,
		yaw, # Yaw angle to adjust steering by.
		speed, # Speed.
		angle, # Final angle. (0=absolute, 1=relative)
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_set_cam_trigg_interval(trigger_cycle, shutter_integration):
	""" Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL,
		0, 0,
		trigger_cycle, # Camera trigger cycle time. -1 or 0 to ignore.
		shutter_integration, # Camera shutter integration time. Should be less than trigger cycle time. -1 or 0 to ignore.
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_mount_control_quat(q1, q2, q3, q4):
	""" Deprecated since 2020-01 and replaced by MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
		Mission command to control a camera or antenna mount, using a quaternion as reference.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL_QUAT,
		0, 0,
		q1, # quaternion param q1, w (1 in null-rotation)
		q2, # quaternion param q2, x (0 in null-rotation)
		q3, # quaternion param q3, y (0 in null-rotation)
		q4, # quaternion param q4, z (0 in null-rotation)
		0,
		0,
		0)

	return cmd

def mav_cmd_do_guided_master(system_id, component_id):
	""" set id of master controller
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_GUIDED_MASTER,
		0, 0,
		system_id, # System ID
		component_id, # Component ID
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_guided_limits(timeout, min_altitude, max_altitude, horizontal_move_limit):
	""" Set limits for external control
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_GUIDED_LIMITS,
		0, 0,
		timeout, # Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no timeout.
		min_altitude, # Altitude (MSL) min - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit.
		max_altitude, # Altitude (MSL) max - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit.
		horizontal_move_limit, # Horizontal move limit - if vehicle moves more than this distance from its location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal move limit.
		0,
		0,
		0)

	return cmd

def mav_cmd_do_engine_control(start_engine, cold_start, height_delay):
	""" Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL,
		0, 0,
		start_engine, # 0: Stop engine, 1:Start Engine
		cold_start, # 0: Warm start, 1:Cold start. Controls use of choke where applicable
		height_delay, # Height delay. This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_set_mission_current(number):
	""" Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
		0, 0,
		number, # Mission sequence value to set
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_last():
	""" NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_LAST,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_preflight_calibration(gyro_temperature, magnetometer, ground_pressure, remote_control, accelerometer, compmot_or_airspeed, esc_or_baro):
	""" Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
		0, 0,
		gyro_temperature, # 1: gyro calibration, 3: gyro temperature calibration
		magnetometer, # 1: magnetometer calibration
		ground_pressure, # 1: ground pressure calibration
		remote_control, # 1: radio RC calibration, 2: RC trim calibration
		accelerometer, # 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
		compmot_or_airspeed, # 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration
		esc_or_baro) # 1: ESC calibration, 3: barometer temperature calibration

	return cmd

def mav_cmd_preflight_set_sensor_offsets(sensor_type, x_offset, y_offset, z_offset, fourth_dimension, fifth_dimension, sixth_dimension):
	""" Set sensor offsets. This command will be only accepted if in pre-flight mode.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS,
		0, 0,
		sensor_type, # Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer
		x_offset, # X axis offset (or generic dimension 1), in the sensor's raw units
		y_offset, # Y axis offset (or generic dimension 2), in the sensor's raw units
		z_offset, # Z axis offset (or generic dimension 3), in the sensor's raw units
		fourth_dimension, # Generic dimension 4, in the sensor's raw units
		fifth_dimension, # Generic dimension 5, in the sensor's raw units
		sixth_dimension) # Generic dimension 6, in the sensor's raw units

	return cmd

def mav_cmd_preflight_uavcan(actuator_id):
	""" Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial vehicle configuration (it is not a normal pre-flight command and has been poorly named).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_PREFLIGHT_UAVCAN,
		0, 0,
		actuator_id, # 1: Trigger actuator ID assignment and direction mapping. 0: Cancel command.
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_preflight_storage(parameter_storage, mission_storage, logging_rate):
	""" Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
		0, 0,
		parameter_storage, # Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults, 3: Reset sensor calibration parameter data to factory default (or firmware default if not available)
		mission_storage, # Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults
		logging_rate, # Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: logging rate (e.g. set to 1000 for 1000 Hz logging)
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_preflight_reboot_shutdown(autopilot, companion):
	""" Request the reboot or shutdown of system components.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
		0, 0,
		autopilot, # 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.
		companion, # 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_upgrade(component_id, reboot):
	""" Request a target system to start an upgrade of one (or all) of its components. For example, the command might be sent to a companion computer to cause it to upgrade a connected flight controller. The system doing the upgrade will report progress using the normal command protocol sequence for a long running operation. Command protocol information: https://mavlink.io/en/services/command.html.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_UPGRADE,
		0, 0,
		component_id, # Component id of the component to be upgraded. If set to 0, all components should be upgraded.
		reboot, # 0: Do not reboot component after the action is executed, 1: Reboot component after the action is executed.
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_override_goto(continue, position, frame, yaw, latitude_x, longitude_y, altitude_z):
	""" Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO,
		0, 0,
		continue, # MAV_GOTO_DO_HOLD: pause mission and either hold or move to specified position (depending on param2), MAV_GOTO_DO_CONTINUE: resume mission.
		position, # MAV_GOTO_HOLD_AT_CURRENT_POSITION: hold at current position, MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position.
		frame, # Coordinate frame of hold point.
		yaw, # Desired yaw angle.
		latitude_x, # Latitude/X position.
		longitude_y, # Longitude/Y position.
		altitude_z) # Altitude/Z position.

	return cmd

def mav_cmd_oblique_survey(distance, shutter, min_interval, positions, roll_angle, pitch_angle):
	""" Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera setup (providing an increased HFOV). This command can also be used to set the shutter integration time for the camera.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_OBLIQUE_SURVEY,
		0, 0,
		distance, # Camera trigger distance. 0 to stop triggering.
		shutter, # Camera shutter integration time. 0 to ignore
		min_interval, # The minimum interval in which the camera is capable of taking subsequent pictures repeatedly. 0 to ignore.
		positions, # Total number of roll positions at which the camera will capture photos (images captures spread evenly across the limits defined by param5).
		roll_angle, # Angle limits that the camera can be rolled to left and right of center.
		pitch_angle, # Fixed pitch angle that the camera will hold in oblique mode if the mount is actuated in the pitch axis.
		0)

	return cmd

def mav_cmd_mission_start(first_item, last_item):
	""" start running a mission
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_MISSION_START,
		0, 0,
		first_item, # first_item: the first mission item to run
		last_item, # last_item:  the last mission item to run (after this item is run, the mission ends)
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_component_arm_disarm(arm, force):
	""" Arms / Disarms a component
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
		0, 0,
		arm, # 0: disarm, 1: arm
		force, # 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_illuminator_on_off(enable):
	""" Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas external to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_ILLUMINATOR_ON_OFF,
		0, 0,
		enable, # 0: Illuminators OFF, 1: Illuminators ON
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_get_home_position():
	""" Request the home position from the vehicle.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_inject_failure(failure_unit, failure_type, instance):
	""" Inject artificial failure for testing purposes. Note that autopilots should implement an additional protection before accepting this command such as a specific param setting.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_INJECT_FAILURE,
		0, 0,
		failure_unit, # The unit which is affected by the failure.
		failure_type, # The type how the failure manifests itself.
		instance, # Instance affected by failure (0 to signal all).
		0,
		0,
		0)

	return cmd

def mav_cmd_start_rx_pair(spektrum, rc_type):
	""" Starts receiver pairing.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_START_RX_PAIR,
		0, 0,
		spektrum, # 0:Spektrum.
		rc_type, # RC type.
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_get_message_interval(message_id):
	""" Request the interval between messages for a particular MAVLink message ID. The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
		0, 0,
		message_id, # The MAVLink message ID
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_set_message_interval(message_id, interval, response_target):
	""" Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
		0, 0,
		message_id, # The MAVLink message ID
		interval, # The interval between two messages. Set to -1 to disable and 0 to request default rate.
		response_target, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
		0,
		0,
		0)

	return cmd

def mav_cmd_request_message(message_id, req_param_1, req_param_2, req_param_3, req_param_4, req_param_5, response_target):
	""" Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
		0, 0,
		message_id, # The MAVLink message ID of the requested message.
		req_param_1, # Use for index ID, if required. Otherwise, the use of this parameter (if any) must be defined in the requested message. By default assumed not used (0).
		req_param_2, # The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
		req_param_3, # The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
		req_param_4, # The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
		req_param_5, # The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
		response_target) # Target address for requested message (if message has target address fields). 0: Flight-stack default, 1: address of requestor, 2: broadcast.

	return cmd

def mav_cmd_request_protocol_version(protocol):
	""" Deprecated since 2019-08 and replaced by MAV_CMD_REQUEST_MESSAGE
		Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their capabilities in an PROTOCOL_VERSION message
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_REQUEST_PROTOCOL_VERSION,
		0, 0,
		protocol, # 1: Request supported protocol versions by all nodes on the network
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_request_autopilot_capabilities(version):
	""" Deprecated since 2019-08 and replaced by MAV_CMD_REQUEST_MESSAGE
		Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
		0, 0,
		version, # 1: Request autopilot version
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_request_camera_information(capabilities):
	""" Deprecated since 2019-08 and replaced by MAV_CMD_REQUEST_MESSAGE
		Request camera information (CAMERA_INFORMATION).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_INFORMATION,
		0, 0,
		capabilities, # 0: No action 1: Request camera capabilities
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_request_camera_settings(settings):
	""" Deprecated since 2019-08 and replaced by MAV_CMD_REQUEST_MESSAGE
		Request camera settings (CAMERA_SETTINGS).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_SETTINGS,
		0, 0,
		settings, # 0: No Action 1: Request camera settings
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_request_storage_information(storage_id, information):
	""" Deprecated since 2019-08 and replaced by MAV_CMD_REQUEST_MESSAGE
		Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_REQUEST_STORAGE_INFORMATION,
		0, 0,
		storage_id, # Storage ID (0 for all, 1 for first, 2 for second, etc.)
		information, # 0: No Action 1: Request storage information
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_storage_format(storage_id, format, reset_image_log):
	""" Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_STORAGE_FORMAT,
		0, 0,
		storage_id, # Storage ID (1 for first, 2 for second, etc.)
		format, # Format storage (and reset image log). 0: No action 1: Format storage
		reset_image_log, # Reset Image Log (without formatting storage medium). This will reset CAMERA_CAPTURE_STATUS.image_count and CAMERA_IMAGE_CAPTURED.image_index. 0: No action 1: Reset Image Log
		0,
		0,
		0)

	return cmd

def mav_cmd_request_camera_capture_status(capture_status):
	""" Deprecated since 2019-08 and replaced by MAV_CMD_REQUEST_MESSAGE
		Request camera capture status (CAMERA_CAPTURE_STATUS)
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS,
		0, 0,
		capture_status, # 0: No Action 1: Request camera capture status
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_request_flight_information(flight_information):
	""" Deprecated since 2019-08 and replaced by MAV_CMD_REQUEST_MESSAGE
		Request flight information (FLIGHT_INFORMATION)
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_REQUEST_FLIGHT_INFORMATION,
		0, 0,
		flight_information, # 1: Request flight information
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_reset_camera_settings(reset):
	""" Reset all camera settings to Factory Default
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_RESET_CAMERA_SETTINGS,
		0, 0,
		reset, # 0: No Action 1: Reset all settings
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_set_camera_mode(camera_mode):
	""" Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_SET_CAMERA_MODE,
		0, 0,
		0,
		camera_mode, # Camera mode
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_set_camera_zoom(zoom_type, zoom_value):
	""" Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM,
		0, 0,
		zoom_type, # Zoom type
		zoom_value, # Zoom value. The range of valid values depend on the zoom type.
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_set_camera_focus(focus_type, focus_value):
	""" Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS,
		0, 0,
		focus_type, # Focus type
		focus_value, # Focus value
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_jump_tag(tag):
	""" Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_JUMP_TAG,
		0, 0,
		tag, # Tag.
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_jump_tag(tag, repeat):
	""" Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
		0, 0,
		tag, # Target tag to jump to.
		repeat, # Repeat count.
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_param_transaction(action, transport, transaction_id):
	""" Request to start or end a parameter transaction. Multiple kinds of transport layers can be used to exchange parameters in the transaction (param, param_ext and mavftp). The command response can either be a success/failure or an in progress in case the receiving side takes some time to apply the parameters.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_PARAM_TRANSACTION,
		0, 0,
		action, # Action to be performed (start, commit, cancel, etc.)
		transport, # Possible transport layers to set and get parameters via mavlink during a parameter transaction.
		transaction_id, # Identifier for a specific transaction.
		0,
		0,
		0)

	return cmd

def mav_cmd_do_gimbal_manager_pitchyaw(pitch_angle, yaw_angle, pitch_rate, yaw_rate, gimbal_manager_flags, gimbal_device_id):
	""" High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. Note: a gimbal is never to react to this command but only the gimbal manager.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
		0, 0,
		pitch_angle, # Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode).
		yaw_angle, # Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode).
		pitch_rate, # Pitch rate (positive to pitch up).
		yaw_rate, # Yaw rate (positive to yaw to the right).
		gimbal_manager_flags, # Gimbal manager flags to use.
		gimbal_device_id, # Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
		0)

	return cmd

def mav_cmd_do_gimbal_manager_configure(sysid_primary_control, compid_primary_control, sysid_secondary_control, compid_secondary_control, gimbal_device_id):
	""" Gimbal configuration to set which sysid/compid is in primary and secondary control.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
		0, 0,
		sysid_primary_control, # Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
		compid_primary_control, # Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
		sysid_secondary_control, # Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
		compid_secondary_control, # Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
		gimbal_device_id, # Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
		0)

	return cmd

def mav_cmd_image_start_capture(interval, total_images, sequence_number):
	""" Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
		0, 0,
		0,
		interval, # Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware (typically greater than 2 seconds).
		total_images, # Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.
		sequence_number, # Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1), otherwise set to 0. Increment the capture ID for each capture command to prevent double captures when a command is re-transmitted.
		0,
		0,
		0)

	return cmd

def mav_cmd_image_stop_capture():
	""" Stop image capture sequence Use NaN for reserved values.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_request_camera_image_capture(number):
	""" Deprecated since 2019-08 and replaced by MAV_CMD_REQUEST_MESSAGE
		Re-request a CAMERA_IMAGE_CAPTURED message.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE,
		0, 0,
		number, # Sequence number for missing CAMERA_IMAGE_CAPTURED message
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_do_trigger_control(enable, reset, pause):
	""" Enable or disable on-board camera triggering system.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_TRIGGER_CONTROL,
		0, 0,
		enable, # Trigger enable/disable (0 for disable, 1 for start), -1 to ignore
		reset, # 1 to reset the trigger sequence, -1 or 0 to ignore
		pause, # 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore
		0,
		0,
		0)

	return cmd

def mav_cmd_camera_track_point(point_x, point_y, radius):
	""" If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT,
		0, 0,
		point_x, # Point to track x value (normalized 0..1, 0 is left, 1 is right).
		point_y, # Point to track y value (normalized 0..1, 0 is top, 1 is bottom).
		radius, # Point radius (normalized 0..1, 0 is image left, 1 is image right).
		0,
		0,
		0)

	return cmd

def mav_cmd_camera_track_rectangle(top_left_corner_x, top_left_corner_y, bottom_right_corner_x, bottom_right_corner_y):
	""" If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE,
		0, 0,
		top_left_corner_x, # Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
		top_left_corner_y, # Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
		bottom_right_corner_x, # Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
		bottom_right_corner_y, # Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
		0,
		0)

	return cmd

def mav_cmd_camera_stop_tracking():
	""" Stops ongoing tracking.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_CAMERA_STOP_TRACKING,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_video_start_capture(stream_id, status_frequency):
	""" Starts video capture (recording).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE,
		0, 0,
		stream_id, # Video Stream ID (0 for all streams)
		status_frequency, # Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency)
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_video_stop_capture(stream_id):
	""" Stop the current video capture (recording).
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE,
		0, 0,
		stream_id, # Video Stream ID (0 for all streams)
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_video_start_streaming(stream_id):
	""" Start video streaming
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING,
		0, 0,
		stream_id, # Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_video_stop_streaming(stream_id):
	""" Stop the given video stream
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING,
		0, 0,
		stream_id, # Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_request_video_stream_information(stream_id):
	""" Deprecated since 2019-08 and replaced by MAV_CMD_REQUEST_MESSAGE
		Request video stream information (VIDEO_STREAM_INFORMATION)
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION,
		0, 0,
		stream_id, # Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_request_video_stream_status(stream_id):
	""" Deprecated since 2019-08 and replaced by MAV_CMD_REQUEST_MESSAGE
		Request video stream status (VIDEO_STREAM_STATUS)
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS,
		0, 0,
		stream_id, # Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_logging_start(format):
	""" Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_LOGGING_START,
		0, 0,
		format, # Format: 0: ULog
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_logging_stop():
	""" Request to stop streaming log data over MAVLink
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_LOGGING_STOP,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_airframe_configuration(landing_gear_id, landing_gear_position):
	""" 
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION,
		0, 0,
		landing_gear_id, # Landing gear ID (default: 0, -1 for all)
		landing_gear_position, # Landing gear position (Down: 0, Up: 1, NaN for no change)
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_control_high_latency(enable):
	""" Request to start/stop transmitting over the high latency telemetry
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_CONTROL_HIGH_LATENCY,
		0, 0,
		enable, # Control transmission over high latency telemetry (0: stop, 1: start)
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_panorama_create(horizontal_angle, vertical_angle, horizontal_speed, vertical_speed):
	""" Create a panorama at the current position
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_PANORAMA_CREATE,
		0, 0,
		horizontal_angle, # Viewing angle horizontal of the panorama (+- 0.5 the total angle)
		vertical_angle, # Viewing angle vertical of panorama.
		horizontal_speed, # Speed of the horizontal rotation.
		vertical_speed, # Speed of the vertical rotation.
		0,
		0)

	return cmd

def mav_cmd_do_vtol_transition(state, immediate):
	""" Request VTOL transition
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
		0, 0,
		state, # The target VTOL state. For normal transitions, only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.
		immediate, # Force immediate transition to the specified MAV_VTOL_STATE. 1: Force immediate, 0: normal transition. Can be used, for example, to trigger an emergency "Quadchute". Caution: Can be dangerous/damage vehicle, depending on autopilot implementation of this command.
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_arm_authorization_request(system_id):
	""" Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
        
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_ARM_AUTHORIZATION_REQUEST,
		0, 0,
		system_id, # Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_set_guided_submode_standard():
	""" This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.
                  
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_set_guided_submode_circle(radius, latitude, longitude):
	""" This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
                  
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE,
		0, 0,
		radius, # Radius of desired circle in CIRCLE_MODE
		0,
		0,
		0,
		latitude, # Target latitude of center of circle in CIRCLE_MODE
		longitude, # Target longitude of center of circle in CIRCLE_MODE
		0)

	return cmd

def mav_cmd_condition_gate(geometry, usealtitude, latitude, longitude, altitude):
	""" Delay mission state machine until gate has been reached.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_CONDITION_GATE,
		0, 0,
		geometry, # Geometry: 0: orthogonal to path between previous and next waypoint.
		usealtitude, # Altitude: 0: ignore altitude
		0,
		0,
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_nav_fence_return_point(latitude, longitude, altitude):
	""" Fence return point (there can only be one such point in a geofence definition). If rally points are supported they should be used instead.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
		0, 0,
		0,
		0,
		0,
		0,
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_nav_fence_polygon_vertex_inclusion(vertex_count, inclusion_group, latitude, longitude):
	""" Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
        
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
		0, 0,
		vertex_count, # Polygon vertex count
		inclusion_group, # Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group, must be the same for all points in each polygon
		0,
		0,
		latitude, # Latitude
		longitude, # Longitude
		0)

	return cmd

def mav_cmd_nav_fence_polygon_vertex_exclusion(vertex_count, latitude, longitude):
	""" Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
        
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
		0, 0,
		vertex_count, # Polygon vertex count
		0,
		0,
		0,
		latitude, # Latitude
		longitude, # Longitude
		0)

	return cmd

def mav_cmd_nav_fence_circle_inclusion(radius, inclusion_group, latitude, longitude):
	""" Circular fence area. The vehicle must stay inside this area.
        
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
		0, 0,
		radius, # Radius.
		inclusion_group, # Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group
		0,
		0,
		latitude, # Latitude
		longitude, # Longitude
		0)

	return cmd

def mav_cmd_nav_fence_circle_exclusion(radius, latitude, longitude):
	""" Circular fence area. The vehicle must stay outside this area.
        
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
		0, 0,
		radius, # Radius.
		0,
		0,
		0,
		latitude, # Latitude
		longitude, # Longitude
		0)

	return cmd

def mav_cmd_nav_rally_point(latitude, longitude, altitude):
	""" Rally point. You can have multiple rally points defined.
        
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
		0, 0,
		0,
		0,
		0,
		0,
		latitude, # Latitude
		longitude, # Longitude
		altitude) # Altitude

	return cmd

def mav_cmd_uavcan_get_node_info():
	""" Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_UAVCAN_GET_NODE_INFO,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_payload_prepare_deploy(operation_mode, approach_vector, ground_speed, altitude_clearance, latitude, longitude, altitude):
	""" Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_PAYLOAD_PREPARE_DEPLOY,
		0, 0,
		operation_mode, # Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.
		approach_vector, # Desired approach vector in compass heading. A negative value indicates the system can define the approach vector at will.
		ground_speed, # Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.
		altitude_clearance, # Minimum altitude clearance to the release position. A negative value indicates the system can define the clearance at will.
		latitude, # Latitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)
		longitude, # Longitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)
		altitude) # Altitude (MSL)

	return cmd

def mav_cmd_payload_control_deploy(operation_mode):
	""" Control the payload deployment.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_PAYLOAD_CONTROL_DEPLOY,
		0, 0,
		operation_mode, # Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_fixed_mag_cal_yaw(yaw, compassmask, latitude, longitude):
	""" Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_FIXED_MAG_CAL_YAW,
		0, 0,
		yaw, # Yaw of vehicle in earth frame.
		compassmask, # CompassMask, 0 for all.
		latitude, # Latitude.
		longitude, # Longitude.
		0,
		0,
		0)

	return cmd

def mav_cmd_do_winch(instance, action, length, rate):
	""" Command to operate winch.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_DO_WINCH,
		0, 0,
		instance, # Winch instance number.
		action, # Action to perform.
		length, # Length of cable to release (negative to wind).
		rate, # Release rate (negative to wind).
		0,
		0,
		0)

	return cmd

def mav_cmd_waypoint_user_1(latitude, longitude, altitude):
	""" User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_WAYPOINT_USER_1,
		0, 0,
		0,
		0,
		0,
		0,
		latitude, # Latitude unscaled
		longitude, # Longitude unscaled
		altitude) # Altitude (MSL)

	return cmd

def mav_cmd_waypoint_user_2(latitude, longitude, altitude):
	""" User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_WAYPOINT_USER_2,
		0, 0,
		0,
		0,
		0,
		0,
		latitude, # Latitude unscaled
		longitude, # Longitude unscaled
		altitude) # Altitude (MSL)

	return cmd

def mav_cmd_waypoint_user_3(latitude, longitude, altitude):
	""" User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_WAYPOINT_USER_3,
		0, 0,
		0,
		0,
		0,
		0,
		latitude, # Latitude unscaled
		longitude, # Longitude unscaled
		altitude) # Altitude (MSL)

	return cmd

def mav_cmd_waypoint_user_4(latitude, longitude, altitude):
	""" User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_WAYPOINT_USER_4,
		0, 0,
		0,
		0,
		0,
		0,
		latitude, # Latitude unscaled
		longitude, # Longitude unscaled
		altitude) # Altitude (MSL)

	return cmd

def mav_cmd_waypoint_user_5(latitude, longitude, altitude):
	""" User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_WAYPOINT_USER_5,
		0, 0,
		0,
		0,
		0,
		0,
		latitude, # Latitude unscaled
		longitude, # Longitude unscaled
		altitude) # Altitude (MSL)

	return cmd

def mav_cmd_spatial_user_1(latitude, longitude, altitude):
	""" User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_SPATIAL_USER_1,
		0, 0,
		0,
		0,
		0,
		0,
		latitude, # Latitude unscaled
		longitude, # Longitude unscaled
		altitude) # Altitude (MSL)

	return cmd

def mav_cmd_spatial_user_2(latitude, longitude, altitude):
	""" User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_SPATIAL_USER_2,
		0, 0,
		0,
		0,
		0,
		0,
		latitude, # Latitude unscaled
		longitude, # Longitude unscaled
		altitude) # Altitude (MSL)

	return cmd

def mav_cmd_spatial_user_3(latitude, longitude, altitude):
	""" User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_SPATIAL_USER_3,
		0, 0,
		0,
		0,
		0,
		0,
		latitude, # Latitude unscaled
		longitude, # Longitude unscaled
		altitude) # Altitude (MSL)

	return cmd

def mav_cmd_spatial_user_4(latitude, longitude, altitude):
	""" User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_SPATIAL_USER_4,
		0, 0,
		0,
		0,
		0,
		0,
		latitude, # Latitude unscaled
		longitude, # Longitude unscaled
		altitude) # Altitude (MSL)

	return cmd

def mav_cmd_spatial_user_5(latitude, longitude, altitude):
	""" User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_SPATIAL_USER_5,
		0, 0,
		0,
		0,
		0,
		0,
		latitude, # Latitude unscaled
		longitude, # Longitude unscaled
		altitude) # Altitude (MSL)

	return cmd

def mav_cmd_user_1():
	""" User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_USER_1,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_user_2():
	""" User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_USER_2,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_user_3():
	""" User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_USER_3,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_user_4():
	""" User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_USER_4,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

def mav_cmd_user_5():
	""" User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
	"""

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		mavutil.mavlink.MAV_CMD_USER_5,
		0, 0,
		0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd

