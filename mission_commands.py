from dronekit import Command
from pymavlink import mavutil

def mav_cmd_nav_waypoint(hold, accept_radius, pass_radius, yaw, latitude, longitude, altitude):
    """ Navigate to waypoint. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0,
        hold,
        accept_radius,
        pass_radius,
        yaw,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_nav_loiter_unlim(radius, yaw, latitude, longitude, altitude):
    """ Loiter around this waypoint an unlimited amount of time """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
        0, 0,
        0, 0,
        radius,
        yaw,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_nav_loiter_turns(turns, heading_required, radius, xtrack_location, latitude, longitude, altitude):
    """ Loiter around this waypoint for X turns """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
        0, 0,
        turns,
        heading_required,
        radius,
        xtrack_location,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_nav_loiter_time(time, heading_required, radius, xtrack_location, latitude, longitude, altitude):
    """ Loiter at the specified latitude, longitude and altitude for a certain amount of time. 
        Multicopter vehicles stop at the point (within a vehicle-specific acceptance radius). 
        Forward-only moving vehicles (e.g. fixed-wing) circle the point with the specified radius_direction.
        If the Heading Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once heading towards the next waypoint.
    """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
        0, 0,
        time,
        heading_required,
        radius,
        xtrack_location,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_nav_return_to_launch():
    """ Return to launch location """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_nav_land(abort_alt, land_mode, yaw_angle, latitude, longitude, altitude):
    """ Land at location. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0,
        abort_alt,
        land_mode,
        0,
        yaw_angle,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_nav_takeoff(pitch, yaw, latitude, longitude, altitude):
    """ Takeoff from ground _ hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0,
        pitch,
        0, 0,
        yaw,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_nav_land_local(target, offset, descend_rate, yaw, y_position, x_position, z_position):
    """ Land at local position (local frame only) """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAND_LOCAL,
        0, 0,
        target,
        offset,
        descend_rate,
        yaw,
        y_position,
        x_position,
        z_position)

    return cmd

def mav_cmd_nav_takeoff_local(pitch, ascend_rate, yaw, y_position, x_position, z_position):
    """ Takeoff from local position (local frame only) """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL,
        0, 0,
        pitch,
        0,
        ascend_rate,
        yaw,
        y_position,
        x_position,
        z_position)

    return cmd

def mav_cmd_nav_follow(following, ground_speed, radius, yaw, latitude, longitude, altitude):
    """ Vehicle following, i.e. this waypoint represents the position of a moving vehicle """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_FOLLOW,
        0, 0,
        following,
        ground_speed,
        radius,
        yaw,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_nav_continue_and_change_alt(action, altitude):
    """ Continue on the current course and climb_descend to specified altitude. 
    When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,
        0, 0,
        action,
        0, 0, 0, 0, 0,
        altitude)

    return cmd

def mav_cmd_nav_loiter_to_alt(heading_required, radius, xtrack_location, latitude, longitude, altitude):
    """ Begin loiter at the specified Latitude and Longitude. 
    If Lat=Lon=0, then loiter at the current position. Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until heading toward the next waypoint. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
        0, 0,
        heading_required,
        radius,
        0,
        xtrack_location,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_do_follow(system_id, altitude_mode, altitude, time_to_land):
    """ Begin following a target """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_FOLLOW,
        0, 0,
        system_id,
        0, 0,
        altitude_mode,
        altitude,
        0,
        time_to_land)

    return cmd

def mav_cmd_do_follow_reposition(camera_q1, camera_q2, camera_q3, camera_q4, altitude_offset, x_offset, y_offset):
    """ Reposition the MAV after a follow target command has been sent """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_FOLLOW_REPOSITION,
        0, 0,
        camera_q1,
        camera_q2,
        camera_q3,
        camera_q4,
        altitude_offset,
        x_offset,
        y_offset)

    return cmd

    
def mav_cmd_do_winch(instance, action, length, rate):
    """ Command to operate winch. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_WINCH,
        0, 0,
        instance,
        action,
        length,
        rate,
        0, 0, 0)

    return cmd

def mav_cmd_waypoint_user_2(latitude, longitude, altitude):
    """ User defined waypoint item. Ground Station will show the Vehicle as flying through this item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_WAYPOINT_USER_2,
        0, 0, 0, 0, 0, 0,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_waypoint_user_3(latitude, longitude, altitude):
    """ User defined waypoint item. Ground Station will show the Vehicle as flying through this item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_WAYPOINT_USER_3,
        0, 0, 0, 0, 0, 0,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_waypoint_user_4(latitude, longitude, altitude):
    """ User defined waypoint item. Ground Station will show the Vehicle as flying through this item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_WAYPOINT_USER_4,
        0, 0, 0, 0, 0, 0,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_waypoint_user_5(latitude, longitude, altitude):
    """ User defined waypoint item. Ground Station will show the Vehicle as flying through this item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_WAYPOINT_USER_5,
        0, 0, 0, 0, 0, 0,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_spatial_user_1(latitude, longitude, altitude):
    """ User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_SPATIAL_USER_1,
        0, 0, 0, 0, 0, 0,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_spatial_user_2(latitude, longitude, altitude):
    """ User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_SPATIAL_USER_2,
        0, 0, 0, 0, 0, 0,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_spatial_user_3(latitude, longitude, altitude):
    """ User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_SPATIAL_USER_3,
        0, 0, 0, 0, 0, 0,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_spatial_user_4(latitude, longitude, altitude):
    """ User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_SPATIAL_USER_4,
        0, 0, 0, 0, 0, 0,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_spatial_user_5(latitude, longitude, altitude):
    """ User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_SPATIAL_USER_5,
        0, 0, 0, 0, 0, 0,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_user_1():
    """ User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_USER_1,
        0, 0, 0, 0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_user_2():
    """ User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_USER_2,
        0, 0, 0, 0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_user_3():
    """ User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_USER_3,
        0, 0, 0, 0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_user_4():
    """ User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_USER_4,
        0, 0, 0, 0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_user_5():
    """ User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_USER_5,
        0, 0, 0, 0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_nav_pathplanning(local_ctrl, global_ctrl, yaw, latitude_x, longitude_y, altitude_z):
    """ Control autonomous path planning on the MAV. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_PATHPLANNING,
        0, 0,
        local_ctrl,
        global_ctrl,
        0,
        yaw,
        latitude_x,
        longitude_y,
        altitude_z)

    return cmd

def mav_cmd_nav_spline_waypoint(hold, latitude_x, longitude_y, altitude_z):
    """ Navigate to waypoint using a spline path. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
        0, 0,
        hold,
        0,
        0,
        0,
        latitude_x,
        longitude_y,
        altitude_z)

    return cmd

def mav_cmd_nav_vtol_takeoff(transition_heading, yaw_angle, latitude, longitude, altitude):
    """ Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.). """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
        0, 0,
        0,
        transition_heading,
        0,
        yaw_angle,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_nav_vtol_land(approach_altitude, yaw, latitude, longitude, ground_altitude):
    """ Land using VTOL mode """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND,
        0, 0,
        0, 0,
        approach_altitude,
        yaw,
        latitude,
        longitude,
        ground_altitude)

    return cmd

def mav_cmd_nav_delay(delay, hour, minute, second):
    """ Delay the next navigation command a number of seconds or until a specified time """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_DELAY,
        0, 0,
        delay,
        hour,
        minute,
        second,
        0, 0, 0)

    return cmd

def mav_cmd_nav_payload_place(max_descent, latitude, longitude, altitude):
    """ Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_PAYLOAD_PLACE,
        0, 0,
        max_descent,
        0, 0, 0,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_nav_last():
    """ NOP - This command is only used to mark the upper limit of the NAV_ACTION commands in the enumeration """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LAST,
        0, 0, 0, 0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_condition_delay(delay):
    """ Delay mission state machine. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_CONDITION_DELAY,
        0, 0,
        delay,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_condition_change_alt(rate, altitude):
    """ Ascend_descend to target altitude at specified rate. Delay mission state machine until desired altitude reached. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
        0, 0,
        rate,
        0, 0, 0, 0, 0,
        altitude)

    return cmd

def mav_cmd_condition_distance(distance):
    """ Delay mission state machine until within desired distance of next NAV point. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_CONDITION_DISTANCE,
        0, 0,
        distance,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_condition_yaw(angle, angular_speed, direction, relative):
    """ Reach a certain target angle. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, 0,
        angle,
        angular_speed,
        direction,
        relative,
        0, 0, 0)

    return cmd

def mav_cmd_condition_last():
    """ NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_CONDITION_LAST,
        0, 0, 0, 0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_set_mode(mode, custom_mode, custom_submode):
    """ Set system mode. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, 0,
        mode,
        custom_mode,
        custom_submode,
        0, 0, 0, 0)

    return cmd

def mav_cmd_do_jump(number, repeat):
    """ Jump to the desired command in the mission list. Repeat this action only the specified number of times """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_JUMP,
        0, 0,
        number,
        repeat,
        0,
        0, 0, 0, 0)

    return cmd

def mav_cmd_do_change_speed(speed_type, speed, throttle, relative):
    """ Change speed and_or throttle set points. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0, 0,
        speed_type,
        speed,
        throttle,
        relative,
        0, 0, 0)

    return cmd

def mav_cmd_do_set_home(use_current, yaw, latitude, longitude, altitude):
    """ Changes the home location either to the current location or a specified location. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0, 0,
        use_current,
        0, 0,
        yaw,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_do_set_parameter(number, value):
    """ Set a system parameter. Caution! Use of this command requires knowledge of the numeric enumeration value of the parameter. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER,
        0, 0,
        number,
        value,
        0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_set_relay(instance, setting):
    """ Set a relay to a condition. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
        0, 0,
        instance,
        setting,
        0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_repeat_relay(instance, count, time):
    """ Cycle a relay on and off for a desired number of cycles with a desired period. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_REPEAT_RELAY,
        0, 0,
        instance,
        count,
        time,
        0, 0, 0, 0)

    return cmd

def mav_cmd_do_set_servo(instance, pwm):
    """ Set a servo to a desired PWM value. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, 0,
        instance,
        pwm,
        0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_repeat_servo(instance, pwm, count, time):
    """ Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO,
        0, 0,
        instance,
        pwm,
        count,
        time,
        0, 0, 0)

    return cmd

def mav_cmd_do_flighttermination(terminate):
    """ Terminate flight immediately """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
        0, 0,
        terminate,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_change_altitude(altitude, frame):
    """ Change altitude set point. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
        0, 0,
        altitude,
        frame,
        0, 0, 0, 0, 0)

    return cmd


def mav_cmd_do_land_start(latitude, longitude):
    """ Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude_Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_LAND_START,
        0, 0, 0, 0, 0, 0,
        latitude,
        longitude,
        0)

    return cmd

def mav_cmd_do_rally_land(altitude, speed):
    """ Mission command to perform a landing from a rally point. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_RALLY_LAND,
        0, 0,
        altitude,
        speed,
        0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_go_around(altitude):
    """ Mission command to safely abort an autonomous landing. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_GO_AROUND,
        0, 0,
        altitude,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_reposition(speed, bitmask, yaw, latitude, longitude, altitude):
    """ Reposition the vehicle to a specific WGS84 global position. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        0, 0,
        speed,
        bitmask,
        0,
        yaw,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_do_pause_continue(continue):
    """ If in a GPS controlled position mode, hold the current position or continue. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
        0, 0,
        continue,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_set_reverse(reverse):
    """ Set moving direction to forward or reverse. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_REVERSE,
        0, 0,
        reverse,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_set_roi_location(gimbal_device_id, latitude, longitude, altitude):
    """ Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this message. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
        0, 0,
        gimbal_device_id,
        0, 0, 0,
        latitude,
        longitude,
        altitude)

    return cmd

def mav_cmd_do_set_roi_wpnext_offset(gimbal_device_id, pitch_offset, roll_offset, yaw_offset):
    """ Sets the region of interest (ROI) to be toward next waypoint, with optional pitch_roll_yaw offset. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET,
        0, 0,
        gimbal_device_id,
        0, 0, 0,
        pitch_offset,
        roll_offset,
        yaw_offset)

    return cmd

def mav_cmd_do_set_roi_none(gimbal_device_id):
    """ Cancels any previous ROI command returning the vehicle_sensors to default flight characteristics. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. After this command the gimbal manager should go back to manual input if available, and otherwise assume a neutral position. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE,
        0, 0,
        gimbal_device_id,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_digicam_control(session_control, zoom_absolute, zoom_relative, focus, shoot_command, command_identity, shot_id):
    """ Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https:__mavlink.io_en_services_camera_def.html ). """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
        0, 0,
        session_control,
        zoom_absolute,
        zoom_relative,
        focus,
        shoot_command,
        command_identity,
        shot_id)

    return cmd

def mav_cmd_do_set_cam_trigg_dist(distance, shutter, trigger):
    """ Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_CAM_TRIGG_DIST,
        0, 0,
        distance,
        shutter,
        trigger,
        0, 0, 0, 0)

    return cmd

def mav_cmd_do_fence_enable(enable):
    """ Mission command to enable the geofence """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
        0, 0,
        enable,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_parachute(action):
    """ Mission item_command to release a parachute or enable_disable auto release. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
        0, 0,
        action,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_motor_test(instance, throttle_type, throttle, timeout, motor_count, test_order):
    """ Command to perform motor test. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0, 0,
        instance,
        throttle_type,
        throttle,
        timeout,
        motor_count,
        test_order,
        0)

    return cmd

def mav_cmd_do_inverted_flight(inverted):
    """ Change to_from inverted flight. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_INVERTED_FLIGHT,
        0, 0,
        inverted,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_gripper(instance, action):
    """ Mission command to operate a gripper. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_GRIPPER,
        0, 0,
        instance,
        action,
        0,
        0, 0, 0, 0)

    return cmd

def mav_cmd_do_autotune_enable(enable):
    """ Enable_disable autotune. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_AUTOTUNE_ENABLE,
        0, 0,
        enable,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_nav_set_yaw_speed(yaw, speed, angle):
    """ Sets a desired vehicle turn angle and speed change. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_SET_YAW_SPEED,
        0, 0,
        yaw,
        speed,
        angle,
        0, 0, 0, 0)

    return cmd

def mav_cmd_do_set_cam_trigg_interval(trigger_cycle, shutter_integration):
    """ Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL,
        0, 0,
        trigger_cycle,
        shutter_integration,
        0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_guided_master(system_id, component_id):
    """ set id of master controller """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_GUIDED_MASTER,
        0, 0,
        system_id,
        component_id,
        0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_guided_limits(timeout, min_altitude, max_altitude, horizon_move_limit):
    """ Set limits for external control """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_GUIDED_LIMITS,
        0, 0,
        timeout,
        min_altitude,
        max_altitude,
        horizon_move_limit,
        0, 0, 0)

    return cmd

def mav_cmd_do_engine_control(start_engine, cold_start, height_delay):
    """ Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL,
        0, 0,
        start_engine,
        cold_start,
        height_delay,
        0, 0, 0, 0)

    return cmd

def mav_cmd_preflight_calibration(gyro_temperature, magnetometer, ground_pressure, remote_control, accelerometer, compmot_or_airspeed, esc_or_baro):
    """ Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0, 0,
        gyro_temperature,
        magnetometer,
        ground_pressure,
        remote_control,
        accelerometer,
        compmot_or_airspeed,
        esc_or_baro)

    return cmd

def mav_cmd_preflight_set_sensor_offsets(sensor_type, x_offset, y_offset, z_offset, fourth_dimension, fifth_dimension, sixth_dimension):
    """ Set sensor offsets. This command will be only accepted if in pre-flight mode. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS,
        0, 0,
        sensor_type,
        x_offset,
        y_offset,
        z_offset,
        fourth_dimension,
        fifth_dimension,
        sixth_dimension)

    return cmd

def mav_cmd_preflight_uavcan(actuator_id):
    """ Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial vehicle configuration (it is not a normal pre-flight command and has been poorly named). """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_UAVCAN,
        0, 0,
        actuator_id,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_preflight_storage(parameter_storage, mission_storage, logging_rate):
    """ Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
        0, 0,
        parameter_storage,
        mission_storage,
        logging_rate,
        0, 0, 0, 0)

    return cmd

def mav_cmd_preflight_reboot_shutdown(autopilot, companion):
    """ Request the reboot or shutdown of system components. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 0,
        autopilot,
        companion,
        0, 0, 0, 0, 0)

    return cmd

def mav_cmd_override_goto(continue, position, frame, yaw, latitude_x, longitude_y, altitude_z):
    """ Override current mission with command to pause mission, pause mission and move to position, continue_resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO,
        0, 0,
        continue,
        position,
        frame,
        yaw,
        latitude_x,
        longitude_y,
        altitude_z)

    return cmd
    
def mav_cmd_mission_start(first_item, last_item, arm, force):
    """ start running a mission """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0,
        first_item,
        last_item,
        arm,
        force)

    return cmd

def mav_cmd_logging_start(format):
    """ Request to start streaming logging data over MAVLink (see also LOGGING_DATA message) """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_LOGGING_START,
        0, 0,
        format,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_logging_stop():
    """ Request to stop streaming log data over MAVLink """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_LOGGING_STOP,
        0, 0, 0, 0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_control_high_latency(enable):
    """ Request to start_stop transmitting over the high latency telemetry """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_CONTROL_HIGH_LATENCY,
        0, 0,
        enable,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_panorama_create(horizontal_angle, vertical_angle, horizontal_speed, vertical_speed):
    """ Create a panorama at the current position """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_PANORAMA_CREATE,
        0, 0,
        horizontal_angle,
        vertical_angle,
        horizontal_speed,
        vertical_speed)

    return cmd

def mav_cmd_nav_fence_return_point(latitude, longitude, altitude):
    """ Fence return point (there can only be one such point in a geofence definition). If rally points are supported they should be used instead. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
        0, 0, 0, 0, 0, 0,
        latitude,
        longitude,
        altitude)

    return cmd
    
def mav_cmd_payload_control_deploy(operation_mode):
    """ Control the payload deployment. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_PAYLOAD_CONTROL_DEPLOY,
        0, 0,
        operation_mode,
        0, 0, 0, 0, 0, 0)

    return cmd

def mav_cmd_do_control_video(id, transmission, interval, recording):
    """ Control onboard camera system. """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_CONTROL_VIDEO,
        0, 0,
        id,
        transmission,
        interval,
        recording,
        0, 0, 0)

def mav_cmd_do_set_mission_current(number):
    """ Set the mission item with sequence number seq as current item. 
        This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between). 
    """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
        0, 0,
        number,
        0, 0, 0, 0, 0, 0)
        
    return cmd


def mav_cmd_component_arm_disarm(arm, force):
    """ Arms _ Disarms a component """
    cmd = Command(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0,
        arm,
        force,
        0, 0, 0, 0, 0)
        
    return cmd

# def MAV_CMD_DO_SET_ROI_SYSID(system_id, gimbal_device_id    id, transmission,):
#     """ Mount tracks system with specified system ID. 
#         Determination of target vehicle position may be done with GLOBAL_POSITION_INT or any other means. 
#         This command can be sent to a gimbal manager but not to a gimbal device. 
#         A gimbal device is not to react to this message.
#     """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         system_id,
#         gimbal_device_id)id,
#         transmission,

#     return cmd

    # """ Start orbiting on the circumference of a circle defined by the parameters. Setting any value NaN results in using defaults. """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     radius,
    #     velocity,
    #     yaw_behavior,
    #     0,
    #     latitude_x,
    #     longitude_y,
    #     altitude_z)

    # return cmd

    # """ Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     roi_mode,
    #     wp_index,
    #     roi_index,
    #     0,
    #     x,
    #     y,
    #     z)

    # return cmd
    
    # "
    # """ hand control over to an external controller """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE,
    #     0, 0,
    #     enable,
    #     0,
    #     0,
    #     0,
    #     0,
    #     0,
    #     0)

    # return cmd
    # "
    # """ Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter). """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     actuator_1,
    #     actuator_2,
    #     actuator_3,
    #     actuator_4,
    #     actuator_5,
    #     actuator_6,
    #     index)

    # return cmd


    # return cmd

    # """ Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     roi_mode,
    #     wp_index,
    #     roi_index,
    #     0, 0, 0, 0)

    # return cmd

    # """ Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https:__mavlink.io_en_services_camera_def.html ). """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE,
    #     0, 0,
    #     mode,
    #     shutter_speed,
    #     aperture,
    #     iso,
    #     exposure,
    #     command_identity,
    #     engine_cut-off)

    # return cmd


    # """ Mission command to configure a camera or antenna mount """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     mode,
    #     stabilize_roll,
    #     stabilize_pitch,
    #     stabilize_yaw,
    #     roll_input_mode,
    #     pitch_input_mode,
    #     yaw_input_mode)

    # return cmd
    # "
    # """ Mission command to control a camera or antenna mount """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     pitch,
    #     roll,
    #     yaw,
    #     altitude,
    #     latitude,
    #     longitude,
    #     mode)

    # return cmd


    # "
    # """ Mission command to control a camera or antenna mount, using a quaternion as reference. """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     q1,
    #     q2,
    #     q3,
    #     q4,
    #     0,
    #     0,
    #     0)

    # return cmd

            # "def mav_cmd_do_last():
    # """ NOP - This command is only used to mark the upper limit of the DO commands in the enumeration """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.MAV_CMD_DO_LAST,
    #     0, 0, 0, 0, 0, 0, 0, 0, 0)

    # return cmd


    # "
    # """ Request a target system to start an upgrade of one (or all) of its components. For example, the command might be sent to a companion computer to cause it to upgrade a connected flight controller. The system doing the upgrade will report progress using the normal command protocol sequence for a long running operation. Command protocol information: https:__mavlink.io_en_services_command.html. """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     component_id,
    #     reboot,
    #     0,
    #     0, 0, 0, 0)

    # return cmd

    # "
    # """ Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera setup (providing an increased HFOV). This command can also be used to set the shutter integration time for the camera. """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     distance,
    #     shutter,
    #     min_interval,
    #     positions,
    #     roll_angle,
    #     pitch_angle,
    #     0)

    return cmd







#     "
#     """ Turns illuminators ON_OFF. An illuminator is a light source that is used for lighting up dark areas external to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light). """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         enable)0,
#         0,
#         0,
        

#     return cmd



#     "def mav_cmd_get_home_position():
#     """ Request the home position from the vehicle. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
#         0, 0, 0, 0, 0, 0, 0, 0, 0)

#     return cmd











#     "
#     """ Inject artificial failure for testing purposes. Note that autopilots should implement an additional protection before accepting this command such as a specific param setting. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         failure_unit,
#         failure_type,
#         instance)spektrum,
        

#     return cmd





#     "def mav_cmd_start_rx_pair(spektrum, rc_type    message_id ):
#     """ Starts receiver pairing. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_START_RX_PAIR,
#         0, 0,
#         spektrum,
#         rc_type)message_id)

#     return cmd




#     "def mav_cmd_get_message_interval(message_id    message_id, interval, response_target):
#     """ Request the interval between messages for a particular MAVLink message ID. The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
#         0, 0,
#         message_id)message_id,
#         interval,
#         response_target)

#     return cmd



#     "def mav_cmd_set_message_interval(message_id, interval, response_target    message_id,):
#     """ Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
#         0, 0,
#         message_id,
#         interval,
#         response_target)message_id,
        

#     return cmd





#     "def mav_cmd_request_message(message_id, req_param_1, req_param_2, req_param_3, req_param_4, req_param_5, response_target):
#     """ Request the target system(s) emit a single instance of a specified message (i.e. a ""one-shot"" version of MAV_CMD_SET_MESSAGE_INTERVAL). """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
#         0, 0,
#         message_id,
#         req_param_1,
#         req_param_2,
#         req_param_3,
#         req_param_4,
#         req_param_5,
#         response_target)

#     return cmd










#     "
#     """ Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their capabilities in an PROTOCOL_VERSION message """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         protocol,
#         0)version,
        

#     return cmd





#     "
#     """ Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         version,
#         0)capabilities,
        

#     return cmd





#     "
#     """ Request camera information (CAMERA_INFORMATION). """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         capabilities,
#         0)settings,
        

#     return cmd





#     "
#     """ Request camera settings (CAMERA_SETTINGS). """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         settings,
#         0)storage_id,
        

#     return cmd





#     "
#     """ Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         storage_id,
#         information,
#         0)storage_id,
        

#     return cmd





#     "def mav_cmd_storage_format(storage_id, format, reset_image_log, 0   ):
#     """ Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_STORAGE_FORMAT,
#         0, 0,
#         storage_id,
#         format,
#         reset_image_log,
#         0)

#     return cmd







#     "
#     """ Request camera capture status (CAMERA_CAPTURE_STATUS) """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         capture_status,
#         0)flight_information,
        

#     return cmd


#     "
#     """ Request flight information (FLIGHT_INFORMATION) """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         flight_information,
#         0)reset,
#         0)

#     return cmd




#     "def mav_cmd_reset_camera_settings(reset, 0    0, camera_mode,):
#     """ Reset all camera settings to Factory Default """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_RESET_CAMERA_SETTINGS,
#         0, 0,
#         reset,
#         0)0,
#         camera_mode,
        

#     return cmd

#     "def mav_cmd_set_camera_mode(0, camera_mode, d=, d=, d=  ):
#     """ Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_SET_CAMERA_MODE,
#         0, 0,
#         0,
#         camera_mode,
#         d=,
#         d=,
#         d=)

#     return cmd

#     "
#     """ Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success). """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         zoom_type,
#         zoom_value,
#         d=,
#         d=,
#         d=)

#     return cmd

#     "
#     """ Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success). """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         focus_type,
#         focus_value,
#         d=,
#         d=,
#         d=)

#     return cmd


#     "def mav_cmd_jump_tag(tag    tag, repeat ):
#     """ Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_JUMP_TAG,
#         0, 0,
#         tag)tag,
#         repeat)

#     return cmd

#     "def mav_cmd_do_jump_tag(tag, repeat     ):
#     """ Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
#         0, 0,
#         tag,
#         repeat)

#     return cmd

#     "
#     """ Request to start or end a parameter transaction. Multiple kinds of transport layers can be used to exchange parameters in the transaction (param, param_ext and mavftp). The command response can either be a success_failure or an in progress in case the receiving side takes some time to apply the parameters. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         action,
#         transport,
#         transaction_id)

#     return cmd

#     "
#     """ High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. Note: a gimbal is never to react to this command but only the gimbal manager. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         pitch_angle,
#         yaw_angle,
#         pitch_rate,
#         yaw_rate,
#         gimbal_manager_flags,
#         gimbal_device_id)

#     return cmd

#     "
#     """ Gimbal configuration to set which sysid_compid is in primary and secondary control. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         sysid_primary_control,
#         compid_primary_control,
#         sysid_secondary_control,
#         compid_secondary_control,
#         gimbal_device_id)

#     return cmd

#     "def mav_cmd_image_start_capture(0, interval, total_images, sequence_number, d=, d=, d=):
#     """ Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
#         0, 0,
#         0,
#         interval,
#         total_images,
#         sequence_number,
#         d=,
#         d=,
#         d=)

#     return cmd

# def mav_cmd_image_stop_capture(0, d=, d=, d=, d=  ):
#     """ Stop image capture sequence Use NaN for reserved values. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE,
#         0, 0,
#         0,
#         d=,
#         d=,
#         d=,
#         d=)

#     return cmd

#     "
#     """ Re-request a CAMERA_IMAGE_CAPTURED message. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         number,
#         d=,
#         d=,
#         d=,
#         d=)

#     return cmd

#     "def mav_cmd_do_trigger_control(enable, reset, pause    ):
#     """ Enable or disable on-board camera triggering system. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_DO_TRIGGER_CONTROL,
#         0, 0,
#         enable,
#         reset,
#         pause)

#     return cmd
#     "
#     """ If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         point_x,
#         point_y,
#         radius)

#     return cmd

#     "
#     """ If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking. """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.,
#         0, 0,
#         top_left_corner_x,
#         top_left_corner_y,
#         bottom_right_corner_x,
#         bottom_right_corner_y)

#     return cmd

#     "def mav_cmd_video_start_capture(stream_id, status_frequency, d=, d=, d=, d=, d=):
#     """ Starts video capture (recording). """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE,
#         0, 0,
#         stream_id,
#         status_frequency,
#         d=,
#         d=,
#         d=,
#         d=,
#         d=)

#     return cmd

# def mav_cmd_video_stop_capture(stream_id, d=, d=, d=, d=, d=, d=):
#     """ Stop the current video capture (recording). """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE,
#         0, 0,
#         stream_id,
#         d=,
#         d=,
#         d=,
#         d=,
#         d=,
#         d=)

#     return cmd

# def mav_cmd_video_start_streaming(stream_id    stream_id  ):
#     """ Start video streaming """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING,
#         0, 0,
#         stream_id)stream_id)

#     return cmd


# def mav_cmd_video_stop_streaming(stream_id ):
#     """ Stop the given video stream """
#     cmd = Command(
#         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING,
#         0, 0,
#         stream_id)

#     return cmd

    # "
    # """ Request video stream information (VIDEO_STREAM_INFORMATION) """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     stream_id)stream_id)

    # return cmd

    # "
    # """ Request video stream status (VIDEO_STREAM_STATUS) """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     stream_id)format,
    #     0,
    #     0,
        

    # return cmd

    # "cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION,
    #     0, 0,
    #     landing_gear_id,
    #     landing_gear_position,
    #     d=,
    #     d=,
    #     d=,
    #     d=,
    #     d=)

    # return cmd

    # "def mav_cmd_do_vtol_transition(state, immediate     system_id):
    # """ Request VTOL transition """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
    #     0, 0,
    #     state,
    #     immediate)system_id)

    # return cmd

    # "cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     system_id)

    # return cmd

    # "cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     radius,
    #     0,
    #     0,
    #     0,
    #     latitude,
    #     longitude)

    # return cmd

    # "
    # """ Delay mission state machine until gate has been reached. """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     geometry,
    #     usealtitude,
    #     0, 0,
    #     latitude,
    #     longitude,
    #     altitude)

    # return cmd

    # "cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     vertex_count,
    #     inclusion_group,
    #     0,
    #     0,
    #     latitude,
    #     longitude,
    #     0)

    # return cmd

    # "cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     vertex_count,
    #     0,
    #     0,
    #     0,
    #     latitude,
    #     longitude,
    #     0)

    # return cmd

    # "cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     radius,
    #     inclusion_group,
    #     0,
    #     0,
    #     latitude,
    #     longitude,
    #     0)

    # return cmd

    # "cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     radius,
    #     0,
    #     0,
    #     0,
    #     latitude,
    #     longitude,
    #     0)

    # return cmd


    # "cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.,
    #     0, 0,
    #     0,
    #     0,
    #     0,
    #     0,
    #     latitude,
    #     longitude,
    #     altitude)

    # return cmd

    # "
    # """ Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages. """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.MAV_CMD_UAVCAN_GET_NODE_INFO,
    #     0, 0, 0, 0, 0, 0, 0, 0, 0)

    # return cmd

    # "
    # """ Deploy payload on a Lat _ Lon _ Alt position. This includes the navigation to reach the required release position and velocity. """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.MAV_CMD_PAYLOAD_PREPARE_DEPLOY,
    #     0, 0,
    #     operation_mode,
    #     approach_vector,
    #     ground_speed,
    #     altitude_clearance,
    #     latitude,
    #     longitude,
    #     altitude)

    # return cmd


    # "
    # """ Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location. """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.MAV_CMD_FIXED_MAG_CAL_YAW,
    #     0, 0,
    #     yaw,
    #     compassmask,
    #     latitude,
    #     longitude,
    #     0,
    #     0,
    #     0)

    # return cmd
    # "
    # """ User defined waypoint item. Ground Station will show the Vehicle as flying through this item. """
    # cmd = Command(
    #     0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.MAV_CMD_WAYPOINT_USER_1,
    #     0, 0, 0, 0, 0, 0,
    #     latitude,
    #     longitude,
    #     altitude)

    # return cmd