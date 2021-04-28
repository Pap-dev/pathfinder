def mav_cmd_user_5():
	""" User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. """

	cmd = Command(
		0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,)
		mavutil.mavlink.MAV_CMD_USER_5,
		0,
		0,
		0,
		0,
		0,
		0,
		0)

	return cmd
