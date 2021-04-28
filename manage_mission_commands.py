from dronekit import connect
from dronekit import Command
from pymavlink import mavutil
from . import auto_mode_commands


class AutoMode(object):

    def send_all_commands(self, list_of_commands):
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()

        for cmd in list_of_commands:
            cmds.add(cmd)
        
        cmds.upload()