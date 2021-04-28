from dronekit import connect, VehicleMode
from preflight import Check

class AutoMode(object):

    def send_all_commands(self, gcs_credentials, connection_string, list_of_commands):

        print("Connecting to vehicle...")
        vehicle = connect(connection_string, wait_ready=True)
        vehicle.mode = VehicleMode("AUTO")

        vehicle.run_check(gcs_credentials, connection_string)

        print("1/3 - Initializing commands...")
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
    
        print("2/3 - Adding commands...")
        for cmd in list_of_commands:
            cmds.add(cmd)
        
        print("3/3 - Uploading commands...")
        cmds.upload()