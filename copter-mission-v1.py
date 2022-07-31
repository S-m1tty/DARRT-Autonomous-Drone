# READ THIS
# Dependencies:
# -- python3
# -- pip3
# -- dronekit (python package)
# -- dronekit-sitl (python package)
# -- MAVProxy==1.8.29 (python package)
# -- pymavlink==2.4.8 (python package)
# Some of these are older versions on purpose because the newer ones didn't work

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
import math

#-- Connect to the vehicle
#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
    sitl_args = ['-I0', '--model', 'quad', '--home=32.9956561,-96.7430654,0,180']
    sitl.launch(sitl_args, await_ready=True, restart=True)

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

#-- Define mission reading function
def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist

#-- Define mission upload function
def upload_mission(aFileName):
    """
    Upload a mission from a file.
    """
    # Read mission from file
    missionlist = readmission(aFileName)

    print("\nUpload mission from a file: %s" % aFileName)
    # Clear existing mission from vehicle
    print('Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    # Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()


#------ MAIN PROGRAM ----
import_mission_filename = 'mission1.txt'
upload_mission(import_mission_filename)

print("made it this far")
# Confirm vehicle armed before attempting to take off
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)
    vehicle.armed = True
    time.sleep(1)

tgt_altitude = 10

print("Takeoff")
vehicle.mode = VehicleMode("GUIDED")
vehicle.simple_takeoff(tgt_altitude)
print("executed takeoff command")

if vehicle.armed == True:
    print("WE ARE ARMED AND READY!!")
    vehicle.mode = VehicleMode("GUIDED")
else:
    print("not armed")

#-- wait to reach the target altitude
while True:
    altitude = vehicle.location.global_relative_frame.alt

    print("Vehicle Armed Status: %s" % vehicle.armed)
    print("Vehicle mode is: %s" % vehicle.mode)
    if altitude >= tgt_altitude -1:
        print("Altitude reached")
        break
    time.sleep(1)

#-- set the default speed
vehicle.airspeed = 7

#-- Go to AUTO Mode
vehicle.mode = VehicleMode("AUTO")
while vehicle.armed:
    if not vehicle.armed:
        break

print("Shutting down")

#-- Close connection
vehicle.close()
