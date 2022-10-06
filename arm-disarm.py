# READ THIS
# Dependencies:
# -- python3
# -- pip3
# -- dronekit (python package)
# -- dronekit-sitl (python package)
# -- MAVProxy==1.8.29 (python package)
# -- pymavlink==2.4.8 (python package)
# Some of these are older versions on purpose because the newer ones didn't work

from dronekit import connect, VehicleMode, LocationGlobalRelative
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


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
    sitl_args = ['-I0', '--model', 'quad', '--home=32.997176,-96.743252,0,180']
    sitl.launch(sitl_args, await_ready=True, restart=True)

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

#-- Define the function for arm-disarm test
def arm_disarm(arm_time):
    print("Arming motors")

    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("STABILIZE")
    if vehicle.is_armable:
        print("Vehicle is Armable")
#    vehicle.armed = True
    
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
        vehicle.armed = True

    print("Now we would normally takeoff")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(arm_time)
    vehicle.armed = False
    print("Disarmed vehicle")

#------ MAIN PROGRAM ----
arm_disarm(1)
print("armed and takeoffed")

print("Shutting down")

#-- Close connection
vehicle.close()
