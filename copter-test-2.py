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

#-- Define the function for takeoff
def arm_and_takeoff(tgt_altitude):
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
        time.sleep(1)

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

#-- Define the function for getting distance to given waypoint
def distance_to_current_waypoint(wp):
    """
    Gets distance in meters to the current waypoint. 
    """
    targetWaypointLocation = wp
    distancetopoint = get_distance_meters(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

#-- Define the function for getting distance between two points
def get_distance_meters(aLocation1, aLocation2):
    """
    Returns the ground distance in meters between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

#------ MAIN PROGRAM ----
arm_and_takeoff(10)
print("armed and takeoffed")

#-- set the default speed
vehicle.airspeed = 7

#-- Go to wp1
print("go to wp1")
wp1 = LocationGlobalRelative(32.996386, -96.743151, 10)

vehicle.simple_goto(wp1)

#--- Check if we've hit the waypoint....
while True:
    nextwaypoint=vehicle.commands.next
    print("Distance to waypoint (%s): %s" % (nextwaypoint, distance_to_current_waypoint(wp1)))
    print("Global location: %s" % vehicle.location.global_frame)
    print("Local location: %s" % vehicle.location.local_frame)
    time.sleep(5)
    if distance_to_current_waypoint(wp1) < 5:
        break

#--- Coming back
print("Coming back")
vehicle.mode = VehicleMode("RTL")

while True:
    nextwaypoint = LocationGlobalRelative(32.997176, -96.743252, 0) # home position
    print("Distance to waypoint (%s): %s" % (nextwaypoint, distance_to_current_waypoint(wp1)))
    print("Global location: %s" % vehicle.location.global_frame)
    print("Local location: %s" % vehicle.location.local_frame)
    time.sleep(5)
    if distance_to_current_waypoint(nextwaypoint) < 5:
        break

while True:
    if vehicle.location.global_relative_frame.alt < 0.5:
        vehicle.armed = False
        break

print("Shutting down")

#-- Close connection
vehicle.close()
