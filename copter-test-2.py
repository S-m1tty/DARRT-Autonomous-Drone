from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

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

#------ MAIN PROGRAM ----
arm_and_takeoff(10)
print("armed and takeoffed")

#-- set the default speed
vehicle.airspeed = 7

#-- Go to wp1
print("go to wp1")
wp1 = LocationGlobalRelative(32.992728, -96.742999, 10)

vehicle.simple_goto(wp1)

#--- Here you can do all your magic....
time.sleep(30)

#--- Coming back
print("Coming back")
vehicle.mode = VehicleMode("RTL")

time.sleep(20)

#-- Close connection
vehicle.close()
