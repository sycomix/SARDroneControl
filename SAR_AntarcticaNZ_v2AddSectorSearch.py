"""
SAR Autonomous Mission Behaviour.
AntarcticaNZ August 2017.
Author: Neeraj Patel
"""


from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

#########################################
#
# CONNECTION
#
#########################################

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

#
# SIMULATOR
#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

#
# VEHICLE
# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

#########################################
#
# FUNCTIONS
#
#########################################

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def getSSMeters(aLocation, alpha, dLat, dLon, turn):
    """
    Returns a LocationGlobal object containing the latitude/longitude values of the next position in the sector search
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    
    if turn == 0:
            #Coordinate offsets in radians
            bearing = math.radians(alpha - 180 + 60)
            if bearing >=360:
                bearing += 360
            Lat = (dLat * math.sin(bearing)) / earth_radius
            Lon = (dLat * math.cos(bearing)) / earth_radius
            print "TestCos: %s" % (math.cos(bearing))
            print "New dx: %s" % (Lon* 180/math.pi)
            print "New dy: %s" % (Lat* 180/math.pi)
            print "New Bearing should be: %s" % (bearing)
 
    #
    elif turn == 1:
            #Coordinate offsets in radians
            print  "alpha: %s" % (alpha)
            bearing = math.radians(alpha + 180 - 60)
            if bearing >=360:
                bearing += 360
            Lat = (dLat * math.sin(bearing)) / earth_radius
            Lon = (dLat * math.cos(bearing)) / earth_radius
            print "TestSin: %s" % (math.sin(bearing))
            print "New dx: %s" % (Lon* 180/math.pi)
            print "New dy: %s" % (Lat* 180/math.pi)
            print "New Bearing should be: %s" % (math.degrees(bearing))
 
    #New position in decimal degrees
    print "Old Latitude: %s Longitude: %s" % (aLocation.lat,aLocation.lon)
    newlat = aLocation.lat + (Lat * 180/math.pi)
    newlon = aLocation.lon + (Lon * 180/math.pi)
    print "New Latitude: %s Longitude: %s" % (newlat ,newlon)
    return LocationGlobal(newlat, newlon,aLocation.alt)



def condition_yaw(heading, relative=False):
    is_relative = 1 if relative else 0
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
       0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    
    print "L1 Latitude: %s Longitude: %s" % (aLocation1.lat,aLocation1.lon)
    print "L2 Latitude: %s Longitude: %s" % (aLocation2.lat,aLocation2.lon)
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = math.degrees(math.atan2(off_y, off_x))
    if bearing < 0:
        bearing += 360.00
    print 'dx: %s' % (off_x)
    print 'dy: %s' % (off_y)
    print 'Confirm New Bearing: %s' % (bearing)
    print "\n"
    
    return bearing;

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    return get_distance_metres(
        vehicle.location.global_frame, targetWaypointLocation
    )


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.



def adds_square_mission(aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    cmds = vehicle.commands

    print " Clear any existing commands"
    cmds.clear() 
    
    print " Define/add new commands."
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    

    print " Upload new commands to vehicle"
    cmds.upload()

###################
# SEARCH PATTERNS #
###################

def addsParallelTrack(aLocation, dLat, dLong ,numLegs, alt):
    """
        Adds a takeoff command and four waypoint commands to the current mission.
        The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
        The function assumes vehicle.commands matches the vehicle mission state
        (you must have called download at least once in the session and after clearing the mission)
        """
    
    cmds = vehicle.commands
    
    print " Clear any existing commands"
    cmds.clear()
    
    print " Define/add new commands."
    # Add new commands. The meaning/order of the parameters is documented in the Command class.
    

    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt))
    
    #Define waypoint pattern - point(lat, long, alt)
    i = 1;
    waypoint = aLocation
    while i <= numLegs :
        # Strafe 
        waypoint = get_location_metres(waypoint, 0, (dLat*(-1)**i))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.lat, waypoint.lon, alt))
        # Advance
        waypoint = get_location_metres(waypoint, dLong, 0)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.lat, waypoint.lon, alt))
        i += 1

    # Return to Launch
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, aLocation.lat, aLocation.lon, alt))
     
    print " Upload new commands to vehicle"
    cmds.upload()



def addsSectorSearch(aLocation, dLat, dLong , alt, alpha, initPoint):
    """
        Adds a takeoff command and four waypoint commands to the current mission.
        The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
        The function assumes vehicle.commands matches the vehicle mission state
        (you must have called download at least once in the session and after clearing the mission)
        """
    
    cmds = vehicle.commands
    
    print " Clear any existing commands"
    cmds.clear()
    
    print " Define/add new commands."
    # Add new commands. The meaning/order of the parameters is documented in the Command class.
    
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt))
    
    #Define waypoint pattern - point(lat, long, alt)

    #Initial waypoint
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, initPoint.lat, initPoint.lon, alt))

    # 1st waypoint
    waypoint = get_location_metres(aLocation, dLat*math.cos(alpha), dLat*math.sin(alpha))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.lat, waypoint.lon, alt))
    alpha = get_bearing(aLocation, waypoint)
    aLocation = waypoint
    tri = 1

    # Sector Search
    while tri <= 3 :

        triCnr = 1
        while triCnr < 3:
            waypoint = getSSMeters(aLocation, alpha, dLat, dLong, 1)
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.lat, waypoint.lon, alt))
            alpha = get_bearing(aLocation, waypoint)
            aLocation = waypoint
            triCnr += 1

        waypoint = getSSMeters(aLocation, alpha, dLat, dLong, 0)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.lat, waypoint.lon, alt))
        alpha = get_bearing(aLocation, waypoint)
        aLocation = waypoint
        
        tri += 1

    # Return to Launch
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, aLocation.lat, aLocation.lon, alt))
     
    print " Upload new commands to vehicle"
    cmds.upload()

    
    
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

        
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)





#########################################
#
# MAIN ROUTINE
#
#########################################

#
# Set Mission
print 'Create mission waypoints around current location.'
numLegs = 3
#addsParallelTrack(vehicle.location.global_frame, 50, 20 ,numLegs, 5)

initPoint = vehicle.location.global_frame
dLat = 50
dLon = 50
alt = 10
alpha = 20
addsSectorSearch(initPoint, dLat, dLon , alt, alpha, initPoint)


# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
arm_and_takeoff(10)

print "Starting mission"
# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")


# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.

while True:
    nextwaypoint=vehicle.commands.next
    print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())
  

##      # Poll Beacon Signal (
##      # if Beacon recieved
##      #   Determine direction
##        """"""
##      #Set to GUIDED mode
##        # Once we have a valid location (see gpsd documentation) we can start moving our vehicle around
##        if (gpsd.valid & gps.LATLON_SET) != 0:
##            altitude = 30  # in meters
##            dest = LocationGlobalRelative(gpsd.fix.latitude, gpsd.fix.longitude, altitude)
##            print "Going to: %s" % dest
##
##            # A better implementation would only send new waypoints if the position had changed significantly
##            vehicle.simple_goto(dest)
##
##            # Send a new target every two seconds
##            # For a complete implementation of follow me you'd want adjust this delay
##            time.sleep(2)
    time.sleep(1)

print 'Return to launch'
vehicle.mode = VehicleMode("RTL")


#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

#
#FOLLOW ME
##Try:
##    # Use the python gps package to access the laptop GPS
##    gpsd = gps.gps(mode=gps.WATCH_ENABLE)
##
##    #Arm and take off to altitude of 5 meters
##    arm_and_takeoff(5)
##
##    while True:
##    
##        if vehicle.mode.name != "GUIDED":
##            print "User has changed flight modes - aborting follow-me"
##            break    
##            
##        # Read the GPS state from the laptop
##        gpsd.next()
##
##        # Once we have a valid location (see gpsd documentation) we can start moving our vehicle around
##        if (gpsd.valid & gps.LATLON_SET) != 0:
##            altitude = 30  # in meters
##            dest = LocationGlobalRelative(gpsd.fix.latitude, gpsd.fix.longitude, altitude)
##            print "Going to: %s" % dest
##
##            # A better implementation would only send new waypoints if the position had changed significantly
##            vehicle.simple_goto(dest)
##
##            # Send a new target every two seconds
##            # For a complete implementation of follow me you'd want adjust this delay
##            time.sleep(2)
##except socket.error:
##    print "Error: gpsd service does not seem to be running, plug in USB GPS or run run-fake-gps.sh"
##    sys.exit(1)


#
# VELOCITY CONTROL
##def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
##    """
##    Move vehicle in direction based on specified velocity vectors.
##    """
##    msg = vehicle.message_factory.set_position_target_local_ned_encode(
##        0,       # time_boot_ms (not used)
##        0, 0,    # target system, target component
##        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
##        0b0000111111000111, # type_mask (only speeds enabled)
##        0, 0, 0, # x, y, z positions (not used)
##        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
##        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
##        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
##
##
##    # send command to vehicle on 1 Hz cycle
##    for x in range(0,duration):
##        vehicle.send_mavlink(msg)
##        time.sleep(1)
            







