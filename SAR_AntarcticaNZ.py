"""
SAR Autonomous Mission Behaviour.
AntarcticaNZ August 2017.
Author: Neeraj Patel
"""


from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import sys
from select import select

#########################################
#
# DRONE PROPERTIES
#
#########################################

dFSA = 30   # Functional Search Area (m)


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


def getSSMeters(aLocation, alpha, Radius, turn):
    """
    Returns a LocationGlobal object containing the latitude/longitude values of the next position in the sector search
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    
    if turn == 0:
            #Coordinate offsets in radians
            bearing = math.radians(alpha - 180 + 60)
            if bearing >=360:
                bearing += 360
            Lat = (Radius * math.sin(bearing)) / earth_radius
            Lon = (Radius * math.cos(bearing)) / earth_radius
    #
    elif turn == 1:
            #Coordinate offsets in radians
            bearing = math.radians(alpha + 180 - 60)
            if bearing >=360:
                bearing += 360
            Lat = (Radius * math.sin(bearing)) / earth_radius
            Lon = (Radius * math.cos(bearing)) / earth_radius

    #New position in decimal degrees
    newlat = aLocation.lat + (Lat * 180/math.pi)
    newlon = aLocation.lon + (Lon * 180/math.pi)
    return LocationGlobal(newlat, newlon, aLocation.alt)



def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
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
    
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = math.degrees(math.atan2(off_y, off_x))
    
    if bearing < 0:
        bearing += 360.00
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
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


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


###################
# SEARCH PATTERNS #
###################

def addsParallelTrack(Area, initPoint, alt):
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

    #Calculate track properties
    trackLength = 2 * math.sqrt(dFSA/math.pi) 
    legConst = 5                                                    #Arbitrary ratio of leg length to track length 
    legLength = legConst * trackLength                              #Leg length function of dFSA radius 
    numLegs = Area / (trackLength  * legLength)

    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt))

    #Go to initial point as specified by user
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, initPoint.lat, initPoint.lon, alt))
    
    #Define waypoint pattern - point(lat, long, alt)
    i = 1;
    waypoint = initPoint
    while i <= numLegs :
        # Strafe 
        waypoint = get_location_metres(waypoint, 0, (legLength*(-1)**i))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.lat, waypoint.lon, alt))
        # Advance
        waypoint = get_location_metres(waypoint, trackLength, 0)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.lat, waypoint.lon, alt))
        i += 1

    # Return to Launch
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, initPoint.lat, initPoint.lon, alt))
     
    print " Upload new commands to vehicle"
    cmds.upload()



def addsSectorSearch(Area, initPoint, alt):
    """
    Adds mission to perform sector search across specified area.
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
    currLocation = initPoint
    
    #Initial direction 
    alpha = 90

    #Area radius
    Radius = math.sqrt(Area/math.pi)
    
    # 1st waypoint
    waypoint = get_location_metres(currLocation, Radius*math.cos(alpha), Radius*math.sin(alpha))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.lat, waypoint.lon, alt))
    alpha = get_bearing(currLocation, waypoint)
    currLocation = waypoint
    tri = 1

    # Sector Search
    while tri <= 3 :

        triCnr = 1
        while triCnr < 3:
            waypoint = getSSMeters(currLocation, alpha, Radius, 1)
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.lat, waypoint.lon, alt))
            alpha = get_bearing(currLocation, waypoint)
            currLocation = waypoint
            triCnr += 1

        tri += 1
        if tri != 4:
            waypoint = getSSMeters(currLocation, alpha,Radius , 0)
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.lat, waypoint.lon, alt))
            alpha = get_bearing(currLocation, waypoint)
            currLocation = waypoint
        
    # Return to Launch
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, initPoint.lat, initPoint.lon, alt))
     
    print " Upload new commands to vehicle"
    cmds.upload()
    

def addsExpandSquare(initPoint, Area, alt):
    """
     Adds mission to expanding square search search across specified area.
        """
    
    cmds = vehicle.commands
    
    print " Clear any existing commands"
    cmds.clear()
    
    print " Define/add new commands."
    # Add new commands. The meaning/order of the parameters is documented in the Command class.

    #Determine number of loops - Square search area
    Radius = math.sqrt(dFSA / math.pi)
    numLoops = math.sqrt(Area) /  Radius

    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt))

    #Initial Waypoint - Centre of search area
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, initPoint.lat, initPoint.lon, alt))
       
    #Define waypoint pattern - point(lat, long, alt)
    i = 1;
    dist = Radius
    advanceToggle = 1
    strafeToggle = 1 
    waypoint = initPoint
    while i <= numLoops :
        
        if i % 2 == 0:
            # Strafe 
            waypoint = get_location_metres(waypoint, 0, (dist*(-1)**strafeToggle))
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.lat, waypoint.lon, alt))
            strafeToggle ^= 1
            dist += Radius
            
        else:
            # Advance
            waypoint = get_location_metres(waypoint, (dist*(-1)**advanceToggle), 0)
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint.lat, waypoint.lon, alt))
            advanceToggle ^= 1
        i += 1

    # Return to Launch
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, initPoint.lat, initPoint.lon, alt))
     
    print " Upload new commands to vehicle"
    cmds.upload()

    

def manControl():
    """
    Facilitates user manual control. User must specify required search pattern and waypoint.
    """
    
    Pattern = raw_input("'SS' = Sector Search \n 'PT' = Parallel Track \n 'ES' = Expanding Square\nSpecify desired search pattern: ")
    Lat = raw_input("Latitude: ")
    Lon = raw_input("Longitude: ")
    Area = raw_input("Specify search area (m2): ")
    Alt = raw_input("Specify search altitude (m): ")

    point = LocationGlobal(float(Lat), float(Lon), float(Alt))
    
    if Pattern == 'SS':
        addSectorSearch(float(Area), point, float(Alt))
    elif Pattern == 'PT':
        addsParallelTrack(float(Area), point, float(Alt))
    elif Patten == 'ES':
        addsExpandSquare(point, float(Area), float(Alt))

    vehicle.commands.next=0
    vehicle.mode = VehicleMode("AUTO")
        



#########################################
#
# MAIN ROUTINE
#
#########################################

#
# Set Mission
print 'Create mission waypoints around current location.'
initPoint = LocationGlobal(-43.522028, 172.577685, 10)
Area = 10000
alt = 10

#################### Parallel Track Search
##addsParallelTrack(Area, initPoint, alt)

#################### Sector Search
addsSectorSearch(Area, initPoint, alt)

#################### Expanding Square Search
##addsExpandSquare(initPoint, Area, alt)


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

    #wait for user input
    timeout = 2
    print "Enter something:",
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        s = raw_input()
        if str(s) == 'GUIDED':
            print s
            vehicle.mode = VehicleMode("GUIDED")
            manControl()
    else:
        print "No input. Moving on..."
    

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
            







