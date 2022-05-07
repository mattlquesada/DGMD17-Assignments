#!/usr/bin/env python3
'''
'write_my_initials.py'

The drone follows a predetermined path to write your name initials on the air.

Author: Raul Alvarez-Torrico (raul@tecbolivia.com)

This code example is based on the 'offboard_position_ned.py' and the 'telemetry_takeoff_and_land.py'
examples included in the MAVSDK-Python repository (see the 'examples' folder).
'''

import asyncio
from mavsdk import System
from mavsdk import (OffboardError, PositionNedYaw)
from math import radians, degrees, cos, sin, asin, sqrt, atan2, pi

# Constants. 
# Max altitud and distance error to consider the current goal reached
MAX_VER_DIST_ERROR= 1.0
MAX_HOR_DIST_ERROR  = 1.0

# Python dictionary for storing the drone's initial position 
# (latitude, longitude and relative altitude)
drone_home_pos = {'lat': None, 'lon': None, 'rel_alt': None}

drone_home_pos2 = {'lat': None, 'lon': None, 'rel_alt': None}

# Python dictionary for storing the current goal point's position 
# (latitude, longitude and relative altitude)
cur_goal_point_pos = {'lat': None, 'lon': None, 'rel_alt': None}

### ---------- This is the application's 'main' asynchronous function ----------
async def run():
    """ Does Offboard control using position NED coordinates. """

    # Declare global variables that will be used inside this function
    global drone_home_pos
    global cur_goal_point_pos

    # Get a reference to 'System' object, which represents the drone,
    # and open a connection to it. The system address used here is the default
    # address for a simulated drone in the same machine machine where this code
    # will run (localhost)
    drone = System()
    await drone.connect(system_address="udp://:14540") # To run with SITL simulation
    # await drone.connect(system_address="serial:///dev/ttyUSB0:57600") # To run with a real drone via telem. module

    # Asynchronously poll the connection state until receiving an
    # 'is_connected' confirmation
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    # Activate the drone motors
    print("-- Arming")
    await drone.action.arm()

    # Wait until the home position GPS coordinates have been received.
    print("Awaiting to get home position...")
    await get_drone_home_pos(drone)

    # Send an initial position before changing to "offboard" flight mode
    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    # Change flight mode to "offboard", if it fails, disarm the motors and abort
    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Define lists of NED coordinates + Bearing angle points describing the letters
    # to be written in the air
    path_points_list = []

    # The first point to which the drone must go is climb up to a certain altitude
    # first_point = [0.0, 0.0, -5.0, 0.0]

    # Points for the 'C' letter in 'CCM'
    c_letter_1 = [[0.0, 0.0, -5.0, 0.0], [1.0, -1.0, -5.0, 30.0], [1.0, -3.0, -5.0, 60.0],
                    [0.0, -4.0, -5.0, 90.0], [-3.0, -4.0, -5.0, 120.0], [-4.0, -3.0, -5.0, 180.0], 
                    [-4.0, -1.0, -5.0, 210.0], [-3.0, 0.0, -5.0, 0.0]]

    # Displace the points of the first 'C' letter by 6 units to the right (to the East)
    # to obtain the second 'C' letter in 'CCM'
    c_letter_2 = []
    for point in c_letter_1 :
        c_letter_2.append([point[0], point[1]+6, point[2], point[3]])

    # Points for the 'M' letter in 'CCM'
    m_letter = [[-4.0, 8.0, -5.0, 0.0], [1.0, 8.0, -5.0, 0.0], [-1.0, 10.0, -5.0, 0.0],
                [1.0, 12.0, -5.0, 0.0], [-4.0, 12.0, -5.0, 0.0]]

    # Concatenate all letter lists to obtain the full path
    path_points_list = c_letter_1 + c_letter_2 + m_letter

    # Iterate over each point in the full path and command the drone to it
    for goal_point in path_points_list:
        
        print("----------------------------------------------")

        # Optionally, scale letter size by a factor (to write bigger letters)
        LETTER_HOR_SCALING_FACTOR = 5.0 # Use 1.0 for no scaling

        north_coord = goal_point[0] * LETTER_HOR_SCALING_FACTOR # north_m in NED coordinates
        east_coord  = goal_point[1] * LETTER_HOR_SCALING_FACTOR # east_m in NED coordinates
        down_coord  = goal_point[2]
        yaw_angle   = goal_point[3]

        print(f"CURRENT GOAL POINT: east_coord={east_coord} | north_coord={north_coord}")

        # Compute the goal_point angle w.r.t. the horizontal coordinate
        # and distance from origin. As if it were in the XY Cartesian plane.
        goal_point_angle = atan2(north_coord , east_coord);
        distance_to_goal_point = sqrt((east_coord**2) + (north_coord**2)) / 1000.0 # In kilometers

        # Convert goal_point angle to goal_point_bearing (i.e. angle w.r.t. northern axis)
        goal_point_bearing = (450 - degrees(goal_point_angle)) % 360;

        print(f"distance_to_goal_point (in mts) ={(distance_to_goal_point*1000): .1f} | goal_point_bearing={goal_point_bearing:.1f}")

        # Get the current goal's GPS coordinates
        dest_latlong = get_dest_latlong(drone_home_pos['lat'], drone_home_pos['lon'], 
                                            distance_to_goal_point, goal_point_bearing)

        # Populate the current goal point position dictionary, which will be used
        # at check_is_at_goal() to check if the drone reached the goal
        cur_goal_point_pos['lat'] = dest_latlong[0]
        cur_goal_point_pos['lon'] = dest_latlong[1]
        cur_goal_point_pos['rel_alt'] = -down_coord
        
        # Drive the drone to the current goal point
        await drone.offboard.set_position_ned(
            PositionNedYaw(north_coord , east_coord, down_coord, yaw_angle))

        # Wait until the current goal is reached
        await check_is_at_goal(drone)

    print("-- Path completed. Waiting 5 sec. before returning to launch...")
    await asyncio.sleep(15)

    print("-- Return to launch...")
    await drone.action.return_to_launch()
    print("-- Check the drone has landed already before running again this script.")


async def get_drone_home_pos(drone):
    """ Gets the first arriving GPS home position telemetry data """
    
    # Declare global variables that will be used inside this function
    global drone_home_pos

    # Get home position telemetry
    async for home_position in drone.telemetry.home():

        drone_home_pos['lat'] = home_position.latitude_deg
        drone_home_pos['lon'] = home_position.longitude_deg
        drone_home_pos['rel_alt'] = home_position.relative_altitude_m

        print(f"Start latitude: {drone_home_pos['lat']} | Start longitude: {drone_home_pos['lon']} | Start altitude: {drone_home_pos['rel_alt']}")
        
        # Home position doesn't change until the next take off, return
        # after obtaining the first reading
        return


async def check_is_at_goal(drone):
    """ Check if the drone reached the current goal position """

    # Declare global variables that will be used inside this function
    global cur_goal_point_pos

    drone_current_pos = {'lat': None, 'lon': None, 'rel_alt': None}

    prev_round_hor_dist_to_goal = None

    # Get position telemetry
    async for position in drone.telemetry.position():

        # Populate the drone current position dictionary
        drone_current_pos['lat'] = position.latitude_deg
        drone_current_pos['lon'] = position.longitude_deg
        drone_current_pos['rel_alt'] = position.relative_altitude_m

        # Compute the horizontal distance to current goal in meters
        hor_dist_to_goal = 1000.0 * get_haversine_distance(drone_current_pos['lat'], drone_current_pos['lon'], 
                                                        cur_goal_point_pos['lat'], cur_goal_point_pos['lon'])
        # Compute the vertical distance to current goal in meters
        ver_dist_to_goal = abs(cur_goal_point_pos['rel_alt'] - drone_current_pos['rel_alt'])

        # Just for debugging: Print the horizontal distance if it has been reduced
        # at least by 1 meter.
        round_hor_dist_to_goal = round(hor_dist_to_goal)
        if round_hor_dist_to_goal != prev_round_hor_dist_to_goal:
            prev_round_hor_dist_to_goal = round_hor_dist_to_goal
            print(f"...round_hor_dist_to_goal: {round_hor_dist_to_goal}")

        # Check if the drone is sufficiently near the goal point, within some error range
        if hor_dist_to_goal <= MAX_HOR_DIST_ERROR and ver_dist_to_goal <= MAX_VER_DIST_ERROR:
            print(f">>> Current goal reached!, hor_dist_to_goal={(hor_dist_to_goal):.2f} mts")
            return


# The following funtion is adapted from:
# https://stackoverflow.com/questions/7707904/php-calculate-lat-lng-of-the-square-around-a-given-point-on-the-surface
# Distance is in km, alat and alon are in degrees
def get_dest_latlong(alat, alon, distance, bearing):
    """ Computes the target latitude and longitud, given a start latitude and longitude
        position and distance/bearing of the destination point """
    alatRad = alat * pi / 180;
    alonRad = alon * pi / 180;
    bearing = bearing * pi / 180;
    alatRadSin = sin(alatRad);
    alatRadCos = cos(alatRad);
    
    # Ratio of distance to earth's radius
    angularDistance = distance / 6370.997;
    angDistSin = sin(angularDistance);
    angDistCos = cos(angularDistance);
    xlatRad = asin(alatRadSin*angDistCos + alatRadCos*angDistSin*cos(bearing) );
    xlonRad = alonRad + atan2(sin(bearing)*angDistSin*alatRadCos, angDistCos-alatRadSin*sin(xlatRad));
    
    # Return latitude and longitude as two element array in degrees
    xlat = xlatRad*180/pi;
    xlon = xlonRad*180/pi;
    
    if(xlat > 90):
        xlat = 90;
    if(xlat < -90):
        xlat = -90;
    while(xlat > 180):
        xlat -= 360;
    while(xlat <= -180):
        xlat += 360;
    while(xlon > 180):
        xlon -= 360;
    while(xlon <= -180):
        xlon += 360;
    
    return [xlat, xlon]
    
# The following funtion is adapted from:
# https://python-decompiler.com/article/2011-02/haversine-formula-in-python-bearing-and-distance-between-two-gps-points
def get_haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance in kilometers between two points 
    on the earth. (lat/lon specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6370.997 # radius of earth in kilometers. Use 3956 for miles
    return c * r # return distance in kilometers

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())