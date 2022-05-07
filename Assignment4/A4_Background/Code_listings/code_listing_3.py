async def get_drone_home_pos(drone):
    
    global drone_home_pos

    async for home_position in drone.telemetry.home():

        drone_home_pos['lat'] = home_position.latitude_deg
        drone_home_pos['lon'] = home_position.longitude_deg
        drone_home_pos['rel_alt'] = home_position.relative_altitude_m

        return

async def check_is_at_goal(drone):

    global cur_goal_point_pos

    drone_current_pos = {'lat': None, 'lon': None, 'rel_alt': None}

    prev_round_hor_dist_to_goal = None

    async for position in drone.telemetry.position():

        drone_current_pos['lat'] = position.latitude_deg
        drone_current_pos['lon'] = position.longitude_deg
        drone_current_pos['rel_alt'] = position.relative_altitude_m

        hor_dist_to_goal = 1000.0 * get_haversine_distance(drone_current_pos['lat'], drone_current_pos['lon'], 
                                                        cur_goal_point_pos['lat'], cur_goal_point_pos['lon'])

        ver_dist_to_goal = abs(cur_goal_point_pos['rel_alt'] - drone_current_pos['rel_alt'])

        round_hor_dist_to_goal = round(hor_dist_to_goal)
        if round_hor_dist_to_goal != prev_round_hor_dist_to_goal:
            prev_round_hor_dist_to_goal = round_hor_dist_to_goal
            print(f"...round_hor_dist_to_goal: {round_hor_dist_to_goal}")

        if hor_dist_to_goal <= MAX_HOR_DIST_ERROR and ver_dist_to_goal <= MAX_VER_DIST_ERROR:
            print(f">>> Current goal reached!, hor_dist_to_goal={(hor_dist_to_goal):.2f} mts")
            return