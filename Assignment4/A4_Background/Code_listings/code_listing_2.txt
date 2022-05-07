# There are some additional code lines before this point...

    c_letter_1 = [[0.0, 0.0, -5.0, 0.0], [1.0, -1.0, -5.0, 30.0], [1.0, -3.0, -5.0, 60.0],
                [0.0, -4.0, -5.0, 90.0], [-3.0, -4.0, -5.0, 120.0], [-4.0, -3.0, -5.0, 180.0], 
                [-4.0, -1.0, -5.0, 210.0], [-3.0, 0.0, -5.0, 0.0]]

    c_letter_2 = []
    for point in c_letter_1 :
        c_letter_2.append([point[0], point[1]+6, point[2], point[3]])

    m_letter = [[-4.0, 8.0, -5.0, 0.0], [1.0, 8.0, -5.0, 0.0], [-1.0, 10.0, -5.0, 0.0],
                [1.0, 12.0, -5.0, 0.0], [-4.0, 12.0, -5.0, 0.0]]

    path_points_list = c_letter_1 + c_letter_2 + m_letter

    for goal_point in path_points_list:

        north_coord = goal_point[0]
        east_coord  = goal_point[1]
        down_coord  = goal_point[2]
        yaw_angle   = goal_point[3]

        goal_point_angle = atan2(north_coord , east_coord);
        distance_to_goal_point = sqrt((east_coord**2) + (north_coord**2)) / 1000.0 # In kilometers

        goal_point_bearing = (450 - degrees(goal_point_angle)) % 360;

        dest_latlong = get_dest_latlong(drone_home_pos['lat'], drone_home_pos['lon'], 
                                            distance_to_goal_point, goal_point_bearing)

        cur_goal_point_pos['lat'] = dest_latlong[0]
        cur_goal_point_pos['lon'] = dest_latlong[1]
        cur_goal_point_pos['rel_alt'] = -down_coord
        
        await drone.offboard.set_position_ned(
            PositionNedYaw(north_coord , east_coord, down_coord, yaw_angle))

        await check_is_at_goal(drone)

# There are some additional code lines after this point...