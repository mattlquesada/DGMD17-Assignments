# There are some additional code lines before this point...

### ---------- This is the application's 'main' asynchronous function ----------
async def run():
    """ Does Offboard control using position NED coordinates. """

    global drone_home_pos
    global cur_goal_point_pos

    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    print("-- Arming")
    await drone.action.arm()

    print("Awaiting to get home position...")
    await get_drone_home_pos(drone)

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

# There are some additional code lines after this point...