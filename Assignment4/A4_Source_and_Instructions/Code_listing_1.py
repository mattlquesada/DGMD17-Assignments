### ---------- This is the application's 'main' asynchronous function ----------
async def run():
    """ Detects a target by using color range segmentation and follows it
    by using Offboard control and position NED coordinates. """
    
    vid_cam = cv2.VideoCapture(0)

    if vid_cam.isOpened() is False: 
        print('[ERROR] Couldnt open the camera.')
        return
    print('-- Camera opened successfully')

    await get_image_params(vid_cam) 
    print(f"-- Original image width, height: {params['image_width']}, {params['image_height']}")

    drone = System()
    await drone.connect(system_address="udp://:14540")
    # ... Additional drone initialization code lines go here...

    N_coord = 0
    E_coord = 0
    D_coord = -HOVERING_ALTITUDE
    yaw_angle = 0

    await drone.offboard.set_position_ned(PositionNedYaw(N_coord, E_coord, D_coord, yaw_angle))
    await asyncio.sleep(4)

    while True:
        tgt_cam_coord, frame, contour = await get_target_coordinates(vid_cam)
        
        if tgt_cam_coord['width'] is not None and tgt_cam_coord['height'] is not None:
            tgt_filt_cam_coord = await moving_average_filter(tgt_cam_coord)
        else:
            tgt_cam_coord = {'width':params['y_ax_pos'], 'height':params['x_ax_pos']}
            tgt_filt_cam_coord = {'width':params['y_ax_pos'], 'height':params['x_ax_pos']}

        tgt_cart_coord = {'x':(tgt_filt_cam_coord['width'] - params['y_ax_pos']),
                          'y':(params['x_ax_pos'] - tgt_filt_cam_coord['height'])}

        COORD_SYS_CONV_FACTOR = 0.1

        if abs(tgt_cart_coord['x']) > params['cent_rect_half_width'] or \
        abs(tgt_cart_coord['y']) > params['cent_rect_half_height']:
            E_coord = tgt_cart_coord['x'] * COORD_SYS_CONV_FACTOR
            N_coord = tgt_cart_coord['y'] * COORD_SYS_CONV_FACTOR
            # D_coord, yaw_angle don't change

        await drone.offboard.set_position_ned(PositionNedYaw(N_coord, E_coord, D_coord, yaw_angle))

        frame = await draw_objects(tgt_cam_coord, tgt_filt_cam_coord, frame, contour)

        cv2.imshow("Detect and Track", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    print("-- Return to launch...")
    await drone.action.return_to_launch()
    print("NOTE: check the drone has landed already before running again this script.")
    await asyncio.sleep(5)