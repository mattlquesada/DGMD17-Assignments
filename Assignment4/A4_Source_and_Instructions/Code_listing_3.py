async def moving_average_filter(coord):
    """ Applies Low-Pass Moving Average Filter to a pair of (x, y) coordinates."""

    filt_buffer['width'].append(coord['width'])
    filt_buffer['height'].append(coord['height'])

    if len(filt_buffer['width']) > NUM_FILT_POINTS:
        filt_buffer['width'] = filt_buffer['width'][1:]
        filt_buffer['height'] = filt_buffer['height'][1:]
    
    N = len(filt_buffer['width'])

    w_sum = sum( filt_buffer['width'] )
    h_sum = sum( filt_buffer['height'] )

    w_filt = int(round(w_sum / N))
    h_filt = int(round(h_sum / N))

    return {'width':w_filt, 'height':h_filt}


async def draw_objects(cam_coord, filt_cam_coord, frame, contour):
    """ Draws visualization objects from the detection process.
        Position coordinates of every object are always in 'camera pixel' units"""

    cv2.line(frame, (0, params['x_ax_pos']), (params['resized_width'], params['x_ax_pos']), (0, 128, 255), 1)
    cv2.line(frame, (params['y_ax_pos'], 0), (params['y_ax_pos'], params['resized_height']), (0, 128, 255), 1)
    cv2.circle(frame, (params['y_ax_pos'], params['x_ax_pos']), 1, (255, 255, 255), -1)
    
    cv2.rectangle(frame, params['cent_rect_p1'], params['cent_rect_p2'], (0, 178, 255), 1)

    cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)

    x_cart_coord = cam_coord['width'] - params['y_ax_pos']
    y_cart_coord = params['x_ax_pos'] - cam_coord['height']

    x_filt_cart_coord = filt_cam_coord['width'] - params['y_ax_pos']
    y_filt_cart_coord = params['x_ax_pos'] - filt_cam_coord['height']

    cv2.circle(frame, (cam_coord['width'], cam_coord['height']), 5, (0, 0, 255), -1) 
    cv2.putText(frame, str(x_cart_coord) + ", " + str(y_cart_coord), 
        (cam_coord['width'] + 25, cam_coord['height'] - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    cv2.circle(frame, (filt_cam_coord['width'], filt_cam_coord['height']), 5, (255, 30, 30), -1)
    cv2.putText(frame, str(x_filt_cart_coord) + ", " + str(y_filt_cart_coord), 
        (filt_cam_coord['width'] + 25, filt_cam_coord['height'] - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 30, 30), 1)

    return frame