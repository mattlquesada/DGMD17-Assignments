async def get_image_params(vid_cam):
    """ Computes useful general parameters derived from the camera image size."""

    _, frame = vid_cam.read()
    params['image_height'], params['image_width'], _ = frame.shape

    if params['image_height'] != DESIRED_IMAGE_HEIGHT:
        params['scaling_factor'] = round((DESIRED_IMAGE_HEIGHT / params['image_height']), 2)
    else:
        params['scaling_factor'] = 1

    params['resized_width'] = int(params['image_width'] * params['scaling_factor'])
    params['resized_height'] = int(params['image_height'] * params['scaling_factor'])
    dimension = (params['resized_width'], params['resized_height'])
    frame = cv2.resize(frame, dimension, interpolation = cv2.INTER_AREA)

    params['cent_rect_half_width'] = round(params['resized_width'] * (0.5 * PERCENT_CENTER_RECT)) 
    params['cent_rect_half_height'] = round(params['resized_height'] * (0.5 * PERCENT_CENTER_RECT)) 

    params['min_tgt_radius'] = round(params['resized_width'] * PERCENT_TARGET_RADIUS)

    params['x_ax_pos'] = int(params['resized_height']/2 - 1)
    params['y_ax_pos'] = int(params['resized_width']/2 - 1)

    params['cent_rect_p1'] = (params['y_ax_pos']-params['cent_rect_half_width'], 
                              params['x_ax_pos']-params['cent_rect_half_height'])
    params['cent_rect_p2'] = (params['y_ax_pos']+params['cent_rect_half_width'], 
                              params['x_ax_pos']+params['cent_rect_half_height'])

    return

async def get_target_coordinates(vid_cam):
    """ Detects a target by using color range segmentation and returns its
        'camera pixel' coordinates."""

    HSV_LOWER_BOUND = (107, 119, 41)
    HSV_UPPER_BOUND = (124, 255, 255)

    _, frame = vid_cam.read()

    if params['scaling_factor'] != 1:
        dimension = (params['resized_width'], params['resized_height'])
        frame = cv2.resize(frame, dimension, interpolation = cv2.INTER_AREA)

    blurred = cv2.GaussianBlur(frame, (11, 11), 0) 
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER_BOUND, HSV_UPPER_BOUND)
    mask = cv2.erode(mask, None, iterations=3)
    mask = cv2.dilate(mask, None, iterations=3)

    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cX = None
    cY = None
    largest_contour = None

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

        if radius > params['min_tgt_radius']:
            M = cv2.moments(largest_contour)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

    return {'width':cX, 'height':cY}, frame, largest_contour