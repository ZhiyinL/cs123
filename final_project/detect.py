import cv2
import numpy as np

def detect_ball(image):
    MIN_AREA = 100

    # Convert the image from BGR to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    cv2.imwrite('./images/hsv_image.jpg', hsv_image)

    # Create a mask for the circular region of interest (ROI)
    height, width = image.shape[:2]
    center = (width // 2, height // 2)  # Center of the image
    radius = min(center) - 100  # Radius of the circle (assumes the circle fits within the image)

    mask_roi = np.zeros((height, width), dtype=np.uint8)
    cv2.circle(mask_roi, center, radius - 10, 255, thickness=-1)  # Adjust the radius

    bottom_3_4_mask = np.zeros((height, width), dtype=np.uint8)
    bottom_3_4_mask[int(height * 1/4):, :] = 255

    mask_roi = cv2.bitwise_and(mask_roi, bottom_3_4_mask)
    
    # Apply the circular mask to the image
    masked_image = cv2.bitwise_and(hsv_image, hsv_image, mask=mask_roi)

    cv2.imwrite('./images/yellow_adjusted_image.jpg', masked_image)

    # Define the yellow color range in HSV (we need to tune for ball)
    lower_yellow = np.array([15, 180, 200])
    upper_yellow = np.array([30, 255, 255])

    # Create a binary mask for yellow color
    mask_yellow = cv2.inRange(masked_image, lower_yellow, upper_yellow)

    # Find contours in the yellow mask
    contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Loop through the contours to find the largest circular blob
    largest_blob = None
    max_area = 0

    for contour in contours:
        # Calculate the area and perimeter of the contour
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * (area / (perimeter ** 2))

        # Check for circularity and size
        if circularity > 0 and area > max_area and area > MIN_AREA:  # Adjust thresholds as needed
            largest_blob = contour
            max_area = area

    # Draw the largest blob inside the circular ROI
    if largest_blob is not None:
        (x, y), radius = cv2.minEnclosingCircle(largest_blob)
        center = (int(x), int(y))
        radius = int(radius)

        # Draw the circle on the original image
        cv2.circle(image, center, radius, (0, 255, 0), 2)
        cv2.imwrite('./images/yellow_masked_image.jpg', mask_yellow)
        cv2.imwrite('./images/largest_yellow_blob_image.jpg', image)
        
        return True, center, radius
    return False, None, None
    # # Display the results


def detect_net(image):
    """
    Detects pink regions in the image and marks the median position of all selected pixels.

    Parameters:
    - image: Input BGR image.

    Returns:
    - True if pink is detected, and the marked image.
    - None if no pink is detected.
    """

    # Convert the image from BGR to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create a mask for the circular region of interest (ROI)
    height, width = image.shape[:2]
    center = (width // 2, height // 2)  # Center of the image
    radius = min(center) - 100  # Radius of the circle (assumes the circle fits within the image)

    mask_roi = np.zeros((height, width), dtype=np.uint8)
    cv2.circle(mask_roi, center, radius - 10, 255, thickness=-1)  # Adjust the radius

    masked_image = cv2.bitwise_and(hsv_image, hsv_image, mask=mask_roi)

    cv2.imwrite('./images/pink_adjusted_image.jpg', masked_image)

    # Define the pink color range in HSV
    lower_pink = np.array([130, 75, 150])  # Adjusted range for pink with extended value range
    upper_pink = np.array([180, 255, 255])

    # Create a binary mask for pink color
    mask_pink = cv2.inRange(masked_image, lower_pink, upper_pink)

    # Get the coordinates of all pixels in the mask
    y_coords, x_coords = np.where(mask_pink > 0)

    if len(x_coords) < 100 or len(y_coords) < 100:
        return False, None, None
    
    x_coords, y_coords = get_largest_clump(x_coords, y_coords)

    if len(x_coords) < 100 or len(y_coords) < 100:
        return False, None, None
    

    x_coords = set(x_coords)
    y_coords = set(y_coords)
    # Calculate the median position of the pink pixels
    median_x = int(np.median(list(x_coords)))
    median_y = int(np.median(list(y_coords)))

    # Draw a circle marker on the original image
    center = (median_x, median_y)
    cv2.circle(image, center, 10, (0, 255, 255), -1)  # Yellow marker for visibility

    # Save the results
    cv2.imwrite('./images/Pink_Mask.jpg', mask_pink)
    cv2.imwrite('./images/Marked_Pink_Center.jpg', image)

    return True, center, None

def get_largest_clump(x_coords, y_coords, margin=2):
    if len(x_coords) != len(y_coords):
        raise ValueError("x_coords and y_coords must have the same length")
    
    # Combine and sort the coordinates based on x_coords
    sorted_pairs = sorted(zip(x_coords, y_coords), key=lambda pair: pair[0])
    sorted_x, sorted_y = zip(*sorted_pairs)
    sorted_x = list(sorted_x)
    sorted_y = list(sorted_y)
    
    # Initialize variables to track the largest clump
    max_clump = []
    max_y = []
    
    # Initialize the first clump
    current_clump = [sorted_x[0]]
    current_y = [sorted_y[0]]
    
    # Iterate through the sorted coordinates to find clumps
    for i in range(1, len(sorted_x)):
        if sorted_x[i] - sorted_x[i-1] <= margin:
            # Continue the current clump
            current_clump.append(sorted_x[i])
            current_y.append(sorted_y[i])
        else:
            # Check if the current clump is the largest so far
            if len(current_clump) > len(max_clump):
                max_clump = current_clump
                max_y = current_y
            # Start a new clump
            current_clump = [sorted_x[i]]
            current_y = [sorted_y[i]]
    
    # After the loop, check the last clump
    if len(current_clump) > len(max_clump):
        max_clump = current_clump
        max_y = current_y
    
    return max_clump, max_y

        

