import cv2
import numpy as np

createImage = True

def detect_ball(image):
    # Convert the image from BGR to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    cv2.imwrite('./images/hsv_image.jpg', hsv_image)

    # Create a mask for the circular region of interest (ROI)
    height, width = image.shape[:2]
    center = (width // 2, height // 2)  # Center of the image
    radius = min(center) - 100  # Radius of the circle (assumes the circle fits within the image)

    mask_roi = np.zeros((height, width), dtype=np.uint8)
    cv2.circle(mask_roi, center, radius - 10, 255, thickness=-1)  # Adjust the radius

    # Apply the circular mask to the image
    masked_image = cv2.bitwise_and(hsv_image, hsv_image, mask=mask_roi)

    # Define the yellow color range in HSV (we need to tune for ball)
    lower_yellow = np.array([15, 180, 200])
    upper_yellow = np.array([30, 255, 255])

    # Create a binary mask for yellow color
    mask_yellow = cv2.inRange(masked_image, lower_yellow, upper_yellow)

    cv2.imwrite('./images/Yellow Mask.jpg', mask_yellow)

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
        if circularity > 0.5 and area > max_area:  # Adjust thresholds as needed
            largest_blob = contour
            max_area = area

    # Draw the largest blob inside the circular ROI
    if largest_blob is not None:
        (x, y), radius = cv2.minEnclosingCircle(largest_blob)
        center = (int(x), int(y))
        radius = int(radius)

        # Draw the circle on the original image
        cv2.circle(image, center, radius, (0, 255, 0), 2)
        cv2.imwrite('./images/Masked Image.jpg', masked_image)
        cv2.imwrite('./images/Detected Largest Blob.jpg', image)
        
        return True, center, radius
    return False, None, None
    # # Display the results
