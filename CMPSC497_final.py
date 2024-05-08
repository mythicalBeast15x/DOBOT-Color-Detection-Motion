import DobotDllType as dType
import cv2
import numpy as np
import time

api = dType.load()

# Connect Dobot
state = dType.ConnectDobot(api, "COM5", 115200)[0]

print('Dobot Connected...')
suction_cup = 1
enable_pump = 1
ctrl_mode = 1
pos = dType.GetPose(api)
x = pos[0]
y = pos[1]
z = pos[2]
rHead = pos[3]

print(x, y, z, rHead)

# TODO - SET PROPER PICKUP, DEAD-ZONE AND INITIAL-START POSITIONS
pickUpSpot = [162.9602508544922, -31.008365631103516, 11.436935424804688, -12.367464065551758]
deadZoneSpot = [99.04, -133.19, 85.58, -53.36]
initialShapeStart = [15.371467590332031, 139.82501220703125, -46.36102294921875, 82.13253784179688]

block_counter = {color: 0 for color in ["Red", "Blue", "Green", "Yellow"]}
block_threshold = 2


def detect_color(frame):
    # Define ROI dimensions
    roi_width = 50
    roi_height = 50

    # Get frame dimensions
    frame_height, frame_width = frame.shape[:2]

    # Calculate ROI coordinates
    roi_x = max(0, (frame_width - roi_width) // 2)
    roi_y = max(0, (frame_height - roi_height) // 2)

    # Extract ROI
    roi = frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]

    # Convert ROI from BGR to HSV color space
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Define range for each color
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([140, 255, 255])

    lower_green = np.array([40, 100, 100])
    upper_green = np.array([80, 255, 255])

    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # Threshold the HSV image to get only desired colors
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Combine masks to get final mask
    mask = mask_red + mask_blue + mask_green + mask_yellow

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Check for detected color and return corresponding label
    if contours:
        area = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(area)
        if w * h > 100:  # Adjust this threshold as needed
            if mask_red[y + h // 2, x + w // 2]:
                return "Red"
            elif mask_blue[y + h // 2, x + w // 2]:
                return "Blue"
            elif mask_green[y + h // 2, x + w // 2]:
                return "Green"
            elif mask_yellow[y + h // 2, x + w // 2]:
                return "Yellow"

    return None


# Number of consecutive detections required
detectionThreshold = 10
# Amount of time to wait after last detection
timeElapsedAfterDetection = 5  # Adjust as needed
cap = cv2.VideoCapture(0)
detection_time = None
color_counter = {color: 0 for color in ["Red", "Blue", "Green", "Yellow"]}

while True:
    ret, frame = cap.read()
    if ret:
        color = detect_color(frame)
        if color:
            print("Color Detected: ", color)
            detection_time = time.time()  # Record time of detection

            # Increment the counter for the detected color
            color_counter[color] += 1

            # Check if the counter for the detected color reaches the threshold
            if color_counter[color] >= detectionThreshold:
                break  # Exit the loop if the threshold is reached

        # Display the frame
        cv2.imshow('Frame', frame)

    # Check if a color was detected and if enough time has passed
    if detection_time and time.time() - detection_time >= timeElapsedAfterDetection:
        break

    # Check for key press to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


def placeBlock(initial_pos, curr_color):
    print("Choosing placement position...")
    if curr_color == "Red":
        if color_counter[curr_color] == 0:
            dType.SetPTPCmd(api, 2, initial_pos[0], initial_pos[1], initial_pos[2], initial_pos[3], 0)
            enable_pump = 0
            dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
            dType.SetPTPCmd(api, 2, x, y, z, rHead, 0)

        elif color_counter[curr_color] == 1:
            dType.SetPTPCmd(api, 2, initial_pos[0] + 27, initial_pos[1], initial_pos[2], initial_pos[3], 0)
            enable_pump = 0
            dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
            dType.SetPTPCmd(api, 2, x, y, z, rHead, 0)

    elif curr_color == "Green":
        if color_counter[curr_color] == 0:
            dType.SetPTPCmd(api, 2, initial_pos[0] + 52, initial_pos[1], initial_pos[2], initial_pos[3], 0)
            enable_pump = 0
            dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
            dType.SetPTPCmd(api, 2, x, y, z, rHead, 0)

        elif color_counter[curr_color] == 1:
            dType.SetPTPCmd(api, 2, initial_pos[0] + 52, initial_pos[1] - 27, initial_pos[2], initial_pos[3], 0)
            enable_pump = 0
            dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
            dType.SetPTPCmd(api, 2, x, y, z, rHead, 0)

    elif curr_color == "Blue":
        if color_counter[curr_color] == 0:
            dType.SetPTPCmd(api, 2, initial_pos[0] + 52, initial_pos[1] - 52, initial_pos[2], initial_pos[3], 0)
            enable_pump = 0
            dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
            dType.SetPTPCmd(api, 2, x, y, z, rHead, 0)

        elif color_counter[curr_color] == 1:
            dType.SetPTPCmd(api, 2, initial_pos[0] + 52, initial_pos[1] - 27, initial_pos[2], initial_pos[3], 0)
            enable_pump = 0
            dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
            dType.SetPTPCmd(api, 2, x, y, z, rHead, 0)

    elif curr_color == "Yellow":
        if color_counter[curr_color] == 0:
            dType.SetPTPCmd(api, 2, initial_pos[0], initial_pos[1] - 52, initial_pos[2], initial_pos[3], 0)
            enable_pump = 0
            dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
            dType.SetPTPCmd(api, 2, x, y, z, rHead, 0)

        elif color_counter[curr_color] == 1:
            dType.SetPTPCmd(api, 2, initial_pos[0], initial_pos[1] - 27, initial_pos[2], initial_pos[3], 0)
            enable_pump = 0
            dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
            dType.SetPTPCmd(api, 2, x, y, z, rHead, 0)
    return


# SETUP------
dType.SetHOMEParams(api, x,  y,  z,  rHead)
dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200)
dType.SetPTPCommonParams(api, 100, 100)

isEnable = 1

infraredPort = 2  # plug IR sensor into GP4 (port 2)

dType.SetInfraredSensor(api, isEnable, infraredPort, version=0)

counter = 0
while counter < 20:
    # Get the current status of the photocell sensor
    IRSensor = dType.GetInfraredSensor(api, infraredPort)
    # Print the current status of the IR sensor
    if IRSensor[0] == 0:
        print("The IR sensor is OFF")  # no object
        dType.SetEMotor(api, 0, 1, 4000)
    else:
        print("The IR sensor is ON")  # detects an object
        dType.SetEMotor(api, 0, 0, 0)
        
        # Number of consecutive detections required
        detectionThreshold = 10
        # Amount of time to wait after last detection
        timeElapsedAfterDetection = 5  # Adjust as needed
        cap = cv2.VideoCapture(1)
        detection_time = None
        color = None
        color_counter = {color: 0 for color in ["Red", "Blue", "Green", "Yellow"]}  # Initialize color counters

        while True:
            ret, frame = cap.read()
            if ret:
                color = detect_color(frame)
                if color:
                    print("Color Detected: ", color)
                    detection_time = time.time()  # Record time of detection

                    # Increment the counter for the detected color
                    color_counter[color] += 1

                    # Check if the counter for the detected color reaches the threshold
                    if color_counter[color] >= detectionThreshold:
                        break  # Exit the loop if the threshold is reached

                # Display the frame
                cv2.imshow('Frame', frame)

            # Check if a color was detected and if enough time has passed
            if detection_time and time.time() - detection_time >= timeElapsedAfterDetection:
                break

            # Check for key press to exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

        if color:
            if color == "Red":
                dType.SetEMotor(api, 0, 0, 0)
                dType.SetPTPCmd(api, 2, pickUpSpot[0], pickUpSpot[1], pickUpSpot[2], pickUpSpot[3], 0)

                enable_pump = 1
                dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
                dType.dSleep(2000)

                if color_counter[color] > block_threshold:
                    print("Placing block in dead zone...")
                    dType.SetPTPCmd(api, 2, deadZoneSpot[0], deadZoneSpot[1], deadZoneSpot[2], deadZoneSpot[3], 0)
                    enable_pump = 0
                    dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
                    dType.SetPTPCmd(api, 2, x, y, z, rHead, 0)
                else:

                    # TODO - Set Proper set-down spots for RED blocks!
                    placeBlock(initialShapeStart, color)

                block_counter[color] += 1

            elif color == "Green":
                dType.SetEMotor(api, 0, 0, 0)
                dType.SetPTPCmd(api, 2, pickUpSpot[0], pickUpSpot[1], pickUpSpot[2], pickUpSpot[3], 0)

                enable_pump = 1
                dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
                dType.dSleep(2000)

                if color_counter[color] > block_threshold:
                    print("Placing block in dead zone...")
                    dType.SetPTPCmd(api, 2, deadZoneSpot[0], deadZoneSpot[1], deadZoneSpot[2], deadZoneSpot[3], 0)
                    enable_pump = 0
                    dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
                    dType.SetPTPCmd(api, 2, x, y, z, rHead, 0)
                else:

                    # TODO - Set Proper set-down spots for GREEN blocks!
                    placeBlock(initialShapeStart, color)

                block_counter[color] += 1

            elif color == "Blue":
                dType.SetEMotor(api, 0, 0, 0)
                dType.SetPTPCmd(api, 2, pickUpSpot[0], pickUpSpot[1], pickUpSpot[2], pickUpSpot[3], 0)

                enable_pump = 1
                dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
                dType.dSleep(2000)

                if color_counter[color] > block_threshold:
                    print("Placing block in dead zone...")
                    dType.SetPTPCmd(api, 2, deadZoneSpot[0], deadZoneSpot[1], deadZoneSpot[2], deadZoneSpot[3], 0)
                    enable_pump = 0
                    dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
                    dType.SetPTPCmd(api, 2, x, y, z, rHead, 0)
                else:

                    # TODO - Set Proper set-down spots for BLUE blocks!
                    placeBlock(initialShapeStart, color)

                block_counter[color] += 1

            elif color == "Yellow":
                dType.SetEMotor(api, 0, 0, 0)
                dType.SetPTPCmd(api, 2, pickUpSpot[0], pickUpSpot[1], pickUpSpot[2], pickUpSpot[3], 0)

                enable_pump = 1
                dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
                dType.dSleep(2000)

                if color_counter[color] > block_threshold:
                    print("Placing block in dead zone...")
                    dType.SetPTPCmd(api, 2, deadZoneSpot[0], deadZoneSpot[1], deadZoneSpot[2], deadZoneSpot[3], 0)
                    enable_pump = 0
                    dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
                    dType.SetPTPCmd(api, 2, x, y, z, rHead, 0)
                else:
                    # TODO - Set Proper set-down spots for YELLOW blocks!
                    placeBlock(initialShapeStart, color)

                block_counter[color] += 1

        else:
            print("No Specific Color Detected!")
            dType.SetEMotor(api, 0, 0, 0)
            dType.SetPTPCmd(api, 2, pickUpSpot[0], pickUpSpot[1], pickUpSpot[2], pickUpSpot[3], 0)
            enable_pump = 1
            dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
            dType.dSleep(2000)

            dType.SetPTPCmd(api, 2, deadZoneSpot[0], deadZoneSpot[1], deadZoneSpot[2], deadZoneSpot[3], 0)
            enable_pump = 0
            dType.SetEndEffectorSuctionCup(api, suction_cup, enable_pump, ctrl_mode)
            dType.SetPTPCmd(api, 2, x, y, z, rHead, 0)

        counter += 1
        print("counter ", counter)
        dType.dSleep(2000)
