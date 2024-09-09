import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
import queue
import subprocess

# Nodes & Topics
NODE_NAME = 'lane_detection_node'
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
CENTROID_TOPIC_NAME = '/centroid'
blackAndWhiteImage_TOPIC = '/blackAndWhiteImage'

class LaneDetection(Node):

    # Initialization Method
    def __init__(self):
        super().__init__(NODE_NAME)
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=1)
        self.result_queue = queue.Queue()
        self.camera_subscriber = self.create_subscription(
            Image, CAMERA_TOPIC_NAME, self.camera_callback, 10)
        self.centroid_error_publisher = self.create_publisher(
            Float32, CENTROID_TOPIC_NAME, 10)
        self.blackAndWhiteImage_publisher = self.create_publisher(
            Image, blackAndWhiteImage_TOPIC, 10)
        
        self.initialize_parameters()
    

        # New Initializations for Threading
        self.processing_thread = threading.Thread(target=self.process_images)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        self.detection_thread = threading.Thread(target=self.detect_lanes)
        self.detection_thread.daemon = True
        self.detection_thread.start()


    # New Method to Initialize Parameters for Organization
    def initialize_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Hue_low', 1),
                ('Hue_high', 1),
                ('Saturation_low', 1),
                ('Saturation_high', 1),
                ('Value_low', 1),
                ('Value_high', 1),
                ('gray_lower', 1),
                ('inverted_filter', 0),
                ('kernal_size',1),
                ('erosion_itterations',1),
                ('dilation_itterations',1),
                ('number_of_lines', 0),
                ('error_threshold', 0),
                ('Width_min', 1),
                ('Width_max', 1),
                ('crop_width_decimal',0.8),
                ('rows_to_watch_decimal',0.2),
                ('rows_offset_decimal',0.5),
                ('camera_centerline',0.5),
                ('debug_cv', 0)
            ])

        self.Hue_low_yellow = 19
        self.Hue_high_yellow = 63
        self.Saturation_low_yellow = 51 
        self.Saturation_high_yellow = 255
        self.Value_low_yellow = 160
        self.Value_high_yellow = 255

        self.Hue_low_red = 150
        self.Hue_high_red = 179
        self.Saturation_low_red = 81 
        self.Saturation_high_red = 255
        self.Value_low_red = 46
        self.Value_high_red = 255

        self.Hue_low_green = 79
        self.Hue_high_green = 88
        self.Saturation_low_green = 65
        self.Saturation_high_green = 255
        self.Value_low_green = 78
        self.Value_high_green = 255

        self.Hue_low_blue = 94
        self.Hue_high_blue = 117
        self.Saturation_low_blue = 90 
        self.Saturation_high_blue = 255
        self.Value_low_blue = 50
        self.Value_high_blue = 255

        self.gray_lower = self.get_parameter('gray_lower').value
        self.inverted_filter = self.get_parameter('inverted_filter').value
        self.kernal_size = self.get_parameter('kernal_size').value
        self.erosion_itterations = self.get_parameter('erosion_itterations').value
        self.dilation_itterations = self.get_parameter('dilation_itterations').value
        self.number_of_lines = self.get_parameter('number_of_lines').value
        self.error_threshold = self.get_parameter('error_threshold').value
        self.min_width = self.get_parameter('Width_min').value
        self.max_width = self.get_parameter('Width_max').value
        self.crop_width_decimal = self.get_parameter('crop_width_decimal').value
        self.rows_to_watch_decimal = self.get_parameter('rows_to_watch_decimal').value
        self.rows_offset_decimal = self.get_parameter('rows_offset_decimal').value
        self.camera_centerline = self.get_parameter('camera_centerline').value
        self.debug_cv = self.get_parameter('debug_cv').value
        
        self.camera_init = False

        self.get_logger().info(
            f'\nHue_low: {self.Hue_low_yellow}'
            f'\nHue_high: {self.Hue_high_yellow}'
            f'\nSaturation_low: {self.Saturation_low_yellow}'
            f'\nSaturation_high: {self.Saturation_high_yellow}'
            f'\nValue_low: {self.Value_low_yellow}'
            f'\nValue_high: {self.Value_high_yellow}'
            f'\ngray_lower: {self.gray_lower}'
            f'\ninverted_filter: {self.inverted_filter}'
            f'\nkernal_size: {self.kernal_size}'
            f'\nerosion_itterations: {self.erosion_itterations}'
            f'\ndilation_itterations: {self.dilation_itterations}'
            f'\nnumber_of_lines: {self.number_of_lines}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nmin_width: {self.min_width}'
            f'\nmax_width: {self.max_width}'
            f'\ncrop_width_decimal: {self.crop_width_decimal}'
            f'\nrows_to_watch_decimal: {self.rows_to_watch_decimal}'
            f'\nrows_offset_decimal: {self.rows_offset_decimal}'
            f'\ncamera_centerline: {self.camera_centerline}'
            f'\ndebug_cv: {self.debug_cv}') 
        
    
    # New Method replacing original locate_centroid 
    # method as the callback for the camera subscription
    def camera_callback(self, data):
        try:
            self.image_queue.put(data, block=False)
        except queue.Full:
            # If queue is full, remove the oldest item and add the new one
            self.image_queue.get()
            self.image_queue.put(data)


    # New Method to Perform Image Processing
    def process_images(self):
        while rclpy.ok():
            try:
                data = self.image_queue.get(timeout=1.0)
                frame = self.bridge.imgmsg_to_cv2(data)
  
                # Perform image processing here
                processed_images = self.image_processing(frame)
            
                self.result_queue.put(processed_images)
            except queue.Empty:
                continue


    # Image processing logic of original locate_centroid method moved here
    def image_processing(self, frame):
        
        # Camera initialization & image cropping
        if not self.camera_init:
            self.get_logger().info(f'\n Initializing Camera...')
            height, width, channels = frame.shape
        
            # Vertical crop/pan
            rows_to_watch = int(height * self.rows_to_watch_decimal)
            rows_offset = int(height * (1 - self.rows_offset_decimal))

            # Horizontal crop
            self.start_height = int(height - rows_offset)
            self.bottom_height = int(self.start_height + rows_to_watch)
            self.left_width = int((width / 2) * (1 - self.crop_width_decimal))
            self.right_width = int((width / 2) * (1 + self.crop_width_decimal))
            self.camera_init = True
            self.get_logger().info(f'\n Camera Initialized')

        self.image_width = int(self.right_width - self.left_width)
        self.image_height = self.bottom_height-self.start_height
        img = frame[self.start_height:self.bottom_height, self.left_width:self.right_width]
        img_red = frame[self.start_height:self.bottom_height, self.left_width:self.right_width]
        img_green = frame[self.start_height:self.bottom_height, self.left_width:self.right_width]
        img_blue = frame[self.start_height:self.bottom_height, self.left_width:self.right_width]

        # Converting color space to HSV, Masking, & Filtering
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([self.Hue_low_yellow, self.Saturation_low_yellow, self.Value_low_yellow])
        upper_yellow = np.array([self.Hue_high_yellow, self.Saturation_high_yellow, self.Value_high_yellow])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        hsv_red = cv2.cvtColor(img_red, cv2.COLOR_BGR2HSV)
        lower_red = np.array([self.Hue_low_red, self.Saturation_low_red, self.Value_low_red])
        upper_red = np.array([self.Hue_high_red, self.Saturation_high_red, self.Value_high_red])
        mask_red = cv2.inRange(hsv_red, lower_red, upper_red)

        hsv_green = cv2.cvtColor(img_green, cv2.COLOR_BGR2HSV)
        lower_green = np.array([self.Hue_low_green, self.Saturation_low_green, self.Value_low_green])
        upper_green = np.array([self.Hue_high_green, self.Saturation_high_green, self.Value_high_green])
        mask_green = cv2.inRange(hsv_green, lower_green, upper_green)

        hsv_blue = cv2.cvtColor(img_blue, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([self.Hue_low_blue, self.Saturation_low_blue, self.Value_low_blue])
        upper_blue = np.array([self.Hue_high_blue, self.Saturation_high_blue, self.Value_high_blue])
        mask_blue = cv2.inRange(hsv_blue, lower_blue, upper_blue)
        
        if self.inverted_filter == 1:
            bitwise_mask = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_not(mask))
            bitwise_mask_red = cv2.bitwise_and(hsv_red, hsv_red, mask=cv2.bitwise_not(mask_red))
            bitwise_mask_green = cv2.bitwise_and(hsv_green, hsv_green, mask=cv2.bitwise_not(mask_green))
            bitwise_mask_blue = cv2.bitwise_and(hsv_blue, hsv_blue, mask=cv2.bitwise_not(mask_blue))
        else:
            bitwise_mask = cv2.bitwise_and(hsv, hsv, mask=mask)
            bitwise_mask_red = cv2.bitwise_and(hsv_red, hsv_red, mask=mask_red)
            bitwise_mask_green = cv2.bitwise_and(hsv_green, hsv_green, mask=mask_green)
            bitwise_mask_blue = cv2.bitwise_and(hsv_blue, hsv_blue, mask=mask_blue)

        # Changing to gray color space
        gray = cv2.cvtColor(bitwise_mask, cv2.COLOR_BGR2GRAY)
        gray_red = cv2.cvtColor(bitwise_mask_red, cv2.COLOR_BGR2GRAY)
        gray_green = cv2.cvtColor(bitwise_mask_green, cv2.COLOR_BGR2GRAY)
        gray_blue = cv2.cvtColor(bitwise_mask_blue, cv2.COLOR_BGR2GRAY)

        # Changing to black and white color space
        gray_upper = 255
        (dummy, blackAndWhiteImage) = cv2.threshold(gray, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        (dummy, blackAndWhiteImage_red) = cv2.threshold(gray_red, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        (dummy, blackAndWhiteImage_green) = cv2.threshold(gray_green, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        (dummy, blackAndWhiteImage_blue) = cv2.threshold(gray_blue, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        
        # Get rid of white noise from grass
        kernel = np.ones((self.kernal_size, self.kernal_size), np.uint8)
        blurred = cv2.blur(blackAndWhiteImage,(self.kernal_size, self.kernal_size))
        erosion = cv2.erode(blurred, kernel, iterations = self.erosion_itterations)
        dilation = cv2.dilate(erosion, kernel, iterations = self.dilation_itterations)
        (dummy, blackAndWhiteImage) = cv2.threshold(dilation, self.gray_lower, gray_upper, cv2.THRESH_BINARY)

        blurred_red = cv2.blur(blackAndWhiteImage_red,(self.kernal_size, self.kernal_size))
        erosion_red = cv2.erode(blurred_red, kernel, iterations = self.erosion_itterations)
        dilation_red = cv2.dilate(erosion_red, kernel, iterations = self.dilation_itterations)
        (dummy, blackAndWhiteImage_red) = cv2.threshold(dilation_red, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        contours_red, dummy = cv2.findContours(blackAndWhiteImage_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        blurred_green = cv2.blur(blackAndWhiteImage_green,(self.kernal_size, self.kernal_size))
        erosion_green = cv2.erode(blurred_green, kernel, iterations = self.erosion_itterations)
        dilation_green = cv2.dilate(erosion_green, kernel, iterations = self.dilation_itterations)
        (dummy, blackAndWhiteImage_green) = cv2.threshold(dilation_green, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        contours_green, dummy = cv2.findContours(blackAndWhiteImage_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        blurred_blue = cv2.blur(blackAndWhiteImage_blue,(self.kernal_size, self.kernal_size))
        erosion_blue = cv2.erode(blurred_blue, kernel, iterations = self.erosion_itterations)
        dilation_blue = cv2.dilate(erosion_blue, kernel, iterations = self.dilation_itterations)
        (dummy, blackAndWhiteImage_blue) = cv2.threshold(dilation_blue, self.gray_lower, gray_upper, cv2.THRESH_BINARY)
        contours_blue, dummy = cv2.findContours(blackAndWhiteImage_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        processed_images = {
        'main': blackAndWhiteImage,
        'color': {
            'red': blackAndWhiteImage_red,
            'green': blackAndWhiteImage_green,
            'blue': blackAndWhiteImage_blue
        }
    }
        return processed_images
    

    # New Method to Perform Lane Detection
    def detect_lanes(self):
        while rclpy.ok():
            try:
                processed_images = self.result_queue.get(timeout=1.0)
                
                # Perform lane detection on the main image
                lanes, error = self.lane_detection(processed_images['main'])
                
                # Publish results
                self.publish_results(lanes, error)
                
                # Check for color markers and play sounds
                self.check_color_markers(processed_images['color'])
            except queue.Empty:
                continue

    
    # Lane detection logic of original locate_centroid method moved here
    def lane_detection(self, blackAndWhiteImage):
        contours, dummy = cv2.findContours(blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        # Defining points of a line to be drawn for visualizing error
        cam_center_line_x = int(self.image_width * self.camera_centerline)
        start_point = (cam_center_line_x,0)
        end_point = (cam_center_line_x, int(self.bottom_height))
        
        start_point_thresh_pos_x = int(cam_center_line_x -  (self.error_threshold * self.image_width/2))
        start_point_thresh_neg_x = int(cam_center_line_x + (self.error_threshold * self.image_width/2))
        
        start_point_thresh_pos = (start_point_thresh_pos_x, 0)
        end_point_thresh_pos = (start_point_thresh_pos_x, int(self.bottom_height))

        start_point_thresh_neg = (start_point_thresh_neg_x, 0)
        end_point_thresh_neg = (start_point_thresh_neg_x, int(self.bottom_height))

        # Setting up data arrays
        cx_list = []
        cy_list = []

        # Plotting contours and their centroids
        for contour in contours[:self.number_of_lines]:
            [x, y], [w, h], phi = cv2.minAreaRect(contour)
            rect = cv2.minAreaRect(contour)
            if self.min_width < w < self.max_width:
                try:
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    img = cv2.drawContours(img,[box], 0, (0, 255, 0), 3)
                    m = cv2.moments(contour)
                    cx = int(m['m10'] / m['m00'])
                    cy = int(m['m01'] / m['m00'])
                    cx_list.append(cx)
                    cy_list.append(cy)
                    cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)
                    img = cv2.line(img, start_point, end_point, (0,255,0), 4)
                    img = cv2.line(img, start_point_thresh_pos, end_point_thresh_pos, (0,0,255), 2)
                    img = cv2.line(img, start_point_thresh_neg, end_point_thresh_neg, (0,0,255), 2)
                except ZeroDivisionError:
                    pass

        # Further image processing to determine optimal steering value
        try:
            # When more than 1 road mark is found
            if len(cx_list) > 1:
                error_list = []
                count = 0

                # Calculate errors for all detected road lines
                for cx_pos in cx_list:
                    error = float((cx_pos - cam_center_line_x) / cam_center_line_x)
                    error_list.append(error)
                
                # Finding average error of all road lines
                avg_error = (sum(error_list) / float(len(error_list)))

                # Check difference in error from closest to furthest road line
                p_horizon_diff = abs(error_list[0] - error_list[-1])

                # If path is approximately straight, then steer towards average error
                if abs(p_horizon_diff) <= self.error_threshold:
                    error_x = avg_error
                    pixel_error = int(cam_center_line_x * (1 + error_x))
                    mid_x, mid_y = pixel_error, int((self.image_height/2))
                    self.get_logger().info(f"Straight curve: [tracking error: {error_x}], [tracking angle: {phi}]")

                # if path is curved, then steer towards minimum error
                else: 
                    # Exclude any road lines within error threshold by making their error large
                    for error in error_list:
                        if abs(error) < self.error_threshold:
                            error = 1
                            error_list[count] = error
                        count+=1
                    
                    # Getting min error (closest roadline)
                    error_x = min(error_list, key=abs)

                    # Get index of min error for plotting
                    error_x_index = error_list.index(min(error_list, key=abs))
                    mid_x, mid_y = cx_list[error_x_index], cy_list[error_x_index]
                    self.get_logger().info(f"Curvy road: [tracking error: {error_x}], [tracking angle: {phi}]")
                
                # Plotting roadline to be tracked
                cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
                start_point_error = (cam_center_line_x, mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)

                # Publish error data
                self.centroid_error.data = float(error_x)
                self.centroid_error_publisher.publish(self.centroid_error)

            # When only 1 road mark was found 
            elif len(cx_list) == 1:
                mid_x, mid_y = cx_list[0], cy_list[0]
                error_x = float((mid_x - cam_center_line_x) / cam_center_line_x)
                start_point_error = (cam_center_line_x, mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)
                cv2.circle(img, (mid_x, mid_y), 7, (0, 0, 255), -1)
                
                self.centroid_error.data = error_x
                self.centroid_error_publisher.publish(self.centroid_error)
                self.get_logger().info(f"Only detected one line: [tracking error: {error_x}], [tracking angle: {phi}]")

            # When Nothing was found
            else:
                self.get_logger().info(f"Nothing detected")

            # clean slate
            error_list = [0] * self.number_of_lines
            cx_list = []
            cy_list = []
        except ValueError:
            pass

        # plotting results
        self.debug_cv = self.get_parameter('debug_cv').value # ability to update debug in real-time
        if self.debug_cv:
            cv2.imshow('img', img)
            cv2.imshow('blackAndWhiteImage', blackAndWhiteImage)
            cv2.waitKey(1)
        else:
            cv2.destroyAllWindows()


    # New Method to Check for RGB in 
    # processed images and play audio files
    def check_color_markers(self, color_images):
        # Check for red, green, and blue markers
        # Play corresponding sounds if detected
        if cv2.countNonZero(color_images['red']) > 0:
            subprocess.run(['play', '/home/projects/sounds/c4.mp3'])
        if cv2.countNonZero(color_images['green']) > 0:
            subprocess.run(['play', '/home/projects/sounds/d4.mp3'])
        if cv2.countNonZero(color_images['blue']) > 0:
            subprocess.run(['play', '/home/projects/sounds/e4.mp3'])


# Updated Main Function from locate_centroid to lane_detection_node
def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = LaneDetection()
    try:
        rclpy.spin(lane_detection_node)
        lane_detection_node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        lane_detection_node.get_logger().info(f'Shutting down {NODE_NAME}...')
    finally:
        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        lane_detection_node.destroy_node()
        rclpy.shutdown()
        lane_detection_node.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()