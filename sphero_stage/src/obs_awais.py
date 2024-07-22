#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from helper_function.utils import Vector2

class ObstacleAvoidance:
    def __init__(self):
        """
        Initialize the ObstacleAvoidance class.
        """
        # rospy.init_node('obstacle_avoidance_node', anonymous=True)
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.map_data = None
        self.fov_length = 20
        self.fov_width = 5
        self.fov_angle = 180 # angle of field of view of the robot in degrees
        self.wall_follow = False

    def map_callback(self, msg):
        """
        Callback function for the /map topic subscriber.
        """
        
        # Convert the OccupancyGrid data to a numpy array
        map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_data = map_data 
        
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        # self.map_data[map_data == 0] = 255  # Free cells
        # self.map_data[map_data == 100] = 0  # Occupied cells
        self.map_data = np.flipud(self.map_data)
        
        

        


    def visualize_map(self, map_data):
        """
        Visualize the map using OpenCV.
        """
        # Create an empty image for visualization
        display_map = np.zeros(map_data.shape, dtype=np.uint8)

        # Set free cells (value 0) to white (255) and occupied cells (value 100) to black (0)
        display_map[map_data == 0] = 255  # Free cells
        display_map[map_data == 100] = 0  # Occupied cells

        # Resize for better visualization
        resized_map = cv2.resize(display_map, (800, 800), interpolation=cv2.INTER_NEAREST)

        # Display the map
        cv2.imshow("Map Visualization", resized_map)
        cv2.waitKey(5)

    

    def compute_steering_angle(self, heading, goal, fov_length, fov_width, robot_position, velocity): 

        """ This method is analyses different steering angles to choose the best steering angle
        
        Input
        heading = The current heading direction of the robot [rad] in the world frame
        fov_length = number of pixels that determines how far the robot can see
        fov_width = number of pixels that determines how wide the robot can see
        goal = goal position [numpy array np.array([x,y])] in the world frame
        robot_position = np.array([x,y]), robot position in the world frame

        Output
        This method returns the best steering angle in rad
        """

        
        costs = {}
        fov_angle = np.deg2rad(self.fov_angle)
        # analyse the neighbourhood with angle resolution of 10 degrees --> num=int(self.fov_angle/10)
        
        for angle in np.linspace(-0.5*fov_angle, 0.5*fov_angle, num=int(self.fov_angle/10)):
            
            costs[angle]=float('inf')
            obs = self.check_obstacle(heading + angle, fov_length, fov_width, robot_position)
            if not obs:
                predicted_position = self.predict_position(heading + angle, robot_position, velocity, dt=2)
                distance = np.linalg.norm(predicted_position - goal)
                # Compute the overall cost  
                costs[angle]= distance
        

        # choose the angle with the minimum cost
        best_angle = min(costs, key=lambda angle: costs[angle])
        
        # return best_angle

        if 0 < best_angle < np.pi:
            # print("steer to the LEFT")
            return 0.5*np.pi
        else:
            # print("steer to the RIGHT")
            return -0.5*np.pi

                
    def predict_position( self, theta, robot_position, velocity,dt=2):
        
        '''
        Method returning the predicted position of the robot in dt=2sec time step
        '''
        
        x = robot_position[0]
        y = robot_position[1]
        

        # Predict the next state based on velocity
        
        x += velocity * np.cos(theta)*dt
        y += velocity * np.sin(theta) * dt

        return np.array([x,y])
            
        

    def check_obstacle(self, heading_rad, length, width, position):
        """
        A method that extracts the pixels in the FOV of the robot and Checks if there is an obstacle in its FOV

        Inputs
        heading_rad = it expects angle with respect to the positive x-axis of the world frame [in radians]
        length = number of pixels in the direction of heading of the robot. It determines how far the robot can see.
        width =  number of pixels perpendicular to the direction of heading of the robot. It determines how wide the robot can see.
        position = np.array([x,y]), position of the robot in the world frame

        Output
        It returns True if there is obstacle in the FOV of the robot, False otherwise

        """
        if self.map_data is None:
            rospy.loginfo("Map data is not available for FOV visualization.")
            return False
        
        # print("heading_rad", np.rad2deg(heading_rad))

        # Create a single-channel image
        map_with_fov = np.zeros((self.map_data.shape[0], self.map_data.shape[1]), dtype=np.uint8)
        fov_pixels   = np.zeros((self.map_data.shape[0], self.map_data.shape[1]), dtype=np.uint8)

        # Set occupied cells to white (255) and free cells to black (0)
        map_with_fov[self.map_data == 0] = 0    # Free cells
        map_with_fov[self.map_data == 100] = 255  # Occupied cells
        

        corners = [
            (0,-width),  # Left corner at robot position
            (0, width),  # Right corner at robot position
            (length, width),  # Right corner at FOV end
            (length, -width)  # Left corner at FOV end
        ]

       
        # Rotate corners by the robot's heading
        rotated_corners = [(x * np.cos(heading_rad) - y * np.sin(heading_rad),
                            x * np.sin(heading_rad) + y * np.cos(heading_rad)) for x, y in corners]
        
        # Shift corners to the robot's position (if not at origin)    
        robot_position = self.robot_to_map_coordinates((position[0],position[1]))  
        
        shifted_corners = [(x + robot_position[1], y + robot_position[0]) for x, y in rotated_corners]

        # for i in range(len(shifted_corners)):
        #     temp = self.robot_to_map_coordinates(shifted_corners[i])
        #     shifted_corners[i]= (temp[1],temp[0])

        # Draw the FOV on the map
        cv2.polylines(map_with_fov, [np.array(shifted_corners, dtype=np.int32)], isClosed=True, color=(200), thickness=1)

        # Create a mask for the FOV to extract the pixels in the FOV of the robot
        fov_mask = np.zeros_like(map_with_fov, dtype=np.uint8)
        cv2.fillPoly(fov_mask, [np.array(shifted_corners, dtype=np.int32)], 255)
        
        
        # Extract pixels within the FOV
        fov_pixels[fov_mask == 255] = map_with_fov[fov_mask == 255]

        # # Resize for better visualization
        # resized_map = cv2.resize(fov_pixels, (800, 800), interpolation=cv2.INTER_NEAREST)
        # fov_pixels =  resized_map
        # # Display the FOV mask
        # cv2.imshow("FOV Mask", fov_pixels)
        # cv2.waitKey(1)  # Wait indefinitely until a key is pressed
    

        
        # Visualize the updated map
        # self.visualize_map(map_with_fov)
        # self.visualize_map(fov_pixels)

        if np.any(fov_pixels == 255):
            # print("Obstacle detected in the FOV.")
            return True
        else:
            # print("No obstacle in the FOV.")
            return False




    def robot_to_map_coordinates(self, robot_position):
        # Convert robot position to map coordinates
        if self.map_resolution is None:
            return 0, 0  # Default value if map resolution is not available yet
        
        column = int((robot_position[0] - self.map_origin[0]) / self.map_resolution)
        raw = int((-robot_position[1] - self.map_origin[1]) / self.map_resolution)
        
        return raw, column




    def main(self, goal, vel, robot_position):
        """
        The main run loop of the node.

        Inputs
        goal = np.array([x_goal, y_goal])
        heading = heading angle (direction) of the robot in the world frame [rad]
        robot_position = np.array([x,y])... current position of the robot in the world frame

        Output
        If obstacle is detected in the FOV of the robot, a unit vector representing the prefered direction of steering is returned else Null vector is returned
        """
        heading = np.deg2rad(vel.arg())

        print("self.wall_follow", self.wall_follow)
        
        # robot_velocity = [0.5*np.cos(heading), 0.5*np.sin(heading)]
        
        # while not rospy.is_shutdown():
        if self.map_data is not None:
            if not self.wall_follow:
                # print("headingheading", np.rad2deg(heading))
                if self.check_obstacle(heading, self.fov_length, self.fov_width,robot_position):
                
                    self.angle = self.compute_steering_angle(heading, goal, self.fov_length + 45, self.fov_width, robot_position, vel.norm())
                    
                    self.wall_follow = True
                    steering_correction = Vector2(vel.x, vel.y)
            
                    perpendicular_vec = Vector2(np.cos(heading + self.angle), np.sin(heading + self.angle))
                
                    steering_correction.rotate(180)
                    # print("perpendicular_vecperpendicular_vec",perpendicular_vec.arg())
                    result = steering_correction.__add__(perpendicular_vec.__mul__(5))
                    # self.compute_slope_of_obstacle_boundary(robot_position, neighborhood_size=40)
            
                    return  result.normalize(ret= True)
            else:
                #check if the robot can see the goal
                g = Vector2(goal[0], goal[1])
                r = Vector2(robot_position[0], robot_position[1])
                g.__sub__(r)
                
                # if False and not self.check_obstacle(np.deg2rad(g.arg()) , int(g.norm() /self.map_resolution), self.fov_width,robot_position) or not self.check_obstacle(heading, self.fov_length, self.fov_width,robot_position) :
                    # self.wall_follow= False
                    # print("Finish wall following")
                
                return self.wall_following(robot_position, neighborhood_size=40)

        
        return Vector2()
    
    # def wall_following(self, heading, wall, robot_position):
    #     desired_distance = 20  # desired distance in pixels (cells) from the wall
    #     pos_x, pos_y = self.robot_to_map_coordinates(robot_position)

    #     x =   np.cos(-wall)*np.cos(heading) - np.sin(-wall)*np.sin(heading)
    #     y =   np.sin(-wall)*np.cos(heading) + np.cos(-wall)*np.sin(heading) 

    #     small = min(x, y)
        
    #     if small!=0:
    #         x= int(x/ small)
    #         y = int(y /small)
        
    #     for i in range(desired_distance):
            
    #         if self.map_data[pos_x+i*x, pos_y+i*y] == 100 and i < (desired_distance-1):
    #             perpendicular_vec = Vector2(np.cos(heading + 0.1*wall), np.sin(heading + 0.1*wall))
    #             perpendicular_vec.__mul__(1)
    #             return perpendicular_vec
    #         elif self.map_data[pos_x+i*x, pos_y+i*y] == 100 and i == (desired_distance-1):
    #             perpendicular_vec = Vector2(np.cos(heading - 0.1*wall), np.sin(heading - 0.1*wall))
    #             perpendicular_vec.__mul__(1)
    #             return perpendicular_vec
    #         else:
    #             heading_vec = Vector2(np.cos(heading), np.sin(heading ))
    #             heading_vec.__mul__(1)
    #             return heading_vec
            
    

    def wall_following(self, robot_position, neighborhood_size):
        
        # # Define the square neighborhood
        r,c = self.robot_to_map_coordinates(robot_position)
        half_size = neighborhood_size // 2
        neighborhood_area = self.map_data[r-half_size:r+half_size, c-half_size:c+half_size]
        
        # neighborhood_area = neighborhood_area.astype(np.uint8)
        # print("neighborhood_area",neighborhood_area[10:,c-2:c+2])
        # print("robot_position", robot_position)

        # # Edge detection
        # edges = cv2.Canny(neighborhood_area, 50, 150)

        # # Line detection using Hough Transform
        # lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=20, maxLineGap=10)

        # # Compute angles of detected lines
        # angles = []
        # if lines is not None:
        #     for line in lines:
        #         x1, y1, x2, y2 = line[0]
        #         angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi  # Convert to degrees
        #         angles.append(angle)
        # print("compute_slope_of_obstacle_boundary(self, image, robot_position, neighborhood_size)")
        # print(angles)
        # return angles

        # import cv2
        # import numpy as np
        import matplotlib
        matplotlib.use('Agg')  # Set the backend to 'Agg'
        from matplotlib import pyplot as plt
        

        # Load the image
        # image_path = '/content/Screenshot from 2023-12-07 16-18-23.png'
        # image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        # print("shape", image.shape)

        # Apply morphological gradient
        kernel = np.ones((3, 3), np.uint8)
        # img = image[20:180, 80:120]
        # img = image[200:430, 320:430]
        img = neighborhood_area
        # img = cv2.GaussianBlur(img.astype(np.uint8), (9, 9), 0)
        # print("img", img[:10,:10])
        gradient = cv2.morphologyEx(img.astype(np.uint8), cv2.MORPH_GRADIENT, kernel)
        # print("gradient",gradient)

        # Apply threshold to create a binary image
        _, binary_image = cv2.threshold(gradient, 10, 100, cv2.THRESH_BINARY)
        # print("binary_image",binary_image[10:,30:])

        # Find contours in the binary image
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print("contours", contours)

        # Draw the contours on a black background
        contour_image = np.zeros_like(self.map_data)
        cv2.drawContours(contour_image.astype(np.uint8), contours, -1, (255, 0, 0), 2)

        # Display the original image
        plt.subplot(121), plt.imshow(img, cmap='gray'), plt.title('Original Image')

        # Display the image with detected contours in red
        plt.subplot(122), plt.imshow(contour_image, cmap='gray'), plt.title('Image with Detected Contours')
        angles = np.array([])

        # Print the angles of each line with respect to the positive x-axis
        for i, contour in enumerate(contours):
            # Fit a line to the contour
            [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
            
            # Calculate the angle with respect to the positive x-axis
            angle_degrees = np.arctan2(vy, vx)
            
            angles = np.append(angles, angle_degrees)
            
            # print(f"Angle of Line {i + 1}: {angle_degrees} degrees")

        plt.savefig('/home/milli/Pictures/result_image.png')
        # self.save_cropped_region_image( robot_position, neighborhood_size)
        # print("The average angle is: ", np.mean(angles))
        angl = np.mean(angles)

        # if self.angle < 0 and angl > 0:
        #     angl=  angl - np.pi
        # elif self.angle > 0 and angl > 0:
        #     angl=  np.pi - angl

        print("wall following angle", np.rad2deg(angl))
        heading_vec= Vector2(np.cos(angl), np.sin(angl))
        return heading_vec.__mul__(1)


    def save_cropped_region_image(self, robot_position, neighborhood_size):

        save_path = '/home/milli/Pictures/cropped.png'
        import matplotlib
        matplotlib.use('Agg')  # Set the backend to 'Agg'
        from matplotlib import pyplot as plt
        # Extract the square neighborhood around the robot's position
        y, x = self.robot_to_map_coordinates(robot_position)
        half_size = neighborhood_size // 2
        neighborhood_area = self.map_data[y - half_size:y + half_size, x - half_size:x + half_size]
        

        # Create a copy of the map_data to draw the rectangle without modifying the original map_data
        map_with_rectangle = np.copy(self.map_data)

        # Draw a red rectangle over the cropped region
        # cv2.rectangle(map_with_rectangle, (y - half_size, x - half_size), (y + half_size, x + half_size), (255, 0, 0), 2)


        # Ensure that map_with_rectangle is a NumPy array
        map_with_rectangle = np.array(map_with_rectangle)

        # Draw a red rectangle on map_with_rectangle
        cv2.rectangle(map_with_rectangle, (int(y - half_size), int(x - half_size)), (int(y + half_size), int(x + half_size)), (255, 0, 0), 2)


        # Display the map_data with the overlaid rectangle
        plt.imshow(map_with_rectangle, cmap='gray')
        plt.title('Map Data with Cropped Region Highlighted')

        # Save the figure to the specified path
        plt.savefig(save_path)


        
    def run(self):

        
        # self.main()
        

        rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass
