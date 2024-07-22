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
        rospy.init_node('obstacle_avoidance_node', anonymous=True)
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.map_data = None
        self.proximity_to_goal = 2.0 # weight associated with the proximity to the goal.... needs to be tuned

    def map_callback(self, msg):
        """
        Callback function for the /map topic subscriber.
        """
        
        # Convert the OccupancyGrid data to a numpy array
        map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_data = np.flipud(map_data)
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        

        # Proceed with visualization
        # self.visualize_map(map_data)
        
        # self.draw_robot_fov(fov_length=95, fov_width=5)


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
        cv2.waitKey(100)

    

    def compute_steering_angle(self, heading, goal, fov_length=50, fov_width=20): 

        """ This method is used different steering angles to choose the best steering angle
        
        Input
        heading = The current heading direction of the robot [rad] in the world frame
        fov_length = number of pixels that determines how far the robot can see
        fov_width = number of pixels that determines how wide the robot can see
        goal = goal position [numpy array np.array([x,y])] in the world frame

        Output
        This method returns the best steering angle in rad
        """

        robot_position = (0,0)
        costs = {}
    
        # Start at 0 degrees, end at 2pi rad, step by 10 degrees
        for angle in np.linspace(-0.5*np.pi, 0.5*np.pi, num=int(180/45)):
            costs[angle]=float('inf')
            obs = self.check_obstacle(heading + angle, fov_length, fov_width)
            if not obs:
                predicted_position = self.predict_position(heading + angle, robot_position, velocity=1, dt=2)
                distance = np.linalg.norm(predicted_position - goal)
                # Compute the overall cost
                cost = self.proximity_to_goal * distance 
                costs[angle]= cost

        # choose the angle with the minimum cost
        best_angle = min(costs, key=lambda angle: costs[angle])

        if 0 < best_angle < np.pi:
            print("steer to the LEFT")
            return 0.5*np.pi
        else:
            print("steer to the RIGHT")
            return -0.5*np.pi

                
    def predict_position( self, theta, robot_position, velocity,dt=2):
        
        '''
        Method returning the predicted position of the robot in dt=2sec time step
        '''
        
        x, y = robot_position
        

        # Predict the next state based on velocity
        
        x += velocity * np.cos(theta)*dt
        y += velocity * np.sin(theta) * dt

        return np.array([x,y])
            
        

    def check_obstacle(self, heading_rad, length, width):
        """
        A method that extracts the pixels in the FOV of the robot and Checks if there is an obstacle in its FOV

        Inputs
        heading_rad = it expects angle with respect to the positive x-axis of the world frame [in radians]
        length = number of pixels in the direction of heading of the robot. It determines how far the robot can see.
        width =  number of pixels perpendicular to the direction of heading of the robot. It determines how wide the robot can see.

        Output
        It returns True if there is obstacle in the FOV of the robot, False otherwise

        """
        if self.map_data is None:
            rospy.loginfo("Map data is not available for FOV visualization.")
            return False
        
        # Create a single-channel image for visualization
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
        rotated_corners = [(x * np.cos(-heading_rad) - y * np.sin(-heading_rad),
                            x * np.sin(-heading_rad) + y * np.cos(-heading_rad)) for x, y in corners]
        
        # Shift corners to the robot's position (if not at origin)
        x=0 # robot x position
        y=0 # robot y position
    
        # current_robot_position = (x + 0*self.map_origin[0], -y + 0*self.map_origin[1]) # transforming from the world frame to the image frame on the top left corner
        robot_position = self.robot_to_map_coordinates((x,y))  
        
        shifted_corners = [(x + robot_position[0], y + robot_position[1]) for x, y in rotated_corners]

        # Draw the FOV on the map
        cv2.polylines(map_with_fov, [np.array(shifted_corners, dtype=np.int32)], isClosed=True, color=(200), thickness=1)

        # Create a mask for the FOV to extract the pixels in the FOV of the robot
        fov_mask = np.zeros_like(map_with_fov, dtype=np.uint8)
        cv2.fillPoly(fov_mask, [np.array(shifted_corners, dtype=np.int32)], 255)
        
        
        # Extract pixels within the FOV
        fov_pixels[fov_mask == 255] = map_with_fov[fov_mask == 255]

        
        # Display the FOV mask
        cv2.imshow("FOV Mask", fov_pixels)
        cv2.waitKey(100)  # Wait indefinitely until a key is pressed
    

        
        # Visualize the updated map
        self.visualize_map(map_with_fov)
        # self.visualize_map(fov_pixels)
        if np.any(fov_pixels == 255):
            print("Obstacle detected in the FOV.")
            return True
        else:
            print("No obstacle in the FOV.")
            return False




    def robot_to_map_coordinates(self, robot_position):
        # Convert robot position to map coordinates
        if self.map_resolution is None:
            return 0, 0  # Default value if map resolution is not available yet

        map_x = int((robot_position[0] + abs(self.map_origin[0])) / self.map_resolution)
        map_y = int((-robot_position[1] + abs(self.map_origin[1])) / self.map_resolution)
        return map_x, map_y




    def main(self):
        """
        The main run loop of the node.
        """
        
        goal = np.array([-4,4])
        heading = np.deg2rad(90) # to be taken from the robot velocity
        robot_velocity = [np.cos(heading), np.sin(heading)]
        
        while not rospy.is_shutdown():
            if self.map_data is not None:
                if self.check_obstacle(heading, length=95, width=5):
                    angle = self.compute_steering_angle(heading, goal, fov_length=20, fov_width=1)
                    steering_correction = Vector2(robot_velocity[0], robot_velocity[1])
                    steering_correction.rotate(180)
                    perpendicular_vec = steering_correction
                    perpendicular_vec.rotate(angle)
                    result = steering_correction.__add__(perpendicular_vec)
                    return result.normalize()
        
        return
        
    def run(self):

        
        self.main()
        

        rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass
