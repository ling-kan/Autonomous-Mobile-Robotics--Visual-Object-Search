# -*- coding: utf-8 -*-
"""
Created on Tue Mar 20 21:14:04 2018

@author: KAN13489110 Ling Kan 
"""

# -*- coding: utf-8 -*-
#!/usr/bin/env python
import rospy    # Import rospy for the subscriber  
import cv2      # OpenCV functions
import numpy    # Used for working with sensor data, suppors arrays of data# change the image message to openCV2 image converter      
import actionlib
from cv_bridge import CvBridge                  # OpenCV ROS functions               
from sensor_msgs.msg import Image               # The message type of the image
from geometry_msgs.msg import Twist             # The cmd_vel message type
from std_msgs.msg import Float32 ,String        # Allows to send single float values
from sensor_msgs.msg import LaserScan           # Laser scan for range-finder
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #Link files and messages used by move base action
from nav_msgs.msg import Odometry               # Find the possition and velocity in free space 
 
 #http://marte.aslab.upm.es/redmine/files/dmsf/p_drone-testbed/170324115730_268_Quigley_-_Programming_Robots_with_ROS.pdf
class Search:        
    def __init__(self):
        self.bridge = CvBridge()                 # Converting ROS images into OpenCV images
        cv2.namedWindow('Image Window', 1)       # Creating a image window to be used as a placeholder for later
        cv2.namedWindow('Mask Window', 2)        # Creating a mask window to be used as a placeholder for later
        cv2.startWindowThread()                  # Tries to start a thread that updates the window automatically, and handles resizing
        
        self.minRange = 1
        
        # Subscribe to a topic
        self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', # Topic location
                                          Image,                             # Topic Data type
                                          self.callback)   
                                          # Gives a new message
        # Publishing data on that topic
        self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop', #Topic location
                                           Twist,                                 # Topic Data type
                                           queue_size=10)                         # Gets all the new messages not old 
                                           
        # Creating a subscriber listening to the laser scans
        self.laser_sub = rospy.Subscriber( '/turtlebot/scan',               # Topic should list to    
                                            LaserScan,                      # Topic Data type
                                            self.laser,                     # Callback function triggers new message arrivedr
                                            queue_size=10)                  # Gets all the new messages not old
                # All the colour points aimed at false.
        # Therefore once false it will be marked as true to stop searching for that specifc colour
        self.rPoint = False     
        self.bPoint = False
        self.gPoint = False
        self.yPoint = False
    
        #Stop at this distance
        self.minDistance = 1.5                          # Minimum Distance to stop       
        self.twist = Twist()                            # Linear component for the (x,y,z) velocities
        self.stop = False
        
        #LaserCallback Funtion
    def laser(self, msg):                   # Laser callback to find distance between
        ranges = msg.ranges                  # Range values of the laser 
        self.minRange = numpy.nanmin(ranges) # Numpynamin - Retrurn min values of array and axis & ignoring NaN values

    #Callback Funtion
    def callback(self,msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')  # Converting an image message pointer to an OpenCV message
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)                    # Converting the image form BGR to HSV
        h, w, d = image.shape                                           # Height width of distance of shape
        #----------------- COLOURS ----------------#
         # Defining all the upper and lower numbers for each colour in HSV 
         # H: 0 - 180, S: 0 - 255, V: 0 - 255
     
        #----RED----#
        # Defining the range of blue in HSV 
        lower_red = numpy.array([0,100,100])
        upper_red = numpy.array([0,255,255])
        rmask = cv2.inRange(hsv, lower_red, upper_red)   # Threshold the HSV image to get the colour red creating a mask 
        rM = cv2.moments(rmask)                          # Capturing moments of mask

    
        #-----BLUE------#
        # Defining the range of blue in HSV 
        lower_blue = numpy.array([110,50,50])
        upper_blue = numpy.array([130,255,255])
        bmask = cv2.inRange(hsv, lower_blue, upper_blue)   # Threshold the HSV image to get the colour blue creating a mask 
        bM = cv2.moments(bmask)                            # Capturing moments of mask
        
         #-----YELLOW------#
        # Defining the range of yellow in HSV 
        lower_yellow= numpy.array([30,100,100])
        upper_yellow = numpy.array([50,255,255])
        ymask = cv2.inRange(hsv, lower_yellow,upper_yellow) # Threshold the HSV image to get the colour yellow creating a mask 
        yM = cv2.moments(ymask)                             # Capturing moments of mask
                          
        #-----GREEN------#
          # defining the range of blue in HSV 
        lower_green = numpy.array([50,30,30])
        upper_green = numpy.array([80,255,255])
        gmask = cv2.inRange(hsv, lower_green,  upper_green) # Threshold the HSV image to get the colour green / creating a mask 
        gM = cv2.moments(gmask)                             # Capturing moments of mask
     
        #-----ALL MASKS------#
        mask = gmask + ymask +bmask +rmask                  # Previwing all mask in window
        mask = cv2.bitwise_and(image,image, mask= mask)     # Bitwise-AND mask and original image - To show colours on mask 

       #------------WAYPOINTS------------#
       # http://marte.aslab.upm.es/redmine/files/dmsf/p_drone-testbed/170324115730_268_Quigley_-_Programming_Robots_with_ROS.pdf
       # Page 21

       # Create a list to hold the waypoint poses
        points = list()
        
        #Points the xpostion, yposition, zposition, xorientation, yorientation, zorientation
        points = ['roomA',[2.87429, -4.01454, 0.0], [0.0, 0.0, 0.0, 1.0],
                  'roomB',[2.50469, 0.763397, 0.0], [0.0, 0.0, 0.0, 1.0],
                  'roomC',[3.71021, 4.422765, 0.0], [0.0, 0.0, 0.0, 1.0],
                  'roomD',[-3.34312, 3.82559, 0.0], [0.0, 0.0, 0.0, 1.0]]

       #pose = MoveBaseGoal()                               # Wait for the server to be ready for client action 
       #pose.position.x = pose[1][0]                        # First bracket, first digit in array
       #pose.position.y = pose[1][1]
       #pose.position.z = pose[1][2]
       #pose.orientation.x = pose[2][0]                     # Second bracket, first digit in array
       #pose.orientation.y = pose[2][1]
       #pose.orientation.z = pose[2][2]
       #pose.orientation.w = pose[2][3]       
        
       #server = actionlib.SimpleActionClient('MoveBase',MoveBaseAction)   # A function to help turn a waypoint into a 'MoveBaseGoal'
       #server.wait_for_server()
       #for pose in points:                                # A for loop going through each waypoint, and sending it to the moving/action goal                              
       #   goTo = points(pose)
       #   server.send_goal(goTo)
       #   server.wait_for_result()                        # Wait for server movement 
           
         #---------- IF STATEMENT TO SEARCH THROUGH -----------#   
        # If any of the colours are not found keep spinning and moving until an item is found 
        if self.rPoint or self.yPoint or self.bPoint or self.gPoint == False:

            self.twist.angular.z = 0.3                                      # Angular volecity
            self.twist.linear.x = 0.1                                       # Linear volecity
            self.cmd_vel_pub.publish(self.twist)                            # Publish twist movement
                                        
           # Look for indivual colours aswell if colours are not found
           # Mark true when they are found 
            if self.rPoint == False:     
                self.rPoint = self.findRed(rM,image,w)
            if self.yPoint == False: 
                self.yPoint = self.findYellow(yM,image,w)                
            if self.bPoint == False: 
                self.bPoint = self.findBlue(bM,image,w)
            elif self.gPoint == False: 
                self.gPoint = self.findGreen(gM,image,w)
                     
        #If all colour points are found (true)            
        elif self.rPoint and self.yPoint and self.bPoint and self.gPoint == True:
            self.twist.angular.z = 0.0                              # Spin and stop moving
            self.twist.linear.x = 0.0 
     
         # Display the images in windows 
        cv2.imshow("Image Window", image)
        cv2.imshow("Mask Window", mask)
         # Display a frame for 25 ms
        cv2.waitKey(25)              

           
        #--------------------- COLOURS --------------------#
         #Function to find colour and mark as true once found 
        #Seaching for RED point
    def findRed (self,rM,image,w):
        #If statment for if redPoint is not found
            if rM['m00'] > 0:
                if self.minRange > self.minDistance:               # If the range is more than the distance 
                    # Compute the center of a contour/shape region.
                    cx = int(rM['m10']/rM['m00'])                  # Centroid of x 
                    cy = int(rM['m01']/rM['m00'])                  # Centroid of y
                    cv2.circle(image, (cx, cy), 10, (0,0,0), -1)   # Draw circle in center 
                    err = cx - w/2
                    self.twist.linear.x = 1.0
                    self.twist.angular.z = -float(err) / 100 
                    self.cmd_vel_pub.publish(self.twist)            # Publish movement
                    print 'Red Colour Found, Ignore red\t', self.rPoint,'Distance:',self.minRange        # Print message
                    return False
                if  self.minRange >= self.minDistance:            # If the laser range is less or equal to the min distance 
                    self.rPoint = True                             # update the red point as true to stop finding the color red
                    self.twist.angular.z = 0.0                     # Angular volecity
                    self.twist.linear.x = 0.1                      # Linear volecity
                    self.cmd_vel_pub.publish(self.twist)           # Publish movement
                    return True
            else: 
                    return False
                     
   #Seaching for BLUE point                    
    def findBlue (self,bM,image,w):
        #If statment for if redPoint is not found
            if bM['m00'] > 0:
                if self.minRange > self.minDistance:               # If the range is more than the distance 
                    # Compute the center of a contour/shape region.
                    cx = int(bM['m10']/bM['m00'])                  # Centroid of x 
                    cy = int(bM['m01']/bM['m00'])                  # Centroid of y
                    cv2.circle(image, (cx, cy), 10, (0,0,0), -1)   # Draw circle in center 
                    err = cx - w/2
                    self.twist.linear.x = 1.0
                    self.twist.angular.z = -float(err) / 100 
                    self.cmd_vel_pub.publish(self.twist)            # Publish movement
                    print 'Blue Colour Found, Ignore blue\t', self.bPoint,'Distance:',self.minRange         # Print message -  Laser callback , and update on bluePoint if true of false
                    return False
                if  self.minRange >= self.minDistance:             # If the laser range is less or equal to the min distance 
                    self.bPoint = True                             # update the red point as true to stop finding the color red
                    self.twist.angular.z = 0.0                     # Angular volecity
                    self.twist.linear.x = 0.1                      # Linear volecity
                    self.cmd_vel_pub.publish(self.twist)           # Publish movement
                    return True
            else: 
                    return False
                     
    def findYellow (self,yM,image,w):
        #If statment for if redPoint is not found
            if yM['m00'] > 0:
                if self.minRange > self.minDistance:               # If the range is more than the distance 
                    # Compute the center of a contour/shape region.
                    cx = int(yM['m10']/yM['m00'])                  # Centroid of x 
                    cy = int(yM['m01']/yM['m00'])                  # Centroid of y
                    cv2.circle(image, (cx, cy), 10, (0,0,0), -1)   # Draw circle in center 
                    err = cx - w/2
                    self.twist.linear.x = 1.0
                    self.twist.angular.z = -float(err) / 100 
                    self.cmd_vel_pub.publish(self.twist)            # Publish movement
                    print 'Yellow Colour Found, Ignore yellow\t', self.yPoint,'Distance:',self.minRange        # Print message
                    return False
                if  self.minRange >= self.minDistance:             # If the laser range is less or equal to the min distance 
                    self.yPoint = True                             # update the red point as true to stop finding the color red
                    self.twist.angular.z = 0.0                     # Angular volecity
                    self.twist.linear.x = 0.1                      # Linear volecity
                    self.cmd_vel_pub.publish(self.twist)           # Publish movement
                    return True
            else: 
                    return False
                     
 
 #Seaching for Green point                     
    def findGreen(self,gM,image,w):
        #If statment for if redPoint is not found
            if gM['m00'] > 0:
                if self.minRange > self.minDistance:               # If the range is more than the distance 
                    # Compute the center of a contour/shape region.
                    cx = int(gM['m10']/gM['m00'])                  # Centroid of x 
                    cy = int(gM['m01']/gM['m00'])                  # Centroid of y
                    cv2.circle(image, (cx, cy), 10, (0,0,0), -1)   # Draw circle in center 
                    err = cx - w/2
                    self.twist.linear.x = 1.0
                    self.twist.angular.z = -float(err) / 100 
                    self.cmd_vel_pub.publish(self.twist)            # Publish movement
                    print 'Green Colour Found, Ignore green\t', self.gPoint,'Distance:',self.minRange  # Print message -  Laser callback , and update on bluePoint if true of false
                    return False
                if  self.minRange >= self.minDistance:             # If the laser range is less or equal to the min distance 
                    self.gPoint = True                             # update the red point as true to stop finding the color red
                    self.twist.angular.z = 0.0                     # Angular volecity
                    self.twist.linear.x = 0.1                      # Linear volecity
                    self.cmd_vel_pub.publish(self.twist)           # Publish movement
                    return True
            else: 
                    return False

rospy.init_node('Search')                                           # Passing node name 
search = Search()                                                   # Search class
rospy.Rate(10)                                                      # Loop 10 times per second (10hz)
rospy.spin()                                                        # Keeps node from e
#cv2.destroyAllWindows()