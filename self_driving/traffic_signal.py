#!/usr/bin/python3
import rclpy 
import cv2 
import os, sys
import numpy as np
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from std_msgs.msg import String  # Import message type for traffic sign

# from .Detection.Lanes.trafficsign  import det_traffic_sign

path = r"/home/an/ros2_ws/src/self_driving/self_driving/Detection/Lanes/selected_images_path"
def det_traffic_sign(frame):
  #  Lenh de in tat ca file va thu muc
  max_good_matches = 0
  best_matching_file = ""
  for file in os.listdir( path ):
      print(file[:-4])
      img1 = cv2.imread(os.path.join(path,file), cv2.IMREAD_GRAYSCALE)
      img2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      MIN_MATCH_COUNT = 3
      # Initiate SIFT detector
      sift = cv2.SIFT_create()
      # find the keypoints and descriptors with SIFT
      kp1, des1 = sift.detectAndCompute(img1,None)
      kp2, des2 = sift.detectAndCompute(img2,None)
      FLANN_INDEX_KDTREE = 1
      index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
      search_params = dict(checks = 50)
      flann = cv2.FlannBasedMatcher(index_params, search_params)
      if des1 is not None and des2 is not None:
        # Tiếp tục xử lý
        matches = flann.knnMatch(des1, des2, k=2)
      # store all the good matches as per Lowe's ratio test.
      good = []
      for m,n in matches:
          if m.distance < 0.7*n.distance:
              good.append(m)

      print(len(good))
      if len(good) > 5:
          src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
          dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
          M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
          matchesMask = mask.ravel().tolist()
          inliers_count = np.sum(matchesMask)
          h, w = img1.shape
          pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
          dst = cv2.perspectiveTransform(pts, M)
          frame = cv2.polylines(frame, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)
          frame = cv2.putText(frame, file[:-4] , (10, 100), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (0, 255, 0), 2, cv2.LINE_AA)
          
          
          if inliers_count>max_good_matches and inliers_count>5:
            max_good_matches = inliers_count
            best_matching_file = file

  # frame = cv2.putText(frame, "[" + best_matching_file[:-4] + "]", (10,100), cv2.FONT_HERSHEY_SIMPLEX,  
  #                    2, (0, 255, 0), 3, cv2.LINE_AA)   
  # print(max_good_matches)
  string = best_matching_file[:-4]
  frame = cv2.putText(frame, string , (10, 100), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (0, 255, 0), 2, cv2.LINE_AA)
  # print(string)
  cv2.imshow("out", frame)    
  return string
    
def det_traffic_light(img):
    # cv2.imshow("input", img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_Blur = cv2.GaussianBlur(hsv, (5,5), 0)

    lower_red1 = np.array([0, 0, 230])
    upper_red1 = np.array([60, 60, 255])
    lower_red2 = np.array([160,0,220])
    upper_red2 = np.array([180,15,255])
    lower_green = np.array([35,80,80])
    upper_green = np.array([60,255,255])
    # lower_yellow = np.array([15,100,100])
    # upper_yellow = np.array([35,255,255])
    lower_yellow = np.array([30,0,255])
    upper_yellow = np.array([35,50,255])

        
    mask1 = cv2.inRange(hsv_Blur, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_Blur, lower_red2, upper_red2)
    maskg = cv2.inRange(hsv_Blur, lower_green, upper_green)
    masky = cv2.inRange(hsv_Blur, lower_yellow, upper_yellow)   
    maskr = cv2.add(mask1, mask2)
    # cv2.imshow("g", maskg)
    # cv2.imshow("r", maskr)


    params = cv2.SimpleBlobDetector_Params()

    params.minThreshold = 10;
    params.maxThreshold = 255;

    # Set Area filtering parameters 
    params.filterByArea = True
    params.minArea = 10
    
    # Set Circularity filtering parameters 
    params.filterByCircularity = True 
    params.minCircularity = 0.3

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.3
    
    # Set Convexity filtering parameters 
    params.filterByConvexity = True
    params.minConvexity = 0.01
    
     #  Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.3

    detector = cv2.SimpleBlobDetector_create(params)

    maskg = cv2.bitwise_not(maskg, mask = None) 
    maskr = cv2.bitwise_not(maskr, mask = None)
    masky = cv2.bitwise_not(masky, mask = None)

    keypoints_g = detector.detect(maskg)
    keypoints_y = detector.detect(masky)
    keypoints_r = detector.detect(maskr)

    blank = np.zeros((5, 5))
    img = cv2.drawKeypoints(
        img, keypoints_g, blank, (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
    )
    img = cv2.drawKeypoints(
        img, keypoints_r, blank, (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
    )
    img = cv2.drawKeypoints(
        img, keypoints_y, blank, (0, 0, 255), 
        flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
    )
    centers_g = [(int(k.pt[0]), int(k.pt[1])) for k in keypoints_g]
    centers_y = [(int(k.pt[0]), int(k.pt[1])) for k in keypoints_y]
    centers_r = [(int(k.pt[0]), int(k.pt[1])) for k in keypoints_r]
    # print(centers_g,centers_r,centers_y)

    if centers_r:
        text = "red_light"
    elif centers_y:
        text = "yellow_light"
    elif centers_g:
        text = "green_light"
    else:
        text = ""
    # print(text)
    # cv2.imshow("out1",img)  
    cv2.waitKey(1)  
    return text
  
  
class Video_get(Node):
  def __init__(self):
    super().__init__('sift_node')# node name
    ## Created a subscriber 
    self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
    self.publisher = self.create_publisher(String, 'traffic_sign', 10)  # Create publisher for traffic sign
    self.sign = ["30",'60','stop']
    self.counts = {s: 0 for s in self.sign}
    # setting for writing the frames into a video
    self.bridge = CvBridge() # converting ros images to opencv data
    self.msg = String()
    self.vel = 50
 
  def process_data(self, data): 
    frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
    width = 320
    height = 240
    dim = (width, height)  
    # resize image
    frame = cv2.resize(frame, dim, cv2.INTER_AREA)
    frame = frame[60:, :]
    text = det_traffic_light(frame)
    string = det_traffic_sign(frame)
    if string in self.sign:
            self.counts[string] += 1
            if self.counts[string] == 1:
                
                # xuli               
                if string == "60":
                  self.vel = "60"
                  # print(self.vel)
                if string == "30":
                  self.vel = "30"
                if string == "stop":
                  self.vel = "stop"
                for sign in self.sign:
                    self.counts[sign] = 0
    if text == "red_light":
      self.vel = "" 
    if text == "green_light":
      self.vel = ""              
    
    self.msg.data = str(self.vel)  # Set message data to traffic sign string
    # self.get_logger().info(self.msg.data)
    self.publisher.publish(self.msg)  # Publish the message 
    

  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_get()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
