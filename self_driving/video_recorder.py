#!/usr/bin/python3
import rclpy 
import datetime
import cv2 
import os, sys
import numpy as np
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
# from .Detection.Lanes.trafficsign  import det_traffic_sign

path = r"/home/cuong/ros2_ws/src/prius_sdc_pkg/prius_sdc_pkg/Detection/Lanes/selected_images_path"
def det_traffic_sign(frame):
  #  Lenh de in tat ca file va thu muc
  max_good_matches = 0
  best_matching_file = ""
  for file in os.listdir( path ):
      # print(file)
      img1 = cv2.imread(os.path.join(path,file), cv2.IMREAD_GRAYSCALE)
      img2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      MIN_MATCH_COUNT = 10
      # Initiate SIFT detector
      sift = cv2.SIFT_create()
      # find the keypoints and descriptors with SIFT
      kp1, des1 = sift.detectAndCompute(img1,None)
      kp2, des2 = sift.detectAndCompute(img2,None)
      FLANN_INDEX_KDTREE = 1
      index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
      search_params = dict(checks = 50)
      flann = cv2.FlannBasedMatcher(index_params, search_params)
      matches = flann.knnMatch(des1,des2,k=2)
      # store all the good matches as per Lowe's ratio test.
      good = []
      for m,n in matches:
          if m.distance < 0.7*n.distance:
              good.append(m)

      # print(len(good))
      if len(good)>MIN_MATCH_COUNT:
          src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
          dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
          M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
          matchesMask = mask.ravel().tolist()
          inliers_count = np.sum(matchesMask)
          
          if inliers_count>max_good_matches and inliers_count>5:
            max_good_matches = inliers_count
            best_matching_file = file

  # frame = cv2.putText(frame, "[" + best_matching_file[:-4] + "]", (10,100), cv2.FONT_HERSHEY_SIMPLEX,  
  #                    2, (0, 255, 0), 3, cv2.LINE_AA)   
  # print(max_good_matches)
  string = best_matching_file[:-4]         
  return string    

class Video_get(Node):
  def __init__(self):
    super().__init__('video_subscriber')# node name
    ## Created a subscriber 
    self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
    # self.sign = ["30",'60','turn','stop']
    # self.counts = {s: 0 for s in self.sign}
    # setting for writing the frames into a video
    ngay_gio_hien_tai = datetime.datetime.now()
    ten_tap_tin = '/home/an/ros2_ws/src/self_driving/self_driving/output{}.avi'.format(ngay_gio_hien_tai.strftime("%Y-%m-%d_%H-%M-%S"))

    # Tạo video writer với tên tập tin đã được định dạng
    self.out = cv2.VideoWriter(ten_tap_tin, cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))    
    self.bridge = CvBridge() # converting ros images to opencv data
    
    print("Ngày và giờ hiện tại:", ngay_gio_hien_tai)

 
  def process_data(self, data): 
    frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
    # frame = cv2.resize(frame, (320, 240))
    # string = det_traffic_sign(frame)
    # if string in self.sign:
    #         self.counts[string] += 1
    #         if self.counts[string] == 2:
    #             print("Traffic sign:", string)
    #             for sign in self.sign:
    #                 self.counts[sign] = 0

    self.out.write(frame)# write the frames to a video
    # cv2.imshow("output", frame) # displaying what is being recorded 
    cv2.waitKey(1) # will save video until it is interrupted
  

  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_get()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
