import cv2
from .Detection.Lanes.lane_detection import detect_lanes
from numpy import interp
import os
import numpy as np




class Control():
    def __init__(self):
        # Khởi tạo các biến cho góc quay và tốc độ ban đầu của xe
        self.angle = 0.0
        self.speed = 80
        # self.pid = [0.005, 0.00005, 0.01]
        self.pid = [0.005, 0.0, 0.005]
        self.setpoint = 0
        self.p_err = 0
        self.i_err = 0
        self.text = ""
        self.anpha = 0.2
        self.p_Ud = 0
        # print(self.i_err)
        
    def follow_lane(self, max_sane_dist, dist, curv):
        self.speed = 80
        # print(self.text)
        if self.text == "60":
            self.speed = 60
        if curv > 30 or curv < -30 or dist < -200 :
            self.speed = 50
        # Thiết lập các giá trị tối đa cho góc quay       
        if self.text == "stop" or self.text == "red_light":
            self.speed = 0
        if self.text == "30" or  self.text == "green_light":
            self.speed = 30
            

        max_turn_angle = 90
        max_turn_angle_neg = -90
        req_turn_angle = 0

        # Xác định góc quay yêu cầu dựa trên khoảng cách và độ cong của làn đường
        if dist > max_sane_dist or dist < -max_sane_dist:
            if dist > max_sane_dist:
                req_turn_angle = max_turn_angle 
            else:
                req_turn_angle = max_turn_angle_neg 
        else:
            # Tính toán góc quay dựa trên khoảng cách và độ cong của làn đường
            req_turn_angle = interp(dist, [-max_sane_dist, max_sane_dist], [-max_turn_angle, max_turn_angle])
        req_turn_angle = req_turn_angle + curv
        
        # Xử lý trường hợp góc quay vượt quá giới hạn
        if req_turn_angle > max_turn_angle or req_turn_angle < max_turn_angle_neg:
            if req_turn_angle > max_turn_angle:
                req_turn_angle = max_turn_angle
            else:
                req_turn_angle = max_turn_angle_neg
        # Ánh xạ góc quay vào khoảng [-45, 45] độ
        self.angle = interp(req_turn_angle, [max_turn_angle_neg, max_turn_angle], [-45, 45])
        
    def drive(self, current_state):
        # Unpacking dữ liệu trạng thái hiện tại của xe
        [dist, curv, img] = current_state

        # Kiểm tra xem liệu có dữ liệu hợp lệ để tiếp tục lái xe hay không
        if dist != 1000 and curv != 1000:
            # Theo dõi làn đường nếu dữ liệu hợp lệ
            self.follow_lane(img.shape[1] / 4, dist, curv)
        else:
            # Dừng xe nếu dữ liệu không hợp lệ
            self.speed = 0.0
        
        # Ánh xạ góc quay và tốc độ của xe vào phạm vi [0.5, -0.5] và [1, 2] tương ứng
        # self.angle = interp(self.angle, [-45, 45], [0.5, -0.5])
        with open('/home/an/ros2_ws/src/self_driving/self_driving/angle.txt', 'a') as f:
            f.write(str(self.angle)+ '\n')
        self.err = self.setpoint - self.angle
        self.i_err += self.err
        self.yaw = self.pid[0] * self.err +self.pid[1]  * self.i_err + self.pid[2] *((1-self.anpha)*(self.err - self.p_err) + self.anpha*(self.p_Ud))
        # self.yaw = self.pid[0] * self.err +self.pid[1]  * self.i_err + self.pid[2]*(self.err - self.p_err)  
        self.p_Ud = (self.err - self.p_err)
        # print(self.yaw)
        self.speed = interp(self.speed, [0, 90], [0, 0.1])

        self.yaw = np.clip(self.yaw, -0.2, 0.2)

        
class Car():
    def __init__(self):
        # Khởi tạo một đối tượng Control để điều khiển xe
        self.control = Control()

    def display_state(self,frame_disp,angle_of_car,current_speed, text):  
         # Translate [ ROS Car Control Range ===> Real World angle and speed  ]
        angle_of_car  = interp(angle_of_car,[-0.2,0.2],[45,-45])
        if (current_speed !=0.0):
            current_speed = interp(current_speed  ,[0  ,   0.1],[0 ,90])

        ###################################################  Displaying CONTROL STATE ####################################

        if (angle_of_car <-10):
            direction_string="[ Left ]"
            color_direction=(120,0,255)
        elif (angle_of_car >10):
            direction_string="[ Right ]"
            color_direction=(120,0,255)
        else:
            direction_string="[ Straight ]"
            color_direction=(0,255,0)

        if(current_speed>0):
            direction_string = "Moving --> "+ direction_string
        else:
            color_direction=(0,0,255)


        cv2.putText(frame_disp,str(direction_string),(20,40),cv2.FONT_HERSHEY_DUPLEX,0.4,color_direction,1)

        angle_speed_str = "[ Angle ,Speed ] = [ " + str(int(angle_of_car)) + "deg ," + str(int(current_speed)) + "mph ]"
        cv2.putText(frame_disp,str(angle_speed_str),(20,20),cv2.FONT_HERSHEY_DUPLEX,0.4,(0,0,255),1)

        cv2.putText(frame_disp,str(text),(20,60),cv2.FONT_HERSHEY_DUPLEX,0.4,(0,0,255),1)
            
        
    def drive_car(self, frame, p_err, text):

        self.control.p_err = p_err
        self.control.text = text
        # Trích xuất phần hình ảnh chứa đường đi từ frame đầu vào
        
        img = frame[100:400, 40:600]
        
        # Resize hình ảnh để giảm thời gian tính toán
        img = cv2.resize(img, (320, 240))
        
        # Nhận dạng khoảng cách và độ cong của làn đường từ hình ảnh
        distance, curvature = detect_lanes(img)

        # Tạo trạng thái hiện tại của xe từ dữ liệu nhận dạng
        current_state = [distance, curvature, img]
        

        
        # Điều khiển xe dựa trên trạng thái hiện tại
        self.control.drive(current_state)


        
        self.display_state(img,self.control.yaw, self.control.speed, text)

        # Trả về góc quay, tốc độ và hình ảnh cho mục đíchhiển thị hoặc xử lý tiếp theo
        return self.control.err, self.control.yaw, self.control.speed, img
