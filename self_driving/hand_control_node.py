import cv2
import mediapipe as mp
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class HandDetector:
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=self.mode,
                                        max_num_hands=self.maxHands,
                                        min_detection_confidence=self.detectionCon,
                                        min_tracking_confidence=self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo=0, draw=True):
        lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
        return lmList

class HandControlNode(Node):
    def __init__(self):
        super().__init__('hand_control_node')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.velocity = Twist()
        self.detector = HandDetector(detectionCon=0.75)
        self.tipIds = [4, 8, 12, 16, 20]
        self.pTime = 0
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img = self.detector.findHands(img)
        lmList = self.detector.findPosition(img, draw=False)

        if len(lmList) != 0:
            fingers = []

            # Thumb
            if lmList[self.tipIds[0]][1] < lmList[self.tipIds[0] - 1][1]:
                fingers.append(1)
            else:
                fingers.append(0)

            # 4 Fingers
            for id in range(1, 5):
                if lmList[self.tipIds[id]][2] < lmList[self.tipIds[id] - 2][2]:
                    fingers.append(1)
                else:
                    fingers.append(0)

            totalFingers = fingers.count(1)
            print(totalFingers)

            if totalFingers == 0:
                self.velocity.linear.x = 0.0
                self.velocity.angular.z = 0.0
            elif totalFingers == 1:
                self.velocity.linear.x = 0.1
                self.velocity.angular.z = 0.0
            elif totalFingers == 2:
                self.velocity.linear.x = -0.1
                self.velocity.angular.z = 0.0

            self.velocity_publisher.publish(self.velocity)

            cv2.rectangle(img, (20, 225), (170, 425), (0, 255, 0), cv2.FILLED)
            cv2.putText(img, str(totalFingers), (45, 375), cv2.FONT_HERSHEY_PLAIN,
                        10, (255, 0, 0), 25)

        cTime = time.time()
        fps = 1 / (cTime - self.pTime)
        self.pTime = cTime

        cv2.putText(img, f'FPS: {int(fps)}', (400, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 3)

        cv2.imshow("Image", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = HandControlNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
