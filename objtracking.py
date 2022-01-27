import cv2
import numpy as np
from numpy import mean, array
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from cv2 import namedWindow, cvtColor, imshow, inRange
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey, COLOR_BGR2HSV
from cv2 import blur, Canny, resize, INTER_CUBIC

class count_fruits:

    def __init__(self): 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect",Image, self.process_image)
        # self.max is used to display the maximum number of grapes in the image. we set it to zero at initial stage.
        self.max = 0

    def process_image(self, camera):

        
        #cam= cv2.VideoCapture(0)
        self.kernelClose=np.ones((15,15))
        img = self.bridge.imgmsg_to_cv2(camera, "bgr8")
        img = resize(img, None, fx=0.6, fy=0.6, interpolation = INTER_CUBIC)
        #img=cv2.resize(img,(340,220))


        # The upper bound and the lower bound is used to identifiy the type of color in the image.
        # In this scenario its purple.

        lowerBound=np.array([100,18,46])
        upperBound=np.array([107,256,256])
        font = cv2.FONT_HERSHEY_SIMPLEX

        #convert BGR to HSV  H - Hue, S - Saturation, V - value.
        imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        cv2.imshow('HSV',imgHSV)
        # create the Mask
        mask=cv2.inRange(imgHSV,lowerBound,upperBound)
        #morphology
        self.maskClose=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,self.kernelClose)

        maskFinal=self.maskClose
        _,conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        
        cv2.drawContours(img,conts,-1,(255,255,255),1)
        for i in range(len(conts)):
            area = cv2.contourArea(conts[i])       
            if area >10:
                x,y,w,h=cv2.boundingRect(conts[i]) 

                # This displays the co-ordinates of the grapes  .
                print("Frames", x, y, w, h)

                #x and y are the coordinates of the image_img and w and h are the width and height.
                #where (x,y)-left part of the image and (x+w,y+h)-right part and the other two represents color and thickness.
                
                #displays rectangular box around the grapes.
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255), 2)
                
                # used to dispaly text in the image
                cv2.putText(img, str(i+1),(x,y+h),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0))
               
               # This line of code is refered from a youtube video (https://www.youtube.com/watch?v=GgGro5IV-cs)
               # This line of code is used to display the center red dot.
                cx = int((x+x+w)/2)
                cy = int((y+y+h)/2)
                cv2.circle(img,(cx,cy), 5, (0,0,255),-1)
            
        # used to display the final processed image .   
        cv2.imshow("MaskClose", self.maskClose)
        cv2.imshow("Mask",mask)
        cv2.imshow("Camera",img)
        print("Total number of grapes")
        
        #displays the maximum number of graph
        # We set the value of max to length of the conts so that it displays the graph value
        if len(conts) > self.max:
            self.max = len(conts)
        print(self.max)
        
        # waitkey is used to freeze the frame

        if cv2.waitKey(10) &0xFF ==ord('q'):
                    cv2.cap.release()

                     # Used to close all the screens
                    cv2.destroyAllWindows()  
                
if __name__ =='__main__':
    rospy.init_node('count_fruits')
    cf = count_fruits()
    rospy.spin()




