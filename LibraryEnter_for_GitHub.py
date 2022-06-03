import smbus


#############Networking###################
import paho.mqtt.client as mqtt
host = '34.196.149.97'
port = 8883
topic = "topic/slibrary/exit"


client = mqtt.Client(protocol = mqtt.MQTTv311)
client.connect(host, port = port, keepalive = 60) 

###########################################

class GridEye:
    __REG_FPSC = 0x02
    __REG_TOOL = 0x0E
    __REG_PIXL = 0x80

    def __init__(self, address=0x69):
        self.i2c = smbus.SMBus(1)   # 0 for Raspberry Pi Model B(256MB)
        self.address = address
        self.i2c.write_byte_data(self.address, self.__REG_FPSC, 0x00)

    def thermistorTemp(self):
        result = self.i2c.read_word_data(self.address, self.__REG_TOOL)
        if(result > 2047):
            result = result - 2048
            result = -result
        return result * 0.0625

    def pixelOut(self):
        out = []
        for x in range(0,54):
            temp = self.i2c.read_word_data(
                self.address, self.__REG_PIXL + (x * 2))
            if(temp > 2047):
                temp = temp - 4096
            out.append(temp * 0.25)
        return out

import datetime
import numpy as np
import cv2
import math
import time

import adafruit_amg88xx
import busio
import board

i2c = smbus.SMBus(1)
i2c_bus = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_amg88xx.AMG88XX(i2c_bus, addr=0x69)

temp = i2c.read_word_data(0x69, 0x80)

myeye = GridEye()

temp_min = 30
temp_max = 40


def red_detect(img):
    #convert to HSV space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    #Green HSV range 1
    hsv_min = np.array([30,64,0])    ######################
    hsv_max = np.array([90,255,255])    #######################
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    #Red HSV range1
    hsv_min = np.array([0,64,0])
    hsv_max = np.array([30,255,255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)    
    
    #Red HSV range 2
    hsv_min = np.array([150,64,0])
    hsv_max = np.array([179,255,255])
    mask3 = cv2.inRange(hsv, hsv_min, hsv_max)

    return mask2 + mask3 + mask1

   
def remove_objects(img, lower_size=None, upper_size=None):
    #find all objects
    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(img)

    sizes = stats[1:, -1]
    _img = np.zeros((labels.shape))

    #process all objects, label=0 is background, objects are stated from 1
    for i in range(1, nlabels):

        #remove small objects
        if (lower_size is not None) and (upper_size is not None):
            if lower_size < sizes[i - 1] and sizes[i - 1] < upper_size:
                _img[labels == i] = 255

        elif (lower_size is not None) and (upper_size is None):
            if lower_size < sizes[i - 1]:
                _img[labels == i] = 255

        elif (lower_size is None) and (upper_size is not None):
            if sizes[i - 1] < upper_size:
                _img[labels == i] = 255
                
    return _img

room_number = 0
total_disp = 0
before_x1 = 0
curr_disp1 = 0


t1 = time.time()
now_x1 = 0

b_room_number = -100

b_retval = 1

try:
    while(True):
        dt = datetime.datetime.today()
        hour = dt.hour
        
        if hour == 0:
            room_number = 0
            total_disp = 0
            curr_disp1 = 0
            
        
        if time.time() - t1 < 3:
            room_number = 0
            total_disp = 0
            curr_disp1 = 0
            
            print('hello')
           
            
       

        pixel = np.array(myeye.pixelOut())
      
        pixel.resize((8, 8))
        
        temp_min = temp_min*0.65 + (pixel.mean()+3.2)*0.35  #####################
        temp_max = temp_max*0.65 + (pixel.mean()+6.3)*0.35  ####################


        pixel = pixel.clip(temp_min, temp_max)
        
        pixel = (pixel - temp_min) / (temp_max - temp_min) * 255.0
        
        
        pixel = pixel.astype(np.uint8)
       
    
        pixel = cv2.applyColorMap(pixel, cv2.COLORMAP_JET)
        pixel = cv2.resize(pixel, (400,400))

    #detect Red Color
        cv2.imshow("PixelExitVision",pixel)
        mask = red_detect(pixel)

        
     #   mask
     #   gray_mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
     #   ret, mask_binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
        
        
    #remove small objects  
    #first, do the median filter for mask. then do the remove_object()
    
        ksize = 15
        kernel = np.ones((20,20),np.uint8)
        mask = cv2.medianBlur(mask,ksize)
        mask_opening = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
        mask_removed = remove_objects(mask_opening, lower_size = 3000) #################
        mask2 = np.uint8(mask_removed)
        

    #count people number, Area, location
    #circle the centroids of the objects      
        connectivity = 8
        retval, labels, stats, centroids = cv2.connectedComponentsWithStats(mask2, connectivity, cv2.CC_STAT_TOP, cv2.CV_32S)
        
        #mask2 = cv2.watershed(mask2, labels)
            
        #this program is just for test
        #you don't need write this below program if you add room number continuity
        
    #count people number, Area, location
    #circle the centroids of the objects      
      
        
        
        
        for i in range(1,retval):
            
            center_x = centroids[i][0]
            center_y = centroids[i][1]
            
          #  print("People"+str(i)+"'s location is "+str(center_y))

      
#            print("Number", i)
           # print("Location:\n X:", math.floor(center_x), "Y:", math.floor(center_y))     
            cv2.circle(mask2,(int(center_x),int(center_y)),30,(0,200,0), thickness=3, lineType=cv2.LINE_AA)
            cv2.putText(mask2,
                        text = str(i),
                        org = (int(center_x),int(center_y)),
                        fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale = 1.0,
                        color = (0,255,0),
                        thickness = 2,
                        lineType = cv2.LINE_4)
        
        cv2.putText(mask2, '0', (1, 400),
                    cv2.FONT_HERSHEY_PLAIN, 1.5,
                    (255, 255, 255), 2, cv2.LINE_AA)
        
        cv2.putText(mask2, '400', (350, 400),
                    cv2.FONT_HERSHEY_PLAIN, 1.5,
                    (255, 255, 255), 2, cv2.LINE_AA)
        
            

        #if someone appear
        
        
    ###################### Count Displacement #############################

        if retval-1 == 0:
            before_x1 = 0
            total_disp = 0
            curr_disp1 = 0
            now_x1 = 0

        
        THRESHOLD = 200  ####################
        
        #if b_retval != retval and retval != 1:
            #print()
            #print()
            
        
        if retval-1 >= 1:
            now_x1 = math.ceil(centroids[1][0])
            
        else:
            now_x1 = 0
        
        curr_disp1 = now_x1 - before_x1

                    
        if curr_disp1 > THRESHOLD:
            print(f"E 1   curr_disp1 :{curr_disp1}")
            print()
            curr_disp1 = 100
            print(f"curr_disp1 :{curr_disp1}")
            before_x1 = now_x1

            
        elif curr_disp1 < -THRESHOLD:
            print(f"E 2   curr_disp1 :{curr_disp1}")
            curr_disp1 = 0
            before_x1 = now_x1
            
           
            
        total_disp = curr_disp1 + total_disp
        
        
        if total_disp >= THRESHOLD:
            room_number += 1
            #total_disp = -50
            
        
        '''
        if total_disp <= -THRESHOLD:
            room_number += 1
            #total_disp = 0
        '''
        
        if retval > 1:
            #print("total_disp:",total_disp)
            if b_room_number != room_number:
                print()
                #print(f" SUCSESS   ROOM NUMBER: {room_number} Time: {dt}")
                print(" SUCSESS ",dt,'ROOM NUMBER: ',room_number, "total_disp", total_disp)
                total_disp = 0
                print()
                
        before_x1 = now_x1

        b_room_number = room_number
        b_retval = retval
        
        client.publish(topic, room_number)
        
        cv2.imshow("MaskedExitVision",mask2)

        
        
        if cv2.waitKey(100) == 27:  # ESC
            break
     
except(KeyboardInterrupt, SystemExit):
    print()

cv2.destroyAllWindows()








