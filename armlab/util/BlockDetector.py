import numpy as np

import sys
import cv2

#TODO: Uncomment these lines to run in real system.
import freenect

import IPython
import pandas as pd
from sklearn.svm import SVC
import pickle


class DetectBlockClass():

    def __init__(self):

        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.ROIRect = np.array([[137,92],[480,440]])
         

        self.font = cv2.FONT_HERSHEY_COMPLEX
        self.output_rgb = "R:%d, G:%d, B:%d " % (0, 0, 0)
        self.output_hsv = "H:%d, S:%d, V:%d" % (0, 0, 0)
        self.output_gray = "Gray: {:d}".format(0)

        self.HSVThreshold =  {}


        self.ColorNameList = [ "black",
                                "red",
                                "orange",
                                "yellow",
                                "green",
                                "blue",
                                "purple",
                                "pink" ]  
        #Lower  HSV threshold first
        #Higher HSV thresfold after
        self.HSVThreshold['black']  = np.array( [[75, 4, 47], [156, 102, 129]])
        self.HSVThreshold['red']    = np.array( [[170, 186, 160], [175, 222, 179]])
        self.HSVThreshold['orange'] = np.array([[4, 113, 222], [9, 218, 255]])
        self.HSVThreshold['yellow'] = np.array([[0, 0, 229], [140, 80, 255]])
        self.HSVThreshold['green']  = np.array([[56, 25, 142], [77, 125, 243]]) #np.array([[55,55,140],[72,120,170]])
        self.HSVThreshold['blue']   = np.array([[110, 73, 151], [118, 170, 237]])  #np.array([[90,110,140],[130,190,225]])
        self.HSVThreshold['purple'] = np.array( [[130, 52, 134], [154, 125, 216]])
        self.HSVThreshold['pink']   = np.array( [[150, 2, 239], [171, 174, 254]])

        self.Depthhomography = np.loadtxt('../DepthRegistrationMatrix.txt',delimiter=',')
        
        #Frames
        self.bgr_frame= []
        self.hsv_frame= []
  
        #Countours
        self.Contours = {}
        self.Contours['black'] = np.array([])
        self.Contours['red'] = np.array([])
        self.Contours['orange'] = np.array([])
        self.Contours['yellow'] = np.array([])
        self.Contours['green'] = np.array([])
        self.Contours['blue'] = np.array([])
        self.Contours['purple'] = np.array([])
        self.Contours['pink'] = np.array([])

        #Center of Mass
        self.CenterOfMass = {}
        self.CenterOfMass['black']  = np.array([])
        self.CenterOfMass['red']    = np.array([])
        self.CenterOfMass['orange'] = np.array([])
        self.CenterOfMass['yellow'] = np.array([])
        self.CenterOfMass['green']  = np.array([])
        self.CenterOfMass['blue']   = np.array([])
        self.CenterOfMass['purple'] = np.array([])
        self.CenterOfMass['pink']   = np.array([])

        #SVM Color Labels
        self.ColorLabel = {}
        self.ColorLabel['black']    = 0
        self.ColorLabel['red']      = 1
        self.ColorLabel['orange']   = 2
        self.ColorLabel['yellow']   = 3
        self.ColorLabel['green']    = 4
        self.ColorLabel['blue']     = 5
        self.ColorLabel['purple']   = 6
        self.ColorLabel['pink']     = 7

        self.ColorlabelText = {}
        self.ColorlabelText[0] = 'black'
        self.ColorlabelText[1] = 'red'
        self.ColorlabelText[2] = 'orange'
        self.ColorlabelText[3] = 'yellow'
        self.ColorlabelText[4] = 'green'
        self.ColorlabelText[5] = 'blue'
        self.ColorlabelText[6] = 'purple'
        self.ColorlabelText[7] = 'pink'




        
        #Source of the image
        self.ImageSourceCamera = True
        self.RGBImageFileName = './color_data/cdata_09262019_2012/RGBImage0.png'
        self.DepthImageFileName = './color_data/cdata_09262019_2012/DepthImage0.csv'

        self.kernel = np.ones((3,3),np.uint8)

        self.color_mean_list = []

        self.Loadimages()

    def Loadimages(self):
        try:
            self.bgr_frame_fromfile = cv2.imread(self.RGBImageFileName)
            #self.bgr_frame = cv2.cvtColor(self.bgr_frame, cv2.COLOR_RGB2BGR)
        except:
            print('Error: no image found.')

        try:
            self.depth_frame_fromfile = np.loadtxt(self.DepthImageFileName,delimiter=',')
            #self.bgr_frame = cv2.cvtColor(self.bgr_frame, cv2.COLOR_RGB2BGR)
        except:
            print('Error: no image found.')

        
    def SETROI(self,UpperLeftPoint, LowerRightPoint):
        self.ROIRect = np.array([UpperLeftPoint, LowerRightPoint])

    def GetBGRImageFrame(self):
        if self.ImageSourceCamera:
            #TODO: Uncomment these lines to run in real system.
            self.bgr_frame = freenect.sync_get_video()[0]
            self.bgr_frame = cv2.cvtColor(self.bgr_frame, cv2.COLOR_RGB2BGR)
            pass
        else:
            self.bgr_frame = self.bgr_frame_fromfile.copy()
            return self.bgr_frame

    def GetDepthImageFrame(self):
        if self.ImageSourceCamera:
            #TODO: Uncomment these lines to run in real system.
            #self.bgr_frame = freenect.sync_get_video()[0]
            dframe = freenect.sync_get_depth()[0]
            self.DepthHSV[...,0] = dframe
            self.DepthHSV[...,1] = 0x9F 
            self.DepthHSV[...,2] = 0xFF
            self.depth_frame = cv2.cvtColor(self.DepthHSV, cv2.COLOR_HSV2BGR)
            self.depth_frame = cv2.cvtColor(self.depth_frame, cv2.COLOR_BGR2GRAY)
            self.depth_frame = cv2.warpPerspective(self.depth_frame, self.Depthhomography, (self.depth_frame.shape[1], self.depth_frame.shape[0])) 
            return self.depth_frame
        else:
            dframe = self.depth_frame_fromfile.copy()
            self.DepthHSV[...,0] = dframe 
            self.DepthHSV[...,1] = 0xFF
            self.DepthHSV[...,2] = 0xFF
            self.depth_frame = cv2.cvtColor(self.DepthHSV, cv2.COLOR_HSV2BGR)
            self.depth_frame = cv2.cvtColor(self.depth_frame, cv2.COLOR_BGR2GRAY)
            return self.depth_frame


    def WriteImageFrame(self):
         #TODO: Uncomment these lines to run in real system.
        #self.bgr_frame = freenect.sync_get_video()[0]
        #cv2.imwrite(self.ImageFileName, self.bgr_frame)
        pass
    def GetBlockContours(self, BGRImage  ,BlockColor):
        self.hsv_frame = cv2.cvtColor(BGRImage, cv2.COLOR_BGR2HSV)
        HSVLower_limit = self.HSVThreshold[BlockColor][0]
        HSVUpper_limit = self.HSVThreshold[BlockColor][1]

        # Threshold the HSV image to get only blue colors
        self.mask = cv2.inRange(self.hsv_frame, HSVLower_limit, HSVUpper_limit)

        # Bitwise-AND mask and original image
        self.masked_image = cv2.bitwise_and(self.bgr_frame, self.bgr_frame, mask= self.mask)
        self.denoised = cv2.morphologyEx(self.masked_image, cv2.MORPH_OPEN, self.kernel)
        self.denoised = cv2.morphologyEx(self.denoised, cv2.MORPH_CLOSE, self.kernel)

        self.denoised_gray = cv2.cvtColor(self.denoised,cv2.COLOR_BGR2GRAY)
        ret,self.binary_image = cv2.threshold(self.denoised_gray,50,255,cv2.THRESH_BINARY)
        contours, heirs = cv2.findContours( self.binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        return contours

    def GetBlockContoursDepth(self, DepthImage):
        

        GrayLimits = np.array([ [180,200],   #First level
                                [142, 175 ]  #Second level
                                ])

        AreaLimit  = np.array([
                                [350,570],
                                [500,700]
                            ])
        
        countor_list = []
        mean_list = []
        for i in range(GrayLimits.shape[0]):
            GrayLow_limit = GrayLimits[i,0]
            GrayUpper_limit = GrayLimits[i,1]

            ROI_image = DepthImage[self.ROIRect[0,1]: self.ROIRect[1,1],self.ROIRect[0,0]: self.ROIRect[1,0]].copy()
            # Threshold the HSV image to get only blue colors
            self.mask = cv2.inRange(ROI_image, GrayLow_limit, GrayUpper_limit)

            # Bitwise-AND mask and original image
            self.masked_image = cv2.bitwise_and(ROI_image, ROI_image, mask= self.mask)
            self.denoised = cv2.morphologyEx(self.masked_image, cv2.MORPH_OPEN, self.kernel)
            self.denoised = cv2.morphologyEx(self.denoised, cv2.MORPH_CLOSE, self.kernel)

            #self.denoised_gray = cv2.cvtColor(self.denoised,cv2.COLOR_BGR2GRAY)
            ret,self.binary_image = cv2.threshold(self.denoised,50,255,cv2.THRESH_BINARY)
            contours, heirs = cv2.findContours( self.binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
            for each_contour in contours:
                area = cv2.contourArea(each_contour)
                #print(area)
                if area >=AreaLimit[i][0] and area<AreaLimit[i][1] :
                #if True:
                    #print(area)
                    countor_list.append(each_contour+self.ROIRect[0])
                    mask = np.zeros(DepthImage.shape, np.uint8)
                    cv2.drawContours(mask,np.array([ each_contour+self.ROIRect[0]]), -1, 255, -1)
                    mean = cv2.mean(self.bgr_frame, mask=mask)
                    #print("Mean: "+str(mean))
                    mean_list.append(mean)

        return np.array(countor_list), np.array(mean_list)

    def detectBlocksInDepthImage(self):
        """
        TODO: Implement a blob detector to find blocks
        in the depth image
        """
        self.block_contours, self.color_mean_list = self.GetBlockContoursDepth(self.depth_frame)
        #IPython.embed()

    def DrawContoursOnFrame(self, ImageFrame):
        for each_contour in self.block_contours:
            M = cv2.moments(each_contour)
            try:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(ImageFrame, (cx,cy), 2, (0,0,0),2)
            except:
                print('Error division by zero.')

    def DrawRectanglesOnFrame(self, ImageFrame):
        for each_contour, each_mean_color in zip(self.block_contours,self.color_mean_list):
            try:
                rect = cv2.minAreaRect(each_contour)
                box = cv2.boxPoints(rect) # cv2.boxPoints(rect) for OpenCV 3.x
                box = np.int0(box)
                #cv2.drawContours(ImageFrame,[box],0,(0,0,255),2)
                cv2.drawContours(ImageFrame,[box],0,each_mean_color,2)
                
            except:
                print('Error division by zero.')

    def mouse_callback(self,event, x,y, flags, param):
        self.hsv_frame = cv2.cvtColor(self.bgr_frame, cv2.COLOR_BGR2HSV)
        if (x>=0) and (x<=639) and (y>=0) and (y<=479): 
            r = self.bgr_frame[y][x][2]
            g = self.bgr_frame[y][x][1]
            b = self.bgr_frame[y][x][0]
            h = self.hsv_frame[y][x][0]
            s = self.hsv_frame[y][x][1]
            v = self.hsv_frame[y][x][2]
            L = self.depth_frame[y][x]
            self.output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
            self.output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
            self.output_gray = "Gray: {:d}".format(L)
            if event == cv2.EVENT_LBUTTONDOWN:
                print "bgr: (%d, %d, %d) \nhsv: (%d, %d, %d)" % (b,g,r,h,s,v)

    def  HSVRGBPicker(self, window):
        self.GetBGRImageFrame()
        cv2.setMouseCallback(window,self.mouse_callback)

    def convertDepthImageToGrayImage(self):
        gray_depth_frame = self.depth_frame.copy()
        self.DepthHSV[...,0] = gray_depth_frame 
        self.DepthHSV[...,1] = 0x9F
        self.DepthHSV[...,2] = 0xFF
        gray_depth_frame = cv2.cvtColor(self.DepthHSV, cv2.COLOR_HSV2BGR)
        gray_depth_frame = cv2.cvtColor(gray_depth_frame, cv2.COLOR_BGR2GRAY)
        return gray_depth_frame       

def main():
    BlockDetector = DetectBlockClass()

    cv2.namedWindow("Image", cv2.WINDOW_GUI_EXPANDED)

    counter = 0
    Data = []
    #This section is to pick HSV points.
    BlockDetector.HSVRGBPicker("Image")

    ColorLabelText = 'black'
    State = 'Display'
    while True:

            if State == 'Display':
                BlockDetector.GetBGRImageFrame()
                BlockDetector.GetDepthImageFrame()
                BlockDetector.detectBlocksInDepthImage()
                cv2.putText(BlockDetector.bgr_frame, BlockDetector.output_rgb,
                            (10, 20), BlockDetector.font, 0.5, (255, 255, 255))
                cv2.putText(BlockDetector.bgr_frame, BlockDetector.output_hsv,
                            (10, 40), BlockDetector.font, 0.5, (255, 255, 255))
                cv2.putText(BlockDetector.bgr_frame, 'Current color label: '+ColorLabelText,
                            (10, 60), BlockDetector.font, 0.5, (255, 255, 255))
                cv2.putText(BlockDetector.bgr_frame, BlockDetector.output_gray,
                            (10, 80), BlockDetector.font, 0.5, (255, 255, 255))

                BlockDetector.DrawContoursOnFrame(BlockDetector.bgr_frame)
                BlockDetector.DrawRectanglesOnFrame(BlockDetector.bgr_frame)

                ColorImg = BlockDetector.bgr_frame
                GrayImage = cv2.cvtColor(
                    BlockDetector.depth_frame, cv2.COLOR_GRAY2BGR)
                ImageHStack = np.hstack((ColorImg, GrayImage))

                cv2.imshow('Image', ImageHStack)
                State = 'Idle'
                print('State: '+State)

            if State == 'Idle':
                State ='Display'
                ch = 0xFF & cv2.waitKey(10)
                if ch == 27:  #ESC
                    break
                if ch == 32:  #SPACE
                    State = 'LoadNextImage'
                    print('State: '+State)
                if ch == 112:  #p
                    State = 'LoadPrevImage'
                    print('State: '+State)
                if ch == 108:  #l
                    State = 'LoadData'
                    print('State: '+State)
                if ch == 116:  #t
                    State = 'TrainSVM'
                    print('State: '+State)
                if ch == 120:  #x
                    State = 'TestSVM'
                    print('State: '+State)
                if ch == 115:  #s
                    State = 'SaveColorValuesAndLabels'
                    print('State: '+State)
                if ch == 49:   #1
                    State = 'SetLabelBlack'
                    print('State: '+State)
                if ch == 50:   #2
                    State = 'SetLabelRed'
                    print('State: '+State)
                if ch == 51:   #3
                    State = 'SetLabelOrange'
                    print('State: '+State)
                if ch == 52:  #4
                    State = 'SetLabelYellow'
                    print('State: '+State)
                if ch == 53:  #5
                    State = 'SetLabelGreen'
                    print('State: '+State)
                if ch == 54:  #6
                    State = 'SetLabelBlue'
                    print('State: '+State)
                if ch == 55:   #7
                    State = 'SetLabelPurple'
                    print('State: '+State)
                if ch == 56:  #8
                    State = 'SetLabelPink'
                    print('State: '+State)
                if ch == 97:   #a
                    State = 'AppendData'
                    print('State: '+State)
                if ch == 99:    #c
                    State = 'CleanData'
                    print('State: '+State)
                if ch == 103:  #g
                    State = 'SaveModel'
                    print('State: '+State)
                if ch == 104:  #h
                    State = 'LoadModel'
                    print('State: '+State)

            if State == 'SetLabelBlack':
                ColorLabelText = 'black'
                State = 'Display'
                print('State: '+State)

            if State == 'SetLabelRed':
                ColorLabelText = 'red'
                State = 'Display'
                print('State: '+State)
            
            if State == 'SetLabelOrange':
                ColorLabelText = 'orange'
                State = 'Display'
                print('State: '+State)
            
            if State == 'SetLabelYellow':
                ColorLabelText = 'yellow'
                State = 'Display'
                print('State: '+State)

            if State == 'SetLabelGreen':
                ColorLabelText = 'green'
                State = 'Display'
                print('State: '+State)

            if State == 'SetLabelBlue':
                ColorLabelText = 'blue'
                State = 'Display'
                print('State: '+State)

            if State == 'SetLabelPurple':
                ColorLabelText = 'purple'
                State = 'Display'
                print('State: '+State)

            if State == 'SetLabelPink':
                ColorLabelText = 'pink'
                State = 'Display'
                print('State: '+State)


            if State == 'LoadNextImage':
                counter = counter + 1
                BlockDetector.RGBImageFileName = './color_data/cdata_09262019_2012/RGBImage' + \
                    str(counter)+'.png'
                BlockDetector.DepthImageFileName = './color_data/cdata_09262019_2012/DepthImage' + \
                    str(counter)+'.csv'
                BlockDetector.Loadimages()
                State = 'Display'
                print('State: '+State)

            if State == 'LoadPrevImage':
                counter = counter - 1
                if counter<0:
                    counter =0
                BlockDetector.RGBImageFileName = './color_data/cdata_09262019_2012/RGBImage' + \
                    str(counter)+'.png'
                BlockDetector.DepthImageFileName = './color_data/cdata_09262019_2012/DepthImage' + \
                    str(counter)+'.csv'
                BlockDetector.Loadimages()
                State = 'Display'
                print('State: '+State)

            

            if State== 'AppendData':
                for each_point in BlockDetector.color_mean_list:
                    Data.append([each_point, BlockDetector.ColorLabel[ColorLabelText]])
                print(Data)
                Action ='Data appended.'
                print('Action : '+Action)
                State = 'Display'
                print('State: '+State)

            if State== 'CleanData':
                Data = []
                Action ='Data cleared.'
                print('Action : '+Action)
                State = 'Display'
                print('State: '+State)

                
            if State == 'SaveColorValuesAndLabels': 
                df = pd.DataFrame(Data, columns =['ColorPoint','ColorClass'])
                print(df.head())
                df.to_pickle('./ColorTrainData.pkl')
                Action = 'Set saved.'
                print('Action : ' + Action)
                State = 'Display'
                print('State: '+State)

            if State == 'LoadData':
                df = pd.read_pickle('./ColorTrainData.pkl')
                print(df.head())
                Action = 'Color data restored.'
                print('Action : ' + Action)
                State = 'Display'
                print('State: '+State)

            if State == 'TrainSVM':
                SVMmodel = SVC(C=1.0, gamma=0.01, kernel='poly', degree=5, verbose=True, decision_function_shape = 'ovr')
                Y = np.array(df['ColorClass'].values)
                X = np.array([each_value for each_value in df['ColorPoint'].values])
                SVMmodel.fit(X,Y)
                Action = 'SVM trained.'
                print('Action : ' + Action)
                State = 'Display'
                print('State: '+State)

            if State ==  'TestSVM':
                for each_point in BlockDetector.color_mean_list:
                    pred = SVMmodel.predict([each_point])
                    print(BlockDetector.ColorlabelText[pred[0]])

                Action = 'SVM tested..'
                print('Action : ' + Action)
                State = 'Display'
                print('State: '+State)

            if State ==  'SaveModel':
                pickle.dump(SVMmodel, open('./SVMmodel.pkl','wb'))
                Action = 'SVM model saved.'
                print('Action : ' + Action)
                State = 'Display'
                print('State: '+State)

            if State ==  'LoadModel':
                SVMmodel = pickle.load(open('./SVMmodel.pkl','rb'))
                Action = 'SVM model restored.'
                print('Action : ' + Action)
                State = 'Display'
                print('State: '+State)
            

    
    #IPython.embed()


    cv2.destroyAllWindows()

if __name__=="__main__":
    main()



