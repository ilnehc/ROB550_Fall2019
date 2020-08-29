import cv2
import numpy as np
from PyQt4.QtGui import QImage
import freenect
from sklearn.svm import SVC
import pickle
import IPython

class Kinect():
    def __init__(self):
        self.currentVideoFrame = np.array([])
        self.currentDepthFrame = np.array([])
        if(freenect.sync_get_depth() == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True
        
        # mouse clicks & calibration variables
        self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
        self.kinectCalibrated = False
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5,2),int)
        self.depth_click_points = np.zeros((5,2),int)

        self.PointCamera_last_click = np.array([0,0,0])
        self.PointWorld_last_click  = np.array([0,0,0])
        self.BlockOrientation_last_click = 0

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])

        self.ROIRect = np.array([[132,95],[475,440]])
        self.Exclusion = np.array([[270,200],[360,285]])
        self.kernel = np.ones((3,3),np.uint8)

        self.ImageDataDirPath = './color_data/cdata_09262019_2012/'
        self.RGBFileNamePrefix      = 'RGBImage'
        self.DepthFileNamePrefix   = 'DepthImage'
        self.SufixCounter = 0

        self.RGBImageLoaded = False
        self.DepthImageLoaded = False

        self.center_list = np.array([])
        self.center_list_in_image_plane = np.array([])
        self.orientation_list = np.array([])
        self.color_list = np.array([])
        self.SpecifcColor2Draw = 0

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

        self.ColorTableDraw = {} 
        self.ColorTableDraw[0] = (0,0,0)
        self.ColorTableDraw[1] =  (255,0,0)
        self.ColorTableDraw[2] = (255,69,0)
        self.ColorTableDraw[3] = (255,255,0)
        self.ColorTableDraw[4] = (0,255,0)
        self.ColorTableDraw[5] = (0,0,255)
        self.ColorTableDraw[6] = (128,0,128)
        self.ColorTableDraw[7] = (255,0,255)

        """ block info """
        self.block_contours = np.array([])
        
        try:      
            self.Depthhomography = np.loadtxt('DepthRegistrationMatrix.txt',delimiter=',')
            self.kinectCalibrated = True
        except:
            self.kinectCalibrated = False
            print('Unable to find depth calibration information.')
            
        try:
            self.IntrinsicMatrix = np.loadtxt('./util/intrinsic_matrix.txt',delimiter=',')
            self.rvec = np.loadtxt('./util/extrinsic_rvec.txt', delimiter=',')
            R = cv2.Rodrigues(self.rvec)
            self.Rcam = R[0] 
            self.Tcam = np.loadtxt('./util/extrinsic_tvec.txt', delimiter=',')
            self.Tvec = self.Tcam 
        except:
            print('Camera calibration data cannot be loaded.')


        try:
            self.ROIRect = np.int16(np.loadtxt('./ROIRect.txt',delimiter=','))
            self.Exclusion = np.int16(np.loadtxt('./Exclusion.txt',delimiter=','))
        except:
            print('ROI areas could not be loaded.')


        try:
            self.SVMmodel = pickle.load(open('./util/SVMmodel.pkl','rb'))
        except:
            print('Error loading the model.')

    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        if(self.kinectConnected):
            self.currentVideoFrame = freenect.sync_get_video()[0]
        else:
            self.loadVideoFrame()
    
    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            if(self.kinectCalibrated):
                self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
            else:
                self.currentDepthFrame = freenect.sync_get_depth()[0]
        else:
            self.loadDepthFrame()

    def loadVideoFrame(self):
        if self.RGBImageLoaded == False:
            self.currentVideoFrame = cv2.cvtColor(
                cv2.imread('./util/color_data/cdata_09262019_2012/RGBImage130.png',cv2.IMREAD_UNCHANGED),
                cv2.COLOR_BGR2RGB
                )
            self.RGBImageLoaded = True

    def loadDepthFrame(self):
        if self.DepthImageLoaded == False:
            self.currentDepthFrame = np.loadtxt('./util/color_data/cdata_09262019_2012/DepthImage130.csv', delimiter=',') 
            self.DepthImageLoaded = True

    def processVideoFrame(self):
        self.DrawCenterOfMassOnFrame(self.VideoCM )
        #self.DrawRectanglesOnFrame(self.VideoCM )
        #self.DrawRectanglesForColorDetection(self.VideoCM)
        self.DrawSpecificRectangleColor(self.VideoCM)
    
    def processDepthFrame(self):
        self.DrawRawContours(self.DepthCM)


    def convertFrame(self):
        """ Converts frame to format suitable for Qt  """
        try:
            self.VideoCM = self.currentVideoFrame.copy()
            self.processVideoFrame()
            img = QImage(self.VideoCM,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for Qt  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """ 
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
            self.processDepthFrame()
            img = QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """
        Given 2 sets of corresponding coordinates, 
        find the affine matrix transform between them.

        TODO: Rewrite this function to take in an arbitrary number of coordinates and 
        find the transform without using cv2 functions
        """
        # coord1 = depth_points, coord2 = image_points
        #self.Depthhomography, self.Depthomography_status = cv2.findHomography(coord1, coord2)
        #IPython.embed()
        CustomHomographyA =  np.zeros( (coord1.shape[0]*2,6))
        pt=0
        for i in range(0,coord1.shape[0]*2,2):
            CustomHomographyA[i][0]=coord1[pt][0]
            CustomHomographyA[i][1]=coord1[pt][1]
            CustomHomographyA[i][2]=1.0
            CustomHomographyA[i+1][3]=coord1[pt][0]
            CustomHomographyA[i+1][4]=coord1[pt][1]
            CustomHomographyA[i+1][5]=1.0
            pt=pt+1 
            
        CustomHomographyB= np.zeros( (coord1.shape[0]*2,1))
        pt=0
        for i in range(0,coord1.shape[0]*2,2):
            CustomHomographyB[i][0]=coord2[pt][0]
            CustomHomographyB[i+1][0]=coord2[pt][1]
            pt=pt+1
        CustomHomographyA_T =np.transpose(CustomHomographyA)
        CustomHomographyCoeff = np.dot(np.matmul(np.linalg.inv(np.matmul(CustomHomographyA_T,CustomHomographyA)),CustomHomographyA_T),CustomHomographyB)
        CustomHomography = np.eye(3)
        CustomHomography[0:2,0:3]=CustomHomographyCoeff.reshape(2,3)        
        self.Depthhomography = CustomHomography
        np.savetxt('DepthRegistrationMatrix.txt',self.Depthhomography,delimiter=',')
        return self.Depthhomography


    def registerDepthFrame(self, frame):
        """
        TODO: Using an Affine transformation, transform the depth frame to match the RGB frame
        """
        RegisteredDepthFrame = cv2.warpPerspective(frame, self.Depthhomography, (frame.shape[1], frame.shape[0]))      
        return RegisteredDepthFrame

    def loadCameraCalibration(self):
        """
        TODO: Load camera intrinsic matrix from file.
        """
        try:
            self.IntrinsicMatrix = np.loadtxt('./util/intrinsic_matrix.txt',delimiter=',')
            rvec = np.loadtxt('./util/extrinsic_rvec.txt', delimiter=',')
            R = cv2.Rodrigues(rvec)
            self.Rcam = R[0] 
            self.Tcam = np.loadtxt('./util/extrinsic_tvec.txt', delimiter=',')
        except:
            print('Camera calibration data cannot be loaded.')
            
    
    def blockDetector(self):
        """
        TODO: Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """
        pass

    def detectBlocksInDepthImage(self):
        """
        TODO: Implement a blob detector to find blocks
        in the depth image
        """
        self.convertDepthImageToGrayImage()
        self.GetBlockContoursDepth(self.gray_depth_frame)
    #***********************************************
    
    def PixelToDepth(self, value):
        return (0.1236 * np.tan(value/2842.5+1.1863))
    
    def ConvertImagePointToCameraFrame(self,ImagePoint):
        self.captureDepthFrame()
        Zc=self.PixelToDepth(self.currentDepthFrame[ImagePoint[1], ImagePoint[0]])
        ImagePointHomogeneous = cv2.convertPointsToHomogeneous(np.array([ImagePoint]))
        PointInCameraFrame = Zc*np.matmul(np.linalg.inv(self.IntrinsicMatrix), ImagePointHomogeneous[0][0])
        return PointInCameraFrame
    
    def ConvertCameraFrametoWorlFrame(self,CameraFramePoint):
        Tx = -259.561
        Ty = 143.983
        Tz = 0.0
        local_CameraFramePoint = np.matrix(CameraFramePoint*1000).transpose()
        local_CameraFramePointH = np.ones((4,1))
        local_CameraFramePointH[:3] = local_CameraFramePoint[:]
        #WorldPoint = np.matmul(np.linalg.inv(self.Rcam),local_CameraFramePoint)-np.matrix((np.matmul(np.linalg.inv(self.Rcam),self.Tcam ))).transpose()
        Extrinsics = np.eye(4)
        Extrinsics[:3,:3] = self.Rcam[:,:]
        Extrinsics[:3,3] = self.Tcam[:]
        #Additonal rotation and translation
        R = self.Rotx(np.pi)
        T = np.array([Tx,Ty,Tz])
        H_world2BaseLink = np.eye(4)
        H_world2BaseLink[:3,:3] = R[:,:]
        H_world2BaseLink[:3,3] = np.matmul(R,T)[:] 
        WorldPoint  = np.matmul( np.linalg.inv(Extrinsics), local_CameraFramePointH) 
        WorldPoint = WorldPoint/WorldPoint[3] 
        BaseLinkPoint  = np.matmul(H_world2BaseLink,WorldPoint) - np.matrix([0.0,0.0,-27.0,0.0]).transpose()
        return BaseLinkPoint[:3]
        #return WorldPoint[:3]

    def GetBlockContoursDepth(self, DepthImage):
        GrayLimits = np.array([ [180,200],   #First level
                                [142, 175 ]  #Second level
                                ])

        AreaLimit  = np.array([
                                [350,570],
                                [500,700]
                            ])
        
        countor_list = [] 
        self.center_list = []
        self.center_list_in_image_plane =[]
        self.orientation_list =[]
        orientation_lst = []
        wrld_center_list = [] 
        img_center_list = []
        for i in range(GrayLimits.shape[0]):
            GrayLow_limit = GrayLimits[i,0]
            GrayUpper_limit = GrayLimits[i,1]
            localDepthImage = DepthImage.copy()
            cv2.rectangle(localDepthImage, (self.Exclusion[0,0], self.Exclusion[0,1]), (self.Exclusion[1,0], self.Exclusion[1,1]), (255, 255, 255), cv2.FILLED)
            ROI_image = localDepthImage[self.ROIRect[0,1]: self.ROIRect[1,1],self.ROIRect[0,0]: self.ROIRect[1,0]].copy()
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
                    rect = cv2.minAreaRect(each_contour+self.ROIRect[0])
                    orientation_lst.append(abs(rect[2]))
                    M = cv2.moments(each_contour)
                    try:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                    except:
                        print('Error division by zero.')
                    PointCamera = self.ConvertImagePointToCameraFrame(np.array([cx+self.ROIRect[0,0],cy+self.ROIRect[0,1]]))
                    PointWorld = self.ConvertCameraFrametoWorlFrame(PointCamera)
                    wrld_center_list.append(PointWorld)
                    img_center_list.append(np.array([cx+self.ROIRect[0,0],cy+self.ROIRect[0,1]]))
        
        self.orientation_list = np.array(orientation_lst)/57.295827909
        self.center_list = np.array(wrld_center_list)
        self.center_list_in_image_plane = np.array(img_center_list)
        self.block_contours= np.array(countor_list)
        
        


    def SelectBlock(self,Point):
        index = 0
        result = -1
        block_contours_local = self.block_contours[:]
        for each_contour in block_contours_local:
            dist = cv2.pointPolygonTest(each_contour,(Point[0],Point[1]),True)
            if dist >= 0 :
                result = index
                break
            index = index +1  
        return result

    def GetBlockInfo(self, index):
        center_list_local = self.center_list[:]
        orientation_list_local = self.orientation_list[:]
        if index < len(center_list_local):
            return center_list_local[index], orientation_list_local[index]
        return  np.array([0.0, 0.0, 0.0]), 0.0

    def GetWorldCubeCenters(self):
        return np.array(self.center_list)

    def IdentifyColors(self):
        #ImageFrame = self.currentVideoFrame.copy() 
        ImageFrame = cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2BGR)
        color_list = []
        for each_contour in self.block_contours:
            mask = np.zeros(ImageFrame.shape[0:2], np.uint8)
            cv2.drawContours(mask, np.array(
                [each_contour]), -1, 255, -1)
            mean = cv2.mean(ImageFrame, mask=mask)
            pred = self.SVMmodel.predict([mean])
            color_list.append(pred)
        self.color_list = np.array(color_list)
        
        return self.color_list

    def DrawCenterOfMassOnFrame(self, ImageFrame):
        for each_contour in  self.center_list_in_image_plane:
            cv2.circle(ImageFrame, (each_contour[0],each_contour[1]), 2, (0,0,0),2)

    def DrawRectanglesOnFrame(self, ImageFrame):
        for each_contour in  self.block_contours:
            try:
                rect = cv2.minAreaRect(each_contour)
                box = cv2.boxPoints(rect) # cv2.boxPoints(rect) for OpenCV 3.x
                box = np.int0(box)
                cv2.drawContours(ImageFrame,[box],0,(0,0,255),2)
            except:
                print('Error division by zero.')
    
    def DrawRectanglesForColorDetection(self, ImageFrame):
        for each_contour,each_color in  zip(self.block_contours,self.color_list):
            try:
                rect = cv2.minAreaRect(each_contour)
                box = cv2.boxPoints(rect) # cv2.boxPoints(rect) for OpenCV 3.x
                box = np.int0(box)
                cv2.drawContours(ImageFrame,[box],0,self.ColorTableDraw[each_color[0]],2)
            except:
                print('Error division by zero.')

    def DrawSpecificRectangleColor(self,ImageFrame):
        for each_contour, each_color in  zip(self.block_contours, self.color_list):
            if (self.SpecifcColor2Draw == each_color[0] ):
                try:
                    rect = cv2.minAreaRect(each_contour)
                    box = cv2.boxPoints(rect) # cv2.boxPoints(rect) for OpenCV 3.x
                    box = np.int0(box)
                    cv2.drawContours(ImageFrame,[box],0,self.ColorTableDraw[each_color[0]],2)
                except:
                    print('Error division by zero.')
                break

    def GetBlockIndexBasedOnColor(self, color):
        result = -1
        index =0
        for each_color in self.color_list:
            if (color == each_color[0]):
                result = index
                break
            else:
                index = index +1 
        return result      

    def DrawRawContours(self, ImageFrame):
        cv2.drawContours(ImageFrame,self.block_contours,-1,(0,0,0),3)
    
    def SETROI(self,imagepoints):
        self.ROIRect = imagepoints
        np.savetxt('./ROIRect.txt',self.ROIRect,delimiter=',')

    def SETExclusion(self,imagepoints):
        self.Exclusion = imagepoints
        np.savetxt('./Exclusion.txt', self.Exclusion,delimiter=',')

    def convertDepthImageToGrayImage(self):
    
        self.gray_depth_frame = self.currentDepthFrame.copy()
        self.DepthHSV[...,0] = self.gray_depth_frame
        self.DepthHSV[...,1] = 0x9F
        self.DepthHSV[...,2] = 0xFF
        self.gray_depth_frame = cv2.cvtColor(self.DepthHSV, cv2.COLOR_HSV2BGR)
        self.gray_depth_frame = cv2.cvtColor(self.gray_depth_frame, cv2.COLOR_BGR2GRAY)
    
        #self.gray_depth_frame = cv2.normalize(self.gray_depth_frame , dst=None, alpha=0, beta=2048, norm_type=cv2.NORM_MINMAX)
        return self.gray_depth_frame

    def SuperposeRGBandDepth(self):
        alpha = 0.5
        beta = (1.0 - alpha)
        self.DepthHSV[...,0] = self.currentDepthFrame
        self.DepthHSV[...,1] = 0x9F
        self.DepthHSV[...,2] = 0xFF
        DepthColor = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
        dst = cv2.addWeighted(self.currentVideoFrame, alpha, DepthColor , beta, 0.0)
        try:
            img = QImage(dst,
                             dst.shape[1],
                             dst.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None
        return dst

    def GetLevelImage(self):
            cvimage = self.currentVideoFrame.copy()
            cv2.line(cvimage, (0,435), (639,435), (255,0,0), 1) 
            cv2.line(cvimage, (130,0), (130,479), (255,0,0), 1) 
            cv2.line(cvimage, (0,400), (639,400), (0,0,255), 1) 
            cv2.line(cvimage, (150,0), (150,479), (0,0,255), 1) 
            level_image = QImage(cvimage,
                             cvimage.shape[1],
                             cvimage.shape[0],
                             QImage.Format_RGB888
                             )
            return level_image

    def SaveBGRandDepthFrames(self):
        self.ImageDataDirPath = './util/color_data/cdata_09262019_2012/'
        self.RGBFileNamePrefix      = 'RGBImage'
        self.DepthFileNamePrefix   = 'DepthImage'
        FullFilename =self.ImageDataDirPath+self.RGBFileNamePrefix+str(self.SufixCounter)+'.png'
        cv2.imwrite(FullFilename, cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2BGR))
        FullFilename = self.ImageDataDirPath+self.DepthFileNamePrefix+str(self.SufixCounter)+'.csv'
        np.savetxt(FullFilename, self.currentDepthFrame, delimiter=',')
        self.SufixCounter = self.SufixCounter + 1

    def Rotz(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        Rz =  np.matrix([ 
                        [c, -s, 0.0],
                        [s, c, 0.0],
                        [0.0, 0.0, 1.0]
                        ])
        return Rz
    
    def Roty(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        Ry =  np.matrix([ 
                        [c, 0.0, s],
                        [0.0, 1.0, 0.0],
                        [-s, 0.0, c]
                        ])
        return Ry

    def Rotx(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        Rx =  np.matrix([ 
                        [1.0, 0.0, 0.0],
                        [0.0, c, -s],
                        [0.0, s, c]
                        ])
        return Rx
