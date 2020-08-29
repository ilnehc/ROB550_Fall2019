
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import matplotlib.pyplot as plt
import numpy as np
import pandas as  pd
import cv2

import IPython


class ProcessColorComponentsClass():
    def __init__(self):

        self.DataDirPath = './color_data/cdata09252019_1423/'
        
        self.ColorDataFilenames= [  'HSVBlack.txt',
                                    'HSVRed.txt',
                                    'HSVOrange.txt',
                                    'HSVYellow.txt',
                                    'HSVGreen.txt',
                                    'HSVBlue.txt',
                                    'HSVPurple.txt',
                                    'HSVPink.txt' ]

        self.ColorNameList = [ "Black",
                                "Red",
                                "Orange",
                                "Yellow",
                                "Green",
                                "Blue",
                                "Purple",
                                "Pink" ]           

        
        self.dfList = []
        for each_filename, each_colorname in zip(self.ColorDataFilenames,self.ColorNameList):
            df = pd.read_csv( self.DataDirPath+each_filename, delimiter=':', names=['color_space','color_data'])
            total_elements = df.shape[0]
            df_additional = pd.DataFrame([each_colorname for i in range(0, total_elements)], columns=['color_name'])
            df = df.join(df_additional)
            self.dfList.append(df)
        self.Datadf = pd.concat(self.dfList, axis=0, ignore_index=True)
    

        self.Data = self.Datadf[self.Datadf['color_space']=='hsv']['color_data'].values
        self.Data = [ each.replace('(','') for each in self.Data]
        self.Data = [ each.replace(')','') for each in self.Data]
        self.Data = np.array([ map(int,each.split(',')) for each in self.Data])

        #Spliting HSV components.
        self.h = [ val[0] for val in self.Data]
        self.s = [ val[1] for val in self.Data]
        self.v = [ val[2] for val in self.Data]

        self.RGBData = self.Datadf[self.Datadf['color_space']=='bgr']['color_data'].values
        self.RGBData = [ each.replace('(','') for each in self.RGBData]
        self.RGBData = [ each.replace(')','') for each in self.RGBData]
        self.RGBData = np.array([ map(int,each.split(',')) for each in self.RGBData])
        self.RGBData = self.RGBData[:,[2,1,0]]
        self.pixel_colors = self.RGBData.copy()
        norm = colors.Normalize(vmin=-1.,vmax=1.)
        norm.autoscale(self.pixel_colors)
        self.pixel_colors = norm(self.pixel_colors).tolist()
     
        #Statistics on each block color.
        self.ColorData = self.Datadf.copy()
        ColorNumericalValues = self.ColorData['color_data'].values
        ColorNumericalValues = [ each.replace('(','') for each in ColorNumericalValues]
        ColorNumericalValues = [ each.replace(')','') for each in ColorNumericalValues]
        ColorNumericalValues = np.array([ map(int,each.split(',')) for each in ColorNumericalValues])
        Temporaldf = pd.DataFrame(ColorNumericalValues, columns =['r/h','g/s','b/v'] )
        self.ColorData = self.ColorData.join(Temporaldf)        
                                            

    def DrawHSVSpace(self):
        fig = plt.figure()
        axis = fig.add_subplot(1,1,1, projection = "3d")
        axis.scatter(self.h, self.s, self.v,facecolors=self.pixel_colors, marker="X",  edgecolors=(0, 0, 0, 0.5))
        axis.set_xlabel("Hue")
        axis.set_ylabel("Saturation")
        axis.set_zlabel("Value")
        plt.show()

    def ShowStats(self):
        for each_color in self.ColorNameList:
            Hmax = self.ColorData[ (self.ColorData['color_space']=='hsv') & ( self.ColorData['color_name']==each_color) ]['r/h'].values.max()
            Hmin = self.ColorData[ (self.ColorData['color_space']=='hsv') & ( self.ColorData['color_name']==each_color) ]['r/h'].values.min()
            Smax = self.ColorData[ (self.ColorData['color_space']=='hsv') & ( self.ColorData['color_name']==each_color) ]['g/s'].values.max()
            Smin = self.ColorData[ (self.ColorData['color_space']=='hsv') & ( self.ColorData['color_name']==each_color) ]['g/s'].values.min()
            Vmax = self.ColorData[ (self.ColorData['color_space']=='hsv') & ( self.ColorData['color_name']==each_color) ]['b/v'].values.max()
            Vmin = self.ColorData[ (self.ColorData['color_space']=='hsv') & ( self.ColorData['color_name']==each_color) ]['b/v'].values.min()
            Rmax = self.ColorData[ (self.ColorData['color_space']=='bgr') & ( self.ColorData['color_name']==each_color) ]['r/h'].values.max()
            Rmin = self.ColorData[ (self.ColorData['color_space']=='bgr') & ( self.ColorData['color_name']==each_color) ]['r/h'].values.min()
            Gmax = self.ColorData[ (self.ColorData['color_space']=='bgr') & ( self.ColorData['color_name']==each_color) ]['g/s'].values.max()
            Gmin = self.ColorData[ (self.ColorData['color_space']=='bgr') & ( self.ColorData['color_name']==each_color) ]['g/s'].values.min()
            Bmax = self.ColorData[ (self.ColorData['color_space']=='bgr') & ( self.ColorData['color_name']==each_color) ]['b/v'].values.max()
            Bmin = self.ColorData[ (self.ColorData['color_space']=='bgr') & ( self.ColorData['color_name']==each_color) ]['b/v'].values.min()
            print('{:}, [[Hmin, Smin, Vmin], [Hmax, Smax, Vmax]], [[Rmin, Gmin, Bmin], [Rmax, Gmax, Bmax]],  [[{:d}, {:d}, {:d}], [{:d}, {:d}, {:d}]], [[{:d}, {:d}, {:d}], [{:d}, {:d}, {:d}]]'.format(each_color, Hmin, Smin, Vmin, Hmax, Smax, Vmax, Rmin, Gmin, Bmin, Rmax, Gmax, Bmax))
            
            Hmean = self.ColorData[ (self.ColorData['color_space']=='hsv') & ( self.ColorData['color_name']==each_color) ]['r/h'].values.mean()
            Smean = self.ColorData[ (self.ColorData['color_space']=='hsv') & ( self.ColorData['color_name']==each_color) ]['g/s'].values.mean()
            Vmean = self.ColorData[ (self.ColorData['color_space']=='hsv') & ( self.ColorData['color_name']==each_color) ]['b/v'].values.mean()
            print('Hmean: {:f}  Smean: {:f}  Vmean: {:f}'.format(Hmean, Smean, Vmean))

            Hvar = self.ColorData[ (self.ColorData['color_space']=='hsv') & ( self.ColorData['color_name']==each_color) ]['r/h'].values.var()
            Svar = self.ColorData[ (self.ColorData['color_space']=='hsv') & ( self.ColorData['color_name']==each_color) ]['g/s'].values.var()
            Vvar = self.ColorData[ (self.ColorData['color_space']=='hsv') & ( self.ColorData['color_name']==each_color) ]['b/v'].values.var()
            print('Hvar: {:f}  Svar: {:f}  Vvar: {:f}'.format(np.sqrt(Hvar), np.sqrt(Svar), np.sqrt(Vvar)))

            

def main():
    ColorProcessor = ProcessColorComponentsClass()

    #ColorProcessor.DrawHSVSpace()
    ColorProcessor.ShowStats()



if __name__=="__main__":
    main()