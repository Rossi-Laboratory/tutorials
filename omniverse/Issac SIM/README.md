# Image Sensing Pipeline
Please follow the [Instruction](https://drive.google.com/uc?export=view&id=1WKVSxp5u5RqL9og7omnQMdO6nHquGhRP) to complete this assignment.

Download the [ipynb file](https://drive.google.com/uc?export=view&id=1LwV17gO5K-Wx72kfa6lwzPIbuvHDQ1hx), [`tone_curves.mat`](https://drive.google.com/uc?export=view&id=1S6cybmirw5ewXt1gKgwz-ui76fCr5pI4), and [`tone_curves_inv.mat`](https://drive.google.com/uc?export=view&id=17GVpWP4kYOUvvYTHH54h3eVxvXK7sSkL), and follow the TODOs to complete the program.

Ensure that all files are placed in the same folder directory.

Submit your files on [E3](https://portal.nycu.edu.tw/#/): the code and the report (covering four questions). Provide the submissions in both .ipynb and PDF formats, respectively.


<img style="float: left;" src="https://drive.google.com/uc?export=view&id=18DPd-oXCprb8RBiCNJNdRnDZxQIqHRHg" width="80%">

# In this homework ¡K¡K

<img style="float: left;" src="https://drive.google.com/uc?export=view&id=1jvmCDrFQZLQ87uUvEKpihHPIRwzIsYvM" width="80%">


```python
import os
import cv2
import math
import random
import scipy.io
import numpy as np
import matplotlib.pyplot as plt
%matplotlib inline
```

# 1. Camera  Parameters

## 1.1 Import Data from tone_curves


```python
curve_path = './'
curve_name = os.path.join(curve_path, 'tone_curves.mat')
curve_inv_name = os.path.join(curve_path, 'tone_curves_inv.mat')
tone_curves = scipy.io.loadmat(curve_name)
tone_curves_inv = scipy.io.loadmat(curve_inv_name)
```


```python
I = tone_curves['I'] # Irradiance
B = tone_curves['B'] # Brightness
I_inv = tone_curves_inv['invI']
B_inv = tone_curves_inv['invB']
```


```python
plt.figure(figsize=(18, 6))
plt.subplot(1, 2, 1)
plt.axis('off')
plt.imshow(I)
plt.title('Irradiance')
plt.subplot(1, 2, 2)
plt.axis('off')
plt.imshow(B)
plt.title('Brightness')
```




    Text(0.5, 1.0, 'Brightness')




![png](https://drive.google.com/uc?export=view&id=1c7H3CYk8dlfguJbstANywiO9pyjuzFjM)



```python
# Tone Curves Parameters
tone_index = 170
B[tone_index]
```




    array([  0.00000000e+00,   5.76713282e-05,   1.16315903e-04, ...,
             9.95653689e-01,   9.97825503e-01,   1.00000000e+00], dtype=float32)



## 1.2 Color Correction Matrix


```python
ccm = np.array([1.0234, -0.2969, -0.2266, 
                -0.5625, 1.6328, -0.0469, 
                -0.0703, 0.2188, 0.6406])
ccm = np.reshape(ccm, (3, 3))
ccm = (ccm / np.tile(np.sum(ccm, axis=1), [3, 1]).T).T
ccm_inv = np.linalg.inv(np.copy(ccm))
```

## 1.3 White Balance


```python
# White Balance Parameters
fr = 0.7715567349551743
fb = 0.9068480239589546
```

# 2.  Load Image


```python
img = cv2.imread('.//images/1.png')
plt.figure(figsize=(12, 6))
plt.axis('off')
plt.imshow(img)
```




    <matplotlib.image.AxesImage at 0x1b4f88432e8>




![png](https://drive.google.com/uc?export=view&id=19nKPIjs-MX-Xkf7zFY3h7-n0rvhJvMl5)



```python
img = cv2.imread('.//images/1.png')
img_gt = img
np.array(img, dtype='uint8')
img = img.astype('double') / 255.0

#### Remember that the image store in OpenCV is BGR instead of RGB
#### We should transfer to RGB first before ISP
plt.figure(figsize=(12, 6))
img = BGR2RGB(img)
plt.axis('off')
plt.imshow(img)
```




    <matplotlib.image.AxesImage at 0x1b4f897a710>




![png](https://drive.google.com/uc?export=view&id=1ijy7U5oVGotircIY-00LBGcNN9TBXsHy)


# 3. Inverse ISP Process

## 3.1 Tone Mapping


```python
import numpy as np
import math

def tone_mapping(img, I, B, index=0, inv=False):
    '''
    Input:
        img: H*W*3 numpy array, input image.
        I: 201*1024 array, represents 201 tone curves for Irradiance.
        B: 201*1024 array, represents 201 tone curves for Brightness.
        index: int, choose which curve to use, default is 0
        inv: bool, judge whether tone mapping (False) or inverse tone mapping (True), default is False
    Output:
        output: H*W*3 numpy array, output image afte (inverse) tone mapping.
    '''
    ########################################################################
    # TODO:                                                                #
    #   Following above instruction to get tone mapping as output.         #
    #   and inverse tone mapping result as output                          #
    ########################################################################
    

    
    ########################################################################
    #                                                                      #
    #                           End of your code                           #
    #                                                                      # 
    ########################################################################

    return output
```


```python
img_inverse_tone = tone_mapping(img, I_inv, B_inv, index=tone_index, inv=True)
```


```python
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.axis('off')
plt.imshow(img)
plt.title('Oringal Image')
plt.subplot(1, 2, 2)
plt.axis('off')
plt.imshow(img_inverse_tone)
plt.title('Image after inverse tone')
```




    Text(0.5, 1.0, 'Image after inverse tone')




![png](https://drive.google.com/uc?export=view&id=1Fn_6JAfpHshbOxFngMmctPaEqMdlKfzi)


## 3.2 from RGB to CIE XYZ
The CIE XYZ color space is a more universal color representation method that is independent of device characteristics and closer to the perception of the **human visual system**. The CIE XYZ color space is composed of three color components: X, Y, and Z. Among these, X and Y represent the **brightness** and **saturation** of the color, while Z represents the **hue** of the color.

<img style="float: left;" src="https://drive.google.com/uc?export=view&id=1-jpaVvYfWpsDBqqrkclCdepoT5GrSbwZ" width="40%">


```python
#from color_correction import RGB2XYZ
img_inverse_tone_XYZ = RGB2XYZ(img_inverse_tone)
```


```python
plt.figure(figsize=(12, 6))
plt.subplot(1, 3, 1)
plt.axis('off')
plt.imshow(img)
plt.title('Oringal Image')
plt.subplot(1, 3, 2)
plt.axis('off')
plt.imshow(img_inverse_tone)
plt.title('Image after inverse tone')
plt.subplot(1, 3, 3)
plt.axis('off')
plt.imshow(img_inverse_tone_XYZ)
plt.title('Image after CIE')
```

    Clipping input data to the valid range for imshow with RGB data ([0..1] for floats or [0..255] for integers).
    




    Text(0.5, 1.0, 'Image after CIE')




![png](https://drive.google.com/uc?export=view&id=14uI7BX88Yb73RQ5W5BN0BC-weC58glEy)


## 3.3. Color Correction


```python
def color_correction(img, ccm):
    '''
    Input:
        img: H*W*3 numpy array, input image
        ccm: 3*3 numpy array, color correction matrix 
    Output:
        output: H*W*3 numpy array, output image after color correction
    '''
    ########################################################################
    # TODO:                                                                #
    #   Following above instruction to get color correction result         #
    #                                                                      #
    ########################################################################
    


    ########################################################################
    #                                                                      #
    #                           End of your code                           #
    #                                                                      # 
    ########################################################################
    
    #### Prevent the value larger than 1 or less than 0
    output = np.clip(output, 0, 1)
    return output
```


```python
ccm
```




    array([[ 2.04720944, -0.54963846, -0.08908884],
           [-0.59391878,  1.59546609,  0.27727791],
           [-0.45329066, -0.04582763,  0.81181092]])




```python
img_inverse_ccm = color_correction(img_inverse_tone_XYZ, ccm)
```


```python
plt.figure(figsize=(14, 6))
plt.subplot(1, 4, 1)
plt.axis('off')
plt.imshow(img)
plt.title('Oringal Image')
plt.subplot(1, 4, 2)
plt.axis('off')
plt.imshow(img_inverse_tone)
plt.title('Image after inverse tone')
plt.subplot(1, 4, 3)
plt.axis('off')
plt.imshow(img_inverse_tone_XYZ)
plt.title('Image after CIE')
plt.subplot(1, 4, 4)
plt.axis('off')
plt.imshow(img_inverse_ccm)
plt.title('Image after CCM')
```

    Clipping input data to the valid range for imshow with RGB data ([0..1] for floats or [0..255] for integers).
    




    Text(0.5, 1.0, 'Image after CCM')




![png](https://drive.google.com/uc?export=view&id=1Dcw4xS_jVJkCIA0OVsggxIppnBSsVFaT)


## 3.4 Mosaic


```python
import colour
from colour.plotting import *

from colour_demosaicing import (
    EXAMPLES_RESOURCES_DIRECTORY,
    demosaicing_CFA_Bayer_bilinear,
    demosaicing_CFA_Bayer_Malvar2004,
    demosaicing_CFA_Bayer_Menon2007,
    mosaicing_CFA_Bayer)

def mosaic(img, pattern):
    '''
    Input:
        img: H*W*3 numpy array, input image.
        pattern: string, 4 different Bayer patterns (GRBG, RGGB, GBRG, BGGR)
    Output:
        output: H*W numpy array, output image after mosaic.
    '''
    ########################################################################
    # TODO:                                                                #
    #   1. Create the H*W output numpy array.                              #   
    #   2. Discard other two channels from input 3-channel image according #
    #      to given Bayer pattern.                                         #
    #                                                                      #
    #   e.g. If Bayer pattern now is BGGR, for the upper left pixel from   #
    #        each four-pixel square, we should discard R and G channel     #
    #        and keep B channel of input image.                            #     
    #        (since upper left pixel is B in BGGR bayer pattern)           #
    ########################################################################
    
    

    ########################################################################
    #                                                                      #
    #                           End of your code                           #
    #                                                                      # 
    ########################################################################

    return output


def demosaic(img, pattern):
    '''
    Input:
        img: H*W numpy array, input RAW image.
        pattern: string, 4 different Bayer patterns (GRBG, RGGB, GBRG, BGGR)
    Output:
        output: H*W*3 numpy array, output de-mosaic image.
    '''
    #### Using Python colour_demosaicing library
    #### You can write your own version, too
    output = demosaicing_CFA_Bayer_Malvar2004(img,pattern)
    output = np.clip(output, 0, 1)

    return output
```


```python
from demosaic_and_mosaic import mosaic, demosaic
pattern='RGGB'
img_mosaic = mosaic(img_inverse_ccm, pattern=pattern)
```


```python
plt.figure(figsize=(18, 6))
plt.subplot(1, 5, 1)
plt.axis('off')
plt.imshow(img)
plt.title('Oringal Image')
plt.subplot(1, 5, 2)
plt.axis('off')
plt.imshow(img_inverse_tone)
plt.title('Image after inverse tone')
plt.subplot(1, 5, 3)
plt.axis('off')
plt.imshow(img_inverse_tone_XYZ)
plt.title('Image after CIE')
plt.subplot(1, 5, 4)
plt.axis('off')
plt.imshow(img_inverse_ccm)
plt.title('Image after CCM')
plt.subplot(1, 5, 5)
plt.axis('off')
plt.imshow(img_mosaic)
plt.title('Image after mosaic')
```

    Clipping input data to the valid range for imshow with RGB data ([0..1] for floats or [0..255] for integers).
    




    Text(0.5, 1.0, 'Image after mosaic')




![png](https://drive.google.com/uc?export=view&id=1jwlecE7j6MovcXKy8TCl2maLyoDPUC1F)


## 3.5 Inverse AWB
1. Create a numpy array with shape of input RAW image.
2. According to the given Bayer pattern, fill the fr into  correspinding red channel position and fb into correspinding  blue channel position. Fill 1 into green channel position otherwise.  


```python
def generate_wb_mask(img, pattern, fr, fb):
    '''
    Input:
        img: H*W numpy array, RAW image
        pattern: string, 4 different Bayer patterns (GRBG, RGGB, GBRG, BGGR)
        fr: float, white balance factor of red channel
        fb: float, white balance factor of blue channel 
    Output:
        mask: H*W numpy array, white balance mask
    '''
    ########################################################################
    # TODO:                                                                #
    #   1. Create a numpy array with shape of input RAW image.             #
    #   2. According to the given Bayer pattern, fill the fr into          #
    #      correspinding red channel position and fb into correspinding    #
    #      blue channel position. Fill 1 into green channel position       #
    #      otherwise.                                                      #
    ########################################################################


    
    ########################################################################
    #                                                                      #
    #                           End of your code                           #
    #                                                                      # 
    ########################################################################
        
    return mask
```


```python
fr = 0.7715567349551743
fb = 0.9068480239589546
wb_mask = generate_wb_mask(img_mosaic, pattern, fr, fb)
img_Inverse_WB = img_mosaic  * wb_mask
```


```python
plt.figure(figsize=(18, 6))
plt.subplot(1, 6, 1)
plt.axis('off')
plt.imshow(img)
plt.title('Oringal Image')
plt.subplot(1, 6, 2)
plt.axis('off')
plt.imshow(img_inverse_tone)
plt.title('Image after inverse tone')
plt.subplot(1, 6, 3)
plt.axis('off')
plt.imshow(img_inverse_tone_XYZ)
plt.title('Image after CIE')
plt.subplot(1, 6, 4)
plt.axis('off')
plt.imshow(img_inverse_ccm)
plt.title('Image after CCM')
plt.subplot(1, 6, 5)
plt.axis('off')
plt.imshow(img_mosaic)
plt.title('Image after mosaic')
plt.subplot(1, 6, 6)
plt.axis('off')
plt.imshow(img_Inverse_WB)
plt.title('Image after Inverse WB')
```

    Clipping input data to the valid range for imshow with RGB data ([0..1] for floats or [0..255] for integers).
    




    Text(0.5, 1.0, 'Image after Inverse WB')




![png](https://drive.google.com/uc?export=view&id=1pfRnQv1lpY5A4-jYNIgQ5WTf95GbOiHN)


# 4. ISP Process

## 4.1 AWB


```python
wb_mask = generate_wb_mask(img_Inverse_WB, pattern, 1/fr, 1/fb)
img_WB = img_Inverse_WB * wb_mask
img_WB = np.clip(img_WB, 0, 1)
```


```python
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1);plt.axis('off')
plt.imshow(img_Inverse_WB)
plt.title('image Raw')
plt.subplot(1, 2, 2);plt.axis('off')
plt.imshow(img_WB)
plt.title('Image after WG')
```




    Text(0.5, 1.0, 'Image after WG')




![png](https://drive.google.com/uc?export=view&id=1Wi6vUdQfUX5iGen6ZcnYIvUsvYtkgg_u)


## 4.2 Demosaic


```python
img_demosaic = demosaic(img_WB, pattern=pattern) #pattern='RGGB'
```


```python
plt.figure(figsize=(12, 6))
plt.subplot(1, 3, 1);plt.axis('off')
plt.imshow(img_Inverse_WB)
plt.title('image Raw')
plt.subplot(1, 3, 2);plt.axis('off')
plt.imshow(img_WB)
plt.title('Image after WG')
plt.subplot(1, 3, 3);plt.axis('off')
plt.imshow(img_demosaic)
plt.title('Image after demosaic')
```




    Text(0.5, 1.0, 'Image after demosaic')




![png](https://drive.google.com/uc?export=view&id=1I_nZD8AoFqotBM-PPeMpMQmADuTP00dU)


## 4.3 Color Correction


```python
img_color_correction = color_correction(img_demosaic, ccm_inv)
```


```python
plt.figure(figsize=(14, 6))
plt.subplot(1, 4, 1);plt.axis('off')
plt.imshow(img_Inverse_WB)
plt.title('image Raw')
plt.subplot(1, 4, 2);plt.axis('off')
plt.imshow(img_WB)
plt.title('Image after WG')
plt.subplot(1, 4, 3);plt.axis('off')
plt.imshow(img_demosaic)
plt.title('Image after demosaic')
plt.subplot(1, 4, 4);plt.axis('off')
plt.imshow(img_color_correction)
plt.title('Image after Color Correction')
```




    Text(0.5, 1.0, 'Image after Color Correction')




![png](https://drive.google.com/uc?export=view&id=19dST7jt4rIFHDCcassdl4B7kO9pJBxVs)


## 4.4 XYZ2RGB


```python
img_RGB = XYZ2RGB(img_color_correction)
```


```python
plt.figure(figsize=(18, 6))
plt.subplot(1, 5, 1);plt.axis('off')
plt.imshow(img_Inverse_WB)
plt.title('image Raw')
plt.subplot(1, 5, 2);plt.axis('off')
plt.imshow(img_WB)
plt.title('Image after WG')
plt.subplot(1, 5, 3);plt.axis('off')
plt.imshow(img_demosaic)
plt.title('Image after demosaic')
plt.subplot(1, 5, 4);plt.axis('off')
plt.imshow(img_color_correction)
plt.title('Image after Color Correction')
plt.subplot(1, 5, 5);plt.axis('off')
plt.imshow(img_RGB)
plt.title('Image after XYZ2RGB')
```




    Text(0.5, 1.0, 'Image after XYZ2RGB')




![png](https://drive.google.com/uc?export=view&id=1RNLyihGW4fG2hsXycRkTFQr41rkKUuXw)


## 4.5 tone_mapping


```python
img_tm = tone_mapping(img_RGB, I, B, index=tone_index, inv=False)
```


```python
plt.figure(figsize=(18, 6))
plt.subplot(1, 6, 1);plt.axis('off')
plt.imshow(img_Inverse_WB)
plt.title('image Raw')
plt.subplot(1, 6, 2);plt.axis('off')
plt.imshow(img_WB)
plt.title('Image after WG')
plt.subplot(1, 6, 3);plt.axis('off')
plt.imshow(img_demosaic)
plt.title('Image after demosaic')
plt.subplot(1, 6, 4);plt.axis('off')
plt.imshow(img_color_correction)
plt.title('Image after Color Correction')
plt.subplot(1, 6, 5);plt.axis('off')
plt.imshow(img_RGB)
plt.title('Image after XYZ2RGB')
plt.subplot(1, 6, 6);plt.axis('off')
plt.imshow(img_tm)
plt.title('Image after tone mapping')
```




    Text(0.5, 1.0, 'Image after tone mapping')




![png](https://drive.google.com/uc?export=view&id=1jSC1R7zFmoXtoxH1btjVjtynuHwmhAeR)



```python
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1);plt.axis('off')
plt.imshow(img)
plt.title('Oringal Image')
plt.subplot(1, 2, 2);plt.axis('off')
plt.imshow(img_tm)
plt.title('Image after inverse process')
```




    Text(0.5, 1.0, 'Image after inverse process')




![png](https://drive.google.com/uc?export=view&id=1IMxtOIRf-sh59Q3o3t8Te-JT9mpzcEFD)


## Peak Signal-to-Noise Ratio


```python
def calculate_psnr(img1, img2):
    '''
    Input:
        img1, img2: H*W*3 numpy array
    Output:
        psnr: the peak signal-to-noise ratio value
    '''
    ########################################################################
    # TODO:                                                                #
    #   Following above instruction to get PSNR as output.                 #
    #                                                                      #
    ########################################################################


    
    ########################################################################
    #                                                                      #
    #                           End of your code                           #
    #                                                                      # 
    ########################################################################
    return psnr
```


```python
calculate_psnr(img, img_tm)
```




    81.43537077717903


