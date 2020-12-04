#import needed libraries

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from scipy.stats import pearsonr

import numpy as np
import math

import cv2

import sys

class Ralph(Node):
    #class constructor
    def __init__(self):
        #create node called ralph_node
        super().__init__("ralph_node")
        #create subsriber to /camera topic
        self.image_sub = self.create_subscription(Image,'/camera',self.listerner_callback,10)
        #create publisher to /robot_vel topic
        self.twist_pub = self.create_publisher(Twist,'/robot_vel',10)
        #create publisher to /output_Image topic
        self.image_pub = self.create_publisher(Image,'/output_Image',10)
        #instantiate class varibles
        self.image_sub
        self.first = True
        self.sums = None

    #gives a score to an image based on the 5 largestest differences between sums of pixel values in columns along the image
    def score(self,img):
        sums = np.zeros(len(img[0]))
        five_best = np.zeros(5)
        for i in range(1,len(img[0])):
            #sum of all the pixes in one column
            sums[i]=np.sum(img[0:len(img),i])
            #difference between ajacent columns
            value = abs(sums[i]-sums[i-1])
            #check if the sum is one of the largest 5 sums
            for j in range(len(five_best)):
                if(value>five_best[j]):
                    for k in range(j+1,len(five_best)):
                        five_best[k]=five_best[k-1]
                    five_best[j] = value
        #return the sum of the best 5 column sums
        return np.sum(five_best)

    #returns the sum of the pixel in each column
    def scanLine(self, img):
        sums = np.zeros(len(img[0]))
        for i in range(len(img[0])):
            sums[i]=np.sum(img[0:len(img),i])
        return sums

    #calculates the optimal offset between the reference scan line and the current scan lin
    def offset_calc(self, current):
        best = -1
        index = 0
        smaller = min(len(current),len(self.reference_scanline))
        #for loop translates the current scan along the reference scan line starting on the right
        for i in range(2,len(current)+len(self.reference_scanline)-2):
            #finds the intercepting part of the each scanline and sets it to data1 and data2
            if(i < len(self.reference_scanline)-1 and i < smaller-1):
                data1 = self.reference_scanline[len(self.reference_scanline)-i:]
            elif(i < len(self.reference_scanline)-1 and i >= smaller-1):
                data1 = self.reference_scanline[len(self.reference_scanline)-i:len(self.reference_scanline)-(i-smaller)]
            elif(i > len(self.reference_scanline) and len(current)-i-1>=len(self.reference_scanline)):
                data1 = self.reference_scanline
            else:
                if(i < len(current)+len(self.reference_scanline)-len(self.reference_scanline)):
                    data1 = self.reference_scanline[0:i]
                else:
                    data1 = self.reference_scanline[:len(current)+len(self.reference_scanline)-i]
            if(i < len(current)-1 and i < smaller):
                data2 = current[:i]
            elif(i < len(current) and i >= smaller-1):
                data2 = current[max(i-smaller,0):i]
            else:
                if(len(current) < smaller):
                    data2 = current[i-smaller:]
                else:
                    data2 = current[i-smaller:]
            #calculates the correlation coefficient between data1 and data2
            cor_coef,_ = pearsonr(data1,data2)
            #check if the coeficient is the largest so far
            if(cor_coef > best):
                best = cor_coef
                index = i
        #return the offset
        return (len(current)+len(self.reference_scanline))/2-index

    #called when the subscriber receives a new image
    def listerner_callback(self,msg):
        #instiate needed varibles and constants
        cv_bridge = CvBridge()
        img = cv_bridge.imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        angle = 0.365
        max_height = 350
        min_height = 200
        #cut off top and bottom of image
        img = img[len(img)-max_height:len(img)-min_height]
        #change the resolution of the image for easier computing
        img = cv2.resize(img,(100,int((max_height-min_height)/2)),interpolation=cv2.INTER_AREA)
        #determine the width of the image
        img_width = len(img[0])
        #create a second image to copy the first image into
        img2 = np.zeros((len(img),len(img[0])))
        #cuts off portions of the image so teh resutling image is of the projection of a rectangle on the ground projected onto the camera
        for i in range(len(img)):
            #determine left and right cutoff for that row
            left_cutoff = int(i*angle)
            right_cutoff = len(img[i])-(left_cutoff)
            temp = np.zeros((1,right_cutoff-left_cutoff))
            #cutoff the sides and set hte img row to the resulting row with cutoffs
            temp[0] = img[len(img)-1-i][left_cutoff:right_cutoff]
            temp = cv2.resize(temp, (img_width,1), interpolation=cv2.INTER_AREA)
            img[len(img)-1-i] = temp[0]
        #if first image, set the scanline as the reference scanline
        if(self.first):
            self.reference_scanline = self.scanLine(img)
            self.first = False
        #set some constants
        min_radius = 200
        searches = 50
        curr = -1/min_radius
        max_added_width = int(math.ceil(math.sqrt(min_radius**2-((len(img)-1)**2))))
        max_width = img_width+2*max_added_width
        values = np.zeros(searches)
        #preforms the shifts on the image based on guessed turn radius 
        for i in range(searches):
            temp_image = np.zeros((len(img),max_width))
            if(curr == 0):
                shift = 0
                values[i] = self.score(img)
                continue
            rad = 1/curr
            #prefroms shift
            for j in range(len(img)):
                shift = int(abs(rad)-math.sqrt(rad**2-j**2))
                if(rad < 0):
                    shift*=-1
                temp_image[len(img)-1-j][max_added_width+shift:max_added_width+shift+img_width] = img[len(img)-1-j]
            values[i] = self.score(temp_image)
            curr+=(1/min_radius)/(searches/2)
        turn_radius = 0
        best = 0
        #finds best fitting raidus best on values array
        for i in range(searches):
            if(values[i] > best):
                best = values[i]
                turn_radius = (-1/min_radius)+((1/min_radius)/(searches/2))*i
        #recreates the best shifted image for debugging purposes
        shifted_best = np.zeros((len(img),max_width))
        if(turn_radius==0):
            shifted_best = img
        else:
            rad = 1/turn_radius
            for j in range(len(img)):
                shift = int(abs(rad)-math.sqrt(rad**2-j**2))
                if(rad < 0):
                    shift*=-1
                shifted_best[len(img)-1-j][max_added_width+shift:max_added_width+shift+img_width] = img[len(img)-1-j]
        #calculates the offset
        offset = self.offset_calc(self.scanLine(shifted_best))
        #print the calculated offset and turn radius
        self.get_logger().info('offset: "%f"' % offset)
        self.get_logger().info('turn_raidus: "%f"' % turn_radius)
        #calculates the robot's speed and rotation
        twist_msg = Twist()
        twist_msg.angular = Vector3()
        x_speed = 0.5
        offset_r = offset*.03
        turn_radius_r = turn_radius/0.03
        #applies maximium value to the rotation caused by offset
        if(offset_r>0):
            offset_r=min(offset_r,0.3)
        else:
            offset_r=max(offset_r,-0.3)
        #applies maximium value to the rotation cuase by turn_radius
        if(turn_radius_r>0):
            turn_radius_r=min(turn_radius_r,0.8)
        else:
            turn_radius_r=max(turn_radius_r,-0.8)
        #slows down the robot and lowers offset rotation influence if the turn is too tight 
        if(abs(turn_radius) > 0.004):
            x_speed -= abs(turn_radius-0.004)*5
            if(offset_r*turn_radius_r < 0):
                if(offset_r>0):
                    offset_r=min(offset_r,0.15)
                else:
                    offset_r=max(offset_r,-0.15)
        #set the twist object to the robots speed and rotation
        twist_msg.angular = Vector3()
        twist_msg.angular.z = turn_radius_r+offset_r
        self.get_logger().info('turning: "%f"' % (turn_radius_r+offset_r))
        self.get_logger().info('turning: "%f"' % twist_msg.angular.z)
        twist_msg.linear = Vector3()
        twist_msg.linear.x = x_speed
        #publish the twist object with teh robots speed and rotation
        self.twist_pub.publish(twist_msg)

#rest of code is copied from https://github.com/ros-perception/vision_opencv/blob/noetic/cv_bridge/python/cv_bridge/core.py
#to convert ros images to opencv images

class CvBridgeError(TypeError):
    """This is the error raised by :class:`cv_bridge.CvBridge` methods when they fail."""

    pass


class CvBridge(object):
    """
    The CvBridge is an object that converts between OpenCV Images and ROS Image messages.
       .. doctest::
           :options: -ELLIPSIS, +NORMALIZE_WHITESPACE
           >>> import cv2
           >>> import numpy as np
           >>> from cv_bridge import CvBridge
           >>> br = CvBridge()
           >>> dtype, n_channels = br.encoding_as_cvtype2('8UC3')
           >>> im = np.ndarray(shape=(480, 640, n_channels), dtype=dtype)
           >>> msg = br.cv2_to_imgmsg(im)  # Convert the image to a message
           >>> im2 = br.imgmsg_to_cv2(msg) # Convert the message to a new image
           >>> # Convert the image to a compress message
           >>> cmprsmsg = br.cv2_to_compressed_imgmsg(im)
           >>> # Convert the compress message to a new image
           >>> im22 = br.compressed_imgmsg_to_cv2(msg)
           >>> cv2.imwrite("this_was_a_message_briefly.png", im2)
    """

    def __init__(self):
        self.cvtype_to_name = {}
        self.cvdepth_to_numpy_depth = {cv2.CV_8U: 'uint8', cv2.CV_8S: 'int8',
                                       cv2.CV_16U: 'uint16', cv2.CV_16S: 'int16',
                                       cv2.CV_32S: 'int32', cv2.CV_32F: 'float32',
                                       cv2.CV_64F: 'float64'}

        for t in ['8U', '8S', '16U', '16S', '32S', '32F', '64F']:
            for c in [1, 2, 3, 4]:
                nm = '%sC%d' % (t, c)
                self.cvtype_to_name[getattr(cv2, 'CV_%s' % nm)] = nm

        self.numpy_type_to_cvtype = {'uint8': '8U', 'int8': '8S', 'uint16': '16U',
                                     'int16': '16S', 'int32': '32S', 'float32': '32F',
                                     'float64': '64F'}
        self.numpy_type_to_cvtype.update(dict((v, k) for (k, v) in self.numpy_type_to_cvtype.items()))

    def dtype_with_channels_to_cvtype2(self, dtype, n_channels):
        return '%sC%d' % (self.numpy_type_to_cvtype[dtype.name], n_channels)

    def cvtype2_to_dtype_with_channels(self, cvtype):
        from cv_bridge.boost.cv_bridge_boost import CV_MAT_CNWrap, CV_MAT_DEPTHWrap
        return self.cvdepth_to_numpy_depth[CV_MAT_DEPTHWrap(cvtype)], CV_MAT_CNWrap(cvtype)

    def encoding_to_cvtype2(self, encoding):
        from cv_bridge.boost.cv_bridge_boost import getCvType

        try:
            return getCvType(encoding)
        except RuntimeError as e:
            raise CvBridgeError(e)

    def encoding_to_dtype_with_channels(self, encoding):
        return self.cvtype2_to_dtype_with_channels(self.encoding_to_cvtype2(encoding))

    def compressed_imgmsg_to_cv2(self, cmprs_img_msg, desired_encoding='passthrough'):
        """
        Convert a sensor_msgs::CompressedImage message to an OpenCV :cpp:type:`cv::Mat`.
        :param cmprs_img_msg:   A :cpp:type:`sensor_msgs::CompressedImage` message
        :param desired_encoding:  The encoding of the image data, one of the following strings:
           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h
        :rtype: :cpp:type:`cv::Mat`
        :raises CvBridgeError: when conversion is not possible.
        If desired_encoding is ``"passthrough"``, then the returned image has the same format
        as img_msg. Otherwise desired_encoding must be one of the standard image encodings
        This function returns an OpenCV :cpp:type:`cv::Mat` message on success,
        or raises :exc:`cv_bridge.CvBridgeError` on failure.
        If the image only has one channel, the shape has size 2 (width and height)
        """
        import cv2
        import numpy as np

        str_msg = cmprs_img_msg.data
        buf = np.ndarray(shape=(1, len(str_msg)),
                         dtype=np.uint8, buffer=cmprs_img_msg.data)
        im = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)

        if desired_encoding == 'passthrough':
            return im

        from cv_bridge.boost.cv_bridge_boost import cvtColor2

        try:
            res = cvtColor2(im, 'bgr8', desired_encoding)
        except RuntimeError as e:
            raise CvBridgeError(e)

        return res

    def imgmsg_to_cv2(self, img_msg, desired_encoding='passthrough'):
        """
        Convert a sensor_msgs::Image message to an OpenCV :cpp:type:`cv::Mat`.
        :param img_msg:   A :cpp:type:`sensor_msgs::Image` message
        :param desired_encoding:  The encoding of the image data, one of the following strings:
           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h
        :rtype: :cpp:type:`cv::Mat`
        :raises CvBridgeError: when conversion is not possible.
        If desired_encoding is ``"passthrough"``, then the returned image has the same format
        as img_msg. Otherwise desired_encoding must be one of the standard image encodings
        This function returns an OpenCV :cpp:type:`cv::Mat` message on success,
        or raises :exc:`cv_bridge.CvBridgeError` on failure.
        If the image only has one channel, the shape has size 2 (width and height)
        """
        import numpy as np
        dtype, n_channels = self.encoding_to_dtype_with_channels(img_msg.encoding)
        dtype = np.dtype(dtype)
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')

        img_buf = np.asarray(img_msg.data, dtype=dtype) if isinstance(img_msg.data, list) else img_msg.data

        if n_channels == 1:
            im = np.ndarray(shape=(img_msg.height, img_msg.width),
                            dtype=dtype, buffer=img_buf)
        else:
            im = np.ndarray(shape=(img_msg.height, img_msg.width, n_channels),
                            dtype=dtype, buffer=img_buf)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            im = im.byteswap().newbyteorder()

        if desired_encoding == 'passthrough':
            return im

        from cv_bridge.boost.cv_bridge_boost import cvtColor2

        try:
            res = cvtColor2(im, img_msg.encoding, desired_encoding)
        except RuntimeError as e:
            raise CvBridgeError(e)

        return res

    def cv2_to_compressed_imgmsg(self, cvim, dst_format='jpg'):
        """
        Convert an OpenCV :cpp:type:`cv::Mat` type to a ROS sensor_msgs::CompressedImage message.
        :param cvim:      An OpenCV :cpp:type:`cv::Mat`
        :param dst_format:  The format of the image data, one of the following strings:
        http://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html
        http://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat
        * imread(const string& filename, int flags)
           * bmp, dib
           * jpeg, jpg, jpe
           * jp2
           * png
           * pbm, pgm, ppm
           * sr, ras
           * tiff, tif
        :rtype:           A sensor_msgs.msg.CompressedImage message
        :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``format``
        This function returns a sensor_msgs::Image message on success,
        or raises :exc:`cv_bridge.CvBridgeError` on failure.
        """
        import cv2
        import numpy as np
        if not isinstance(cvim, (np.ndarray, np.generic)):
            raise TypeError('Your input type is not a numpy array')
        cmprs_img_msg = sensor_msgs.msg.CompressedImage()
        cmprs_img_msg.format = dst_format
        ext_format = '.' + dst_format
        try:
            cmprs_img_msg.data.frombytes(np.array(cv2.imencode(ext_format, cvim)[1]).tobytes())
        except RuntimeError as e:
            raise CvBridgeError(e)

        return cmprs_img_msg

    def cv2_to_imgmsg(self, cvim, encoding='passthrough'):
        """
        Convert an OpenCV :cpp:type:`cv::Mat` type to a ROS sensor_msgs::Image message.
        :param cvim:      An OpenCV :cpp:type:`cv::Mat`
        :param encoding:  The encoding of the image data, one of the following strings:
           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h
        :rtype:           A sensor_msgs.msg.Image message
        :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``encoding``
        If encoding is ``"passthrough"``, then the message has the same encoding as the image's
        OpenCV type. Otherwise desired_encoding must be one of the standard image encodings
        This function returns a sensor_msgs::Image message on success,
        or raises :exc:`cv_bridge.CvBridgeError` on failure.
        """
        import numpy as np
        if not isinstance(cvim, (np.ndarray, np.generic)):
            raise TypeError('Your input type is not a numpy array')
        img_msg = Image()
        img_msg.height = cvim.shape[0]
        img_msg.width = cvim.shape[1]
        if len(cvim.shape) < 3:
            cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, 1)
        else:
            cv_type = self.dtype_with_channels_to_cvtype2(cvim.dtype, cvim.shape[2])
        if encoding == 'passthrough':
            img_msg.encoding = cv_type
        else:
            img_msg.encoding = encoding
            # Verify that the supplied encoding is compatible with the type of the OpenCV image
            if self.cvtype_to_name[self.encoding_to_cvtype2(encoding)] != cv_type:
                raise CvBridgeError('encoding specified as %s, but image has incompatible type %s'
                                    % (encoding, cv_type))
        if cvim.dtype.byteorder == '>':
            img_msg.is_bigendian = True
        img_msg.data.frombytes(cvim.tobytes())
        img_msg.step = len(img_msg.data) // img_msg.height

        return img_msg
    
def main(args=None):
    rclpy.init(args=args)
    ralph_node = Ralph()
    rclpy.spin(ralph_node)
    ralph_node.destroy_node()
    rclpy.shutdown()