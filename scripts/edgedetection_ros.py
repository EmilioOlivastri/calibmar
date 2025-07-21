import cv2
import argparse
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CropLayer(object):
    def __init__(self, params, blobs):
        self.xstart = 0
        self.xend = 0
        self.ystart = 0
        self.yend = 0

    def getMemoryShapes(self, inputs):
        inputShape, targetShape = inputs[0], inputs[1]
        batchSize, numChannels = inputShape[0], inputShape[1]
        height, width = targetShape[2], targetShape[3]

        self.ystart = (inputShape[2] - targetShape[2]) // 2
        self.xstart = (inputShape[3] - targetShape[3]) // 2
        self.yend = self.ystart + height
        self.xend = self.xstart + width

        return [[batchSize, numChannels, height, width]]

    def forward(self, inputs):
        return [inputs[0][:, :, self.ystart:self.yend, self.xstart:self.xend]]


class EdgeDetection(object):
    def __init__(self, topic='/cam0/image_raw', factor=1.0, vis_edge=1, vis_deep=0):
        self.sub = rospy.Subscriber(topic, Image, self.extractCorners)
        self.net = cv2.dnn.readNetFromCaffe('scripts/weights/deploy.prototxt', 'scripts/weights/hed_pretrained_bsds.caffemodel')
        self.cuda_enabled = vis_deep == 1
        self.resize_factor = factor
        self.vis_edge = vis_edge
        self.vis_deep = vis_deep

    def extractCorners(self, data):
        img = CvBridge().imgmsg_to_cv2(data)
        width  =  int(img.shape[1] *  self.resize_factor)
        height = int(img.shape[0] * self.resize_factor)
        img_res = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
        
        if self.cuda_enabled == True:
            color_img = img_res.copy()
            if len(color_img.shape) < 3:
                color_img = cv2.cvtColor(color_img, cv2.COLOR_GRAY2BGR)

            input_blob = cv2.dnn.blobFromImage(color_img, scalefactor=1.0, size=(width, height), mean=(104.00698793, 116.66876762, 122.67891434), swapRB=False, crop=False)
            self.net.setInput(input_blob)
            output = self.net.forward()
            output = output[0, 0]
            output = cv2.resize(output, (color_img.shape[1], color_img.shape[0]))
            output = (255 * output).astype('uint8')
            #ret, output = cv2.threshold(output, 50, 255, cv2.THRESH_BINARY)
            
        # Change the image to grayscale
        img_gray = img_res.copy()
        if len(img_gray.shape) == 3:
            img_gray = cv2.cvtColor(img_gray, cv2.COLOR_BGR2GRAY)

        # Blur the image for better edge detection
        img_blur = cv2.GaussianBlur(img_gray, (3,3), 0)
        edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200)

        if self.vis_edge == 1:
            cv2.imshow('Edge Detection', edges)
            if self.cuda_enabled == True:
                cv2.imshow('Edge Detection Deep', output)
            
        cv2.imshow('Original', img_res)    
        cv2.waitKey(30)



def main():


    print('Edge Detection Listener Activated...')
    parser = argparse.ArgumentParser(description='Convert video to images at desired frame rate')
    parser.add_argument('--topic',    type=str,    default='/cam0/image_raw',  help='ROS topic for image')
    parser.add_argument('--factor',   type=float,  default=1.0, help='Resize factor for images')
    parser.add_argument('--vis_edge', type=int,    default=1, help='Visualize the edge detection')
    parser.add_argument('--vis_deep', type=int,    default=0, help='Visualize the deep edge detection')

    args = parser.parse_args()
    image_topic = args.topic
    factor = args.factor
    vis_deep = args.vis_deep
    vis_edge = args.vis_edge

    cv2.dnn_registerLayer('Crop', CropLayer)

    rospy.init_node('extractEdges', anonymous=True)
    print('Node Idle...')
    edge_detection = EdgeDetection(topic=image_topic, factor=factor, vis_edge=vis_edge, vis_deep=vis_deep)
    rospy.spin()

    return



if __name__ == '__main__':
    main()
