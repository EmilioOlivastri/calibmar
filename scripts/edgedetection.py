import cv2
import argparse
from tqdm import tqdm


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



def main():

    print('Converting video to images...')
    parser = argparse.ArgumentParser(description='Convert video to images at desired frame rate')
    parser.add_argument('--video', type=str, help='Path to video file')
    parser.add_argument('--factor', type=float, default=1.0, help='Resize factor for images')
    parser.add_argument('--vis_edge', type=int, default=1, help='Visualize the edge detection')
    parser.add_argument('--vis_deep', type=int, default=0, help='Visualize the deep edge detection')

    args = parser.parse_args()
    path_video = args.video
    factor = args.factor
    vis_deep = args.vis_deep

    cv2.dnn_registerLayer('Crop', CropLayer)
    net = cv2.dnn.readNetFromCaffe('scripts/weights/deploy.prototxt', 'scripts/weights/hed_pretrained_bsds.caffemodel')
    cuda_enabled = False
    if vis_deep == 1:
        cuda_enabled = True
    print('CUDA enabled:', cv2.cuda.getCudaEnabledDeviceCount() > 0)

    cap = cv2.VideoCapture(path_video)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    for i in tqdm(range(frame_count), desc ="Extracting"):
        ret, img = cap.read()

        if ret :
          
            # Resize the image
            width = int(img.shape[1] *  factor)
            height = int(img.shape[0] * factor)
            img2 = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
            
            if cuda_enabled == True:
                input_blob = cv2.dnn.blobFromImage(img2, scalefactor=factor, size=(width, height), mean=(104.00698793, 116.66876762, 122.67891434), swapRB=False, crop=False)
                net.setInput(input_blob)
                output = net.forward()
                output = output[0, 0]
                output = cv2.resize(output, (img2.shape[1], img2.shape[0]))
                output = (255 * output).astype('uint8')
                #ret, output = cv2.threshold(output, 50, 255, cv2.THRESH_BINARY)
            
            # Change the image to grayscale
            img_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

            # Blur the image for better edge detection
            img_blur = cv2.GaussianBlur(img_gray, (3,3), 0)
            edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200)

            if args.vis_edge == 1:
                cv2.imshow('Edge Detection', edges)
                if cuda_enabled == True:
                    cv2.imshow('Edge Detection Deep', output)
            else:
                cv2.imshow('Original', img2)
            cv2.waitKey(30)
            
    
    cap.release()
    cv2.destroyAllWindows()

    return



if __name__ == '__main__':
    main()