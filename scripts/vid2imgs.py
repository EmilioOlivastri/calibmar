import cv2
import argparse
import os
from tqdm import tqdm
import time

def sharpness(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fm = cv2.Laplacian(gray, cv2.CV_64F).var()
    return fm

def main():

    print('Converting video to images...')
    parser = argparse.ArgumentParser(description='Convert video to images at desired frame rate')
    parser.add_argument('--video', type=str, help='Path to video file')
    parser.add_argument('--output', type=str, help='Path to output directory')
    parser.add_argument('--fps', type=int, default=30, help='Frame rate of the video')
    parser.add_argument('--skip', type=int, default=0, help='Number of frames to skip')
    parser.add_argument('--max_frames', type=int, default=2000, help='Maximum number of frames to extract')
    parser.add_argument('--vis', type=bool, default=False, help='Visualize the video')
    parser.add_argument('--factor', type=float, default=1.0, help='Resize factor for images')
    parser.add_argument('--format', type=str, default='png', help='Image format')


    args = parser.parse_args()
    path_video = args.video
    path_output = args.output
    skip = args.skip
    factor = args.factor
    format = args.format
    fps = args.fps
    max_frames = args.max_frames
    vis = args.vis


    time_ns = time.time_ns()
    delta = int(1.0/fps * 1e9)

    if format not in ['png', 'jpg', 'jpeg', 'bmp', 'tiff']:
        print('Invalid image format!!!')
        return

    cap = cv2.VideoCapture(path_video)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    counter = 0
    for i in tqdm(range(frame_count), desc ="Extracting"):
        ret, img = cap.read()
        time_ns += delta

        if ret and (skip == 0 or i % skip == 0):
            image_id = str(counter).zfill(5)
            image_timestamp = time_ns
            width = int(img.shape[1] *  factor)
            height = int(img.shape[0] * factor)
            res_img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
            # Check sharpness
            fm = sharpness(res_img)
            if fm < 30:
                continue
            
            if vis:
                cv2.imshow('Frame', res_img)
                cv2.waitKey(30)
            
            # Change the image to grayscale
            gray = cv2.cvtColor(res_img, cv2.COLOR_BGR2GRAY)

            cv2.imwrite(os.path.join(path_output,f'{image_timestamp}.{format}'), gray)
            #cv2.imwrite(os.path.join(path_output,f'{image_id}.{format}'), res_img)
            counter += 1

        if counter >= max_frames:
            break
    
    cap.release()
    if vis: cv2.destroyAllWindows()

    return



if __name__ == '__main__':
    main()