import cv2
import argparse
import os
from tqdm import tqdm
import time


def list_images(folder_path, extensions=['.png', '.jpg', '.jpeg', '.bmp', '.tiff']):
    images = [f for f in os.listdir(folder_path) if os.path.splitext(f)[1].lower() in extensions]
    images.sort()
    return images

def main():

    print('Converting video to images...')
    parser = argparse.ArgumentParser(description='Convert video to images at desired frame rate')
    parser.add_argument('--input_path', type=str, help='Path to output directory')
    

    args = parser.parse_args()
    input_path = args.input_path

    image_list = list_images(input_path)

    keys = {}
    keys['delete'] = 100 
    num_images = len(image_list)

    for i in tqdm(range(num_images), desc ="Extracting"):
        img = cv2.imread(os.path.join(input_path, image_list[i]))
        w = int(img.shape[1] / 2)
        h = int(img.shape[0]/2)
        res_img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)

        cv2.imshow('Frame', res_img)
        key = cv2.waitKey(0)
        
        if key == keys['delete']:
            os.remove(os.path.join(input_path, image_list[i]))
            print(f"Deleted {image_list[i]}")
        

                
                

    cv2.destroyAllWindows()

    return



if __name__ == '__main__':
    main()