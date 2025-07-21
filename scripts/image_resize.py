import cv2
import argparse
import os
from tqdm import tqdm

def list_images(folder_path, extensions=['.png', '.jpg', '.jpeg', '.bmp', '.tiff']):
    images = [f for f in os.listdir(folder_path) if os.path.splitext(f)[1].lower() in extensions]
    images.sort()
    return images

def main():

    print('Converting video to images...')
    parser = argparse.ArgumentParser(description='Convert video to images at desired frame rate')
    parser.add_argument('--input_path', type=str, help='Path to output directory')
    parser.add_argument('--output_path', type=str, help='Path to output directory')
    parser.add_argument('--resize_factor', type=float, default=0.5, help='Resize factor for images')
    
    args = parser.parse_args()
    input_path = args.input_path
    output_path = args.output_path
    resize_factor = float(args.resize_factor)

    print(f"Input path: {input_path}")
    print(f"Output path: {output_path}")
    print(f"Resize factor: {resize_factor}")

    image_list = list_images(input_path)
    
    num_images = len(image_list)
    if num_images == 0:
        print("No images found in the directory.")
        return
    print(f"Found {num_images} images in the directory.")

    for i in tqdm(range(num_images), desc ="Extracting"):
        img = cv2.imread(os.path.join(input_path, image_list[i]))
        w = int(img.shape[1] * resize_factor)
        h = int(img.shape[0] * resize_factor)
        res_img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        cv2.imwrite(os.path.join(output_path, image_list[i]), res_img)
        
    return

if __name__ == '__main__':
    main()