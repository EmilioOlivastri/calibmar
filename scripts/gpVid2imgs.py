import cv2
import argparse
import os
from tqdm import tqdm
import datetime as dt
import time
import ffmpeg

def main():

    print('GoPro Image Syncrhonizer...')
    parser = argparse.ArgumentParser(description='Convert video to images at desired frame rate')
    parser.add_argument('--video', type=str, help='Path to video')
    parser.add_argument('--output', type=str, help='Path to output directory')
    parser.add_argument('--fps', type=int, default=30, help='Frame rate of the video')
    parser.add_argument('--vis', type=bool, default=False, help='Visualize the video')
    parser.add_argument('--factor', type=float, default=1.0, help='Resize factor for images')
    parser.add_argument('--format', type=str, default='png', help='Image format')

    args = parser.parse_args()
    path_video = args.video
    path_output = args.output
    factor = args.factor
    format = args.format
    fps = args.fps
    vis = args.vis

    print(f'Video: {path_video.split("/")[-1]}')
    metadata = ffmpeg.probe(path_video)
    videodate = metadata['streams'][0]['tags']['creation_time']
    timecode = metadata['streams'][0]['tags']['timecode']
    duration = metadata['format']['duration']

    expected_num_frames = int(float(duration) * fps)

    day, hours = videodate.split('T')
    hours = hours.split('.')[0]
    millisec = timecode.split(':')[-1]
    delta = int(1.0/fps * 1e9)

    date_string = day + ' ' + hours + '.' + millisec
    format_string = "%Y-%m-%d %H:%M:%S.%f"
    dt_object = dt.datetime.strptime(date_string, format_string)
    st_timestamp = int(dt_object.timestamp() * 1e9)
    print(f'Datestring: {date_string} | Format: {format_string}')
    print(f'Start timestamp: {st_timestamp}')

    if format not in ['png', 'jpg', 'jpeg', 'bmp', 'tiff']:
        print('Invalid image format!!!')
        return

    cap = cv2.VideoCapture(path_video)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    counter = 0
    for i in tqdm(range(frame_count), desc ="Extracting"):
        ret, img = cap.read()
        st_timestamp = st_timestamp + i * delta

        if ret:
            image_timestamp = st_timestamp
            width = int(img.shape[1] *  factor)
            height = int(img.shape[0] * factor)
            res_img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

            if vis:
                cv2.imshow('Frame', res_img)
                cv2.waitKey(30)
            
            # Change the image to grayscale
            gray = cv2.cvtColor(res_img, cv2.COLOR_BGR2GRAY)

            #cv2.imwrite(os.path.join(path_output,f'{image_timestamp}.{format}'), gray)
            cv2.imwrite(os.path.join(path_output,f'{image_timestamp}.{format}'), res_img)
            counter += 1

    cap.release()
    if vis: cv2.destroyAllWindows()

    print(f'Frames / Expected: [{counter} / {expected_num_frames}]')

    return



if __name__ == '__main__':
    main()