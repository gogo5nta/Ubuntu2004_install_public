# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

# examples/Python/ReconstructionSystem/sensors/realsense_recorder.py
# Ubuntu20.04.4 + ROS(melodic)
# OpenCV 4.5.5.64 (4.x)
# OpenCV document https://docs.opencv.org/4.5.5/

# update 2022.04.19

# pyrealsense2 is required.
# Please see instructions in https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python
import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
from os import makedirs
from os.path import exists, join
import shutil
import json
from enum import IntEnum

try:
    # Python 2 compatible
    input = raw_input
except NameError:
    pass


class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


def make_clean_folder(path_folder):
    if not exists(path_folder):
        makedirs(path_folder)
    else:
        user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
        if user_input.lower() == 'y':
            shutil.rmtree(path_folder)
            makedirs(path_folder)
        else:
            exit()


def save_intrinsic_as_json(filename, frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    with open(filename, 'w') as outfile:
        obj = json.dump(
            {
                'width':
                    intrinsics.width,
                'height':
                    intrinsics.height,
                'intrinsic_matrix': [
                    intrinsics.fx, 0, 0, 0, intrinsics.fy, 0, intrinsics.ppx,
                    intrinsics.ppy, 1
                ]
            },
            outfile,
            indent=4)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description=
        "Realsense Recorder. Please select one of the optional arguments")
    parser.add_argument("--output_folder",
                        default='../dataset/realsense/',
                        help="set output folder")
    parser.add_argument("--record_rosbag",
                        action='store_true',
                        help="Recording rgbd stream into realsense.bag")
    parser.add_argument(
        "--record_imgs",
        action='store_true',
        help="Recording save color and depth images into realsense folder")
    parser.add_argument("--playback_rosbag",
                        action='store_true',
                        help="Play recorded realsense.bag file")
    args = parser.parse_args()

    if sum(o is not False for o in vars(args).values()) != 2:
        parser.print_help()
        exit()

    path_output = args.output_folder
    path_depth = join(args.output_folder, "depth")
    path_color = join(args.output_folder, "rgb")

    if args.record_imgs:
        make_clean_folder(path_output)
        make_clean_folder(path_depth)
        make_clean_folder(path_color)

    path_bag = join(args.output_folder, "realsense.bag")
    if args.record_rosbag:
        if exists(path_bag):
            user_input = input("%s exists. Overwrite? (y/n) : " % path_bag)
            if user_input.lower() == 'n':
                exit()

    # Create a pipeline
    pipeline = rs.pipeline()

    #Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()


    ctx = rs.context()
    device_list = ctx.query_devices()
    num_devices = device_list.size()

    device = device_list[0]
    sensors = device.query_sensors()

    color_idx = -1
    for i in range(len(sensors)):
        if not sensors[i].is_depth_sensor():
            color_idx = i
            break

    #color_sensor = sensors[color_idx]
    #color_sensor.set_option(rs.option.enable_auto_exposure, 0) # on
    #color_sensor.set_option(rs.option.exposure, 200.000)

    framerate = 30

    if args.record_imgs or args.record_rosbag:
        # note: using 640 x 480 depth resolution produces smooth depth boundaries
        #       using rs.format.bgr8 for color image format for OpenCV based image visualization
        #config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, framerate)
        #config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, framerate)
        #config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, framerate)
        if args.record_rosbag:
            config.enable_record_to_file(path_bag)
    if args.playback_rosbag:
        config.enable_device_from_file(path_bag, repeat_playback=True)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()

    # Using preset HighAccuracy for recording
    if args.record_rosbag or args.record_imgs:
        depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_scale = depth_sensor.get_depth_scale()

    # We will not display the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 5  # 3 meter
    clipping_distance = clipping_distance_in_meters / depth_scale
    print(1.0 / depth_scale)

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    #create rgb.txt depth.txt
    rgb_file_name = path_output + '/rgb.txt'
    rgb_file = open(rgb_file_name,"w")

    rgb_file.writelines("# color images\n")
    rgb_file.writelines("# file: 'xxx.bag'\n")
    rgb_file.writelines("# timestamp filename\n")

    depth_file_name = path_output + '/depth.txt'
    depth_file = open(depth_file_name,"w")

    depth_file.writelines("# color images\n")
    depth_file.writelines("# file: 'xxx.bag'\n")
    depth_file.writelines("# timestamp filename\n")




    # Streaming loop
    frame_count = 0
    frame_num = 0
    hundred_thousand = 100000
    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            if args.record_imgs:
                if frame_count == 0:
                    save_intrinsic_as_json(
                        join(args.output_folder, "camera_intrinsic.json"),
                        color_frame)
                if frame_count % 1 == 0:
                    if frame_count > 10:
                        ts = frame_num * 1/float(framerate)
                        ts_int = int(ts * 1000) + hundred_thousand
                        rgb_file.writelines(str(ts_int))
                        rgb_file.writelines(" rgb/")
                        rgb_file.writelines("%06d.jpg" % \
                            (ts_int))
                        rgb_file.writelines("\n")
                        depth_file.writelines(str(ts_int))
                        depth_file.writelines(" depth/")
                        depth_file.writelines("%06d.png" % \
                            (ts_int))
                        depth_file.writelines("\n")
                        height,width = depth_image.shape
                        bunkatsu = 8
                        
                        # cv2.rectangle(img, pt1(int, int), pt2(int, int), 0, CV2.FILLED)
                        depth_image = cv2.rectangle(depth_image,(0,0),(width,int(height/bunkatsu)),0,cv2.FILLED)
                        depth_image = cv2.rectangle(depth_image,(0,0),(int(width/bunkatsu),height),0,cv2.FILLED)
                        depth_image = cv2.rectangle(depth_image,(0,int((bunkatsu-1)*height/bunkatsu)),(width,height),0,cv2.FILLED)
                        depth_image = cv2.rectangle(depth_image,(int((bunkatsu-1)*width/bunkatsu),0),(width,height),0,cv2.FILLED)
                    
                        cv2.imwrite("%s/%06d.png" % \
                            (path_depth, ts_int), depth_image)
                        cv2.imwrite("%s/%06d.jpg" % \
                            (path_color, ts_int), color_image)
                        """
                        cv2.imwrite("%s/%06d.png" % \
                            (path_depth, frame_num), depth_image)
                        cv2.imwrite("%s/%06d.jpg" % \
                            (path_color, frame_num), color_image)
                        """
                        frame_num += 1
                        print("Saved color + depth image %06d" % frame_num)
                frame_count += 1

            #cv2.namedWindow('Recorder Realsense', cv2.WINDOW_AUTOSIZE)

            # resize images
            rate1 = 0.50
            color_image_resize = cv2.resize(color_image, (int(color_image.shape[1]*rate1), int(color_image.shape[0]*rate1))) 

            # render color_image
            cv2.imshow('Recorder Realsense color', color_image_resize)
            #cv2.imshow('Recorder Realsense color', color_image)

            # Remove background - Set pixels further than clipping_distance to grey
            grey_color = 153
            #depth image is 1 channel, color is 3 channels
            depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
            bg_removed = np.where((depth_image_3d > clipping_distance) | \
                    (depth_image_3d <= 0), grey_color, color_image)

            # Render images
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.09), cv2.COLORMAP_JET)
            images = np.hstack((bg_removed, depth_colormap))
            cv2.namedWindow('Recorder Realsense bg_remove', cv2.WINDOW_AUTOSIZE)

            # resize images
            rate2 = 0.25
            images_resize = cv2.resize(images, (int(images.shape[1]*rate2), int(images.shape[0]*rate2))) 

            # render bg_removed
            cv2.imshow('Recorder Realsense bg_remove', images_resize)
            #cv2.imshow('Recorder Realsense bg_remove', images)
            key = cv2.waitKey(1)

            # if 'esc' button pressed, escape loop and exit program
            if key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()
