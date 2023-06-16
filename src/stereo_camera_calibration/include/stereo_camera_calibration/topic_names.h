#ifndef TOPIC_NAMES_H
#define TOPIC_NAMES_H

#define PARAM_VIDEO_PATH_1 "/stereo_camera_calibration/record/video_path_1"
#define PARAM_VIDEO_PATH_2 "/stereo_camera_calibration/record/video_path_2"
#define PARAM_IMAGES_PATH_1 "/stereo_camera_calibration/record/images_path_1"
#define PARAM_IMAGES_PATH_2 "/stereo_camera_calibration/record/images_path_2"
#define PARAM_CAM_1_WIDTH "/stereo_camera_calibration/record/cam_1_width"
#define PARAM_CAM_2_WIDTH "/stereo_camera_calibration/record/cam_2_width"
#define PARAM_CAM_1_HEIGHT "/stereo_camera_calibration/record/cam_1_height"
#define PARAM_CAM_2_HEIGHT "/stereo_camera_calibration/record/cam_2_height"
#define PARAM_FPS "/stereo_camera_calibration/record/fps"

#define TOPIC_CAM_1_IMG "/camera1/image_raw"
#define TOPIC_CAM_2_IMG "/camera2/image_raw"
#define SERVICE_START_VIDEO_RECORDING "start_video_recording"
#define SERVICE_STOP_VIDEO_RECORDING "stop_video_recording"
#define SERVICE_TAKE_CAPTURE "take_capture"
#define IMAGE_1_NAME "cam_1_capture_"
#define IMAGE_2_NAME "cam_2_capture_"

#define MSG_START_VIDEO_RECORDING "Start video recording"
#define MSG_STOP_VIDEO_RECORDING "Stop video recording"
#define MSG_RECORDING "Video is already recording"
#define MSG_NOT_RECORDING "Video is not recording"
#define MSG_CAPTURE_TAKEN "Capture taken"
#define MSG_WRONG_IMAGE_SIZE "Wrong image size"

#endif