#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/photo/photo.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <unistd.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <ros/ros.h>

using namespace std;
using namespace cv;

#define PARAM_CAM_1_VIDEO_FILE "/stereo_camera_calibration/calibration/stereo/cam_1_video_file"
#define PARAM_CAM_2_VIDEO_FILE "/stereo_camera_calibration/calibration/stereo/cam_2_video_file"
#define PARAM_CAM_1_CALIBRATION_FILE "/stereo_camera_calibration/calibration/stereo/cam_1_calibration_file"
#define PARAM_CAM_2_CALIBRATION_FILE "/stereo_camera_calibration/calibration/stereo/cam_2_calibration_file"
#define PARAM_OUTPUT_PATH "/stereo_camera_calibration/calibration/stereo/output_path"
#define PARAM_OUTPUT_FILE "/stereo_camera_calibration/calibration/stereo/output_file"
#define PARAM_MIN_FRAME "/stereo_camera_calibration/calibration/stereo/min_frame"
#define PARAM_MAX_FRAME "/stereo_camera_calibration/calibration/stereo/max_frame"
#define PARAM_NUM_CENTERS_X "/stereo_camera_calibration/calibration/number_of_centers_x"
#define PARAM_NUM_CENTERS_Y "/stereo_camera_calibration/calibration/number_of_centers_y"
#define PARAM_SQUARE_SIZE "/stereo_camera_calibration/calibration/square_size"

std::string cam_1_calibration_file;
std::string cam_2_calibration_file;
std::string cam_1_video_file;
std::string cam_2_video_file;
std::string output_path;
std::string output_file;
int min_frame;
int max_frame;
int number_of_centers_x;
int number_of_centers_y;
double square_size;

void convertObjPoints2CamPoints(vector<Point3f> &obj_points, vector<Point3f> &cam_points, Mat rvec, Mat tvec)
{
    Mat rvec_32, tvec_32;
    rvec.convertTo(rvec_32, CV_32F);
    tvec.convertTo(tvec_32, CV_32F);

    cam_points.clear();
    Mat R3x3;
    Rodrigues(rvec_32, R3x3);

    for (int i = 0; i < obj_points.size(); i++)
    {
        Mat p = Mat::zeros(3, 1, CV_32F);

        p.at<float>(0, 0) = obj_points[i].x;
        p.at<float>(1, 0) = obj_points[i].y;
        p.at<float>(2, 0) = obj_points[i].z;

        Mat p_cam = R3x3 * p + tvec_32;

        // cout << "Object point: " << p.t() << endl;
        //  cout << "Cam point " << p_cam.t() << endl;

        Point3f point;
        point.x = p_cam.at<float>(0, 0);
        point.y = p_cam.at<float>(0, 1);
        point.z = p_cam.at<float>(0, 2);

        cam_points.push_back(point);
    }
}

static double computeReprojectionErrors(const vector<vector<Point3f>> &objectPoints,
                                        const vector<vector<Point2f>> &imagePoints,
                                        const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                                        const Mat &cameraMatrix, const Mat &distCoeffs,
                                        vector<float> &perViewErrors)
{

    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int)objectPoints.size(); ++i)
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                      distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}

bool detectSquares(Mat &gray, Size pattern_size, vector<Point2f> &corners)
{
    // in order to speed up the detection of chessboard patter we resize the image
    bool pattern_found;

    // Mat gray_resized;
    // resize(gray, gray_resized, Size(0, 0), 0.25, 0.25);
    pattern_found = findChessboardCorners(gray, pattern_size, corners,
                                          CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

    //    for (int i = 0; i < corners.size(); i++) {
    //        corners[i].x *= 4;
    //        corners[i].y *= 4;
    //    }
    if (pattern_found)
        cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                     TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));

    return pattern_found;
}

static void runCalibration(Mat &img, Mat &K, Mat &dist_coeffs,
                           vector<vector<Point3f>> &objPoints, vector<vector<Point2f>> &imgPoints,
                           vector<Point2f> &corners, vector<float> &reprojErrs, double &totalAvgErr)
{

    vector<Mat> rvecs, tvecs;

    imgPoints.push_back(corners);
    objPoints.resize(imgPoints.size(), objPoints[0]);

    K = Mat::eye(3, 3, CV_64F);
    dist_coeffs = Mat::zeros(8, 1, CV_64F);

    double rms = calibrateCamera(objPoints, imgPoints, Size(img.cols, img.rows), K,
                                 dist_coeffs, rvecs, tvecs);

    cout << "K: " << K << endl;
    cout << "Distortion coeffs: " << dist_coeffs << endl;
    // cout << "dist_coeffs: " << dist_coeffs_proj << endl;
    cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

    totalAvgErr = computeReprojectionErrors(objPoints, imgPoints,
                                            rvecs, tvecs, K, dist_coeffs, reprojErrs);
}

void shutdown_tracker(int error)
{
    fprintf(stdout, "\nShutdown.\n");
    exit(error);
}

void checkIfFileExists(const std::string &file)
{
    struct stat buffer;
    ROS_DEBUG("Checking if file exists: %s", file.c_str());
    ROS_ASSERT(stat(file.c_str(), &buffer) == 0);
}

void getStringParameter(ros::NodeHandle& n, const char *name, std::string &output, bool is_a_file)
{
    ROS_ASSERT(n.getParam(name, output));
    if (is_a_file)
    {
        checkIfFileExists(output);
    }
    ROS_DEBUG("%s = %s", name, output.c_str());
}

void getIntParameter(ros::NodeHandle& n, const char *name, int &output)
{
    ROS_ASSERT(n.getParam(name, output));
    ROS_DEBUG("%s = %d", name, output);
}

void getDoubleParameter(ros::NodeHandle& n, const char *name, double &output)
{
    XmlRpc::XmlRpcValue tmp;
    ROS_ASSERT(n.getParam(name, tmp));
    ROS_ASSERT(tmp.getType() == XmlRpc::XmlRpcValue::TypeDouble);
    output = tmp;
    ROS_DEBUG("%s = %f", name, output);
}

void loadParameters(ros::NodeHandle &nh)
{
    ROS_DEBUG("getting parameters");
    getStringParameter(nh, PARAM_CAM_1_VIDEO_FILE, cam_1_video_file, true);
    getStringParameter(nh, PARAM_CAM_2_VIDEO_FILE, cam_2_video_file, true);
    getStringParameter(nh, PARAM_CAM_1_CALIBRATION_FILE, cam_1_calibration_file, true);
    getStringParameter(nh, PARAM_CAM_2_CALIBRATION_FILE, cam_2_calibration_file, true);
    getStringParameter(nh, PARAM_OUTPUT_PATH, output_path, true);
    getStringParameter(nh, PARAM_OUTPUT_FILE, output_file, true);
    getIntParameter(nh, PARAM_MIN_FRAME, min_frame);
    getIntParameter(nh, PARAM_MAX_FRAME, max_frame);
    getIntParameter(nh, PARAM_NUM_CENTERS_X, number_of_centers_x);
    getIntParameter(nh, PARAM_NUM_CENTERS_Y, number_of_centers_y);
    getDoubleParameter(nh, PARAM_SQUARE_SIZE, square_size);
    ROS_DEBUG("---");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_calibration_from_video_node");
    ros::NodeHandle nh;

    loadParameters(nh);

    Size square_pattern_size(number_of_centers_x, number_of_centers_y); // number of centers

    signal(SIGINT, shutdown_tracker);
    signal(SIGTERM, shutdown_tracker);

    // string path;

    cv::Mat K_cam1, dist_coeffs_cam1, K_cam2, dist_coeffs_cam2;
    int img_width_cam1, img_height_cam1, img_width_cam2, img_height_cam2;

    cv::FileStorage fs;
    ROS_DEBUG("opennig calibration file 1");
    fs.open(cam_1_calibration_file, FileStorage::READ);
    fs["K_cam"] >> K_cam1;
    fs["dist_coeffs_cam"] >> dist_coeffs_cam1;
    fs["img_width"] >> img_width_cam1;
    fs["img_height"] >> img_height_cam1;
    fs.release();

    ROS_DEBUG("opennig calibration file 2");
    fs.open(cam_2_calibration_file, FileStorage::READ);
    fs["K_cam"] >> K_cam2;
    fs["dist_coeffs_cam"] >> dist_coeffs_cam2;
    fs["img_width"] >> img_width_cam2;
    fs["img_height"] >> img_height_cam2;

    cout << "K_cam1: " << K_cam1 << endl;

    Size img1Size(img_width_cam1, img_height_cam1);
    Size img2Size(img_width_cam2, img_height_cam2);

    vector<Point3f> worldPts;
    vector<vector<Point3f>> objPoints(0);
    for (int i = 0; i < square_pattern_size.height; ++i)
        for (int j = 0; j < square_pattern_size.width; ++j)
            worldPts.push_back(Point3f(float(j * square_size), float(i * square_size), 0));

    objPoints.push_back(worldPts);

    // vector<float> reprojErrs;
    // double totalAvgErr;

    ROS_DEBUG("opening video 1: %s", cam_1_video_file.c_str());
    ROS_DEBUG("opening video 2: %s", cam_2_video_file.c_str());
    VideoCapture cap1(cam_1_video_file);
    VideoCapture cap2(cam_2_video_file);

    // Check if camera opened successfully
    if (!cap1.isOpened() || !cap2.isOpened())
    {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    vector<vector<Point2f>> img1Points(0);
    vector<vector<Point2f>> img2Points(0);
    vector<float> reprojErrs1, reprojErrs2;
    double totalAvgErr1, totalAvgErr2;

    bool success1, success2;

    // string path_images = path + "result_images/";

    // Check if directory exists
    struct stat st;
    if (stat(output_path.c_str(), &st) == 0)
    {
        printf("output_path/ is present\n");
    }
    else
    {
        const int dir_err = mkdir(output_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (-1 == dir_err)
        {
            printf("Error creating directory!n");
            exit(1);
        }
    }

    int i = 0;
    Mat frame1, frame2, gray1, gray2;
    while (1)
    {
        i++;
        // Capture frame-by-frame
        cap1 >> frame1;
        cap2 >> frame2;

        if (frame1.empty() || frame2.empty())
            break;

        if (min_frame > 0 && i < min_frame)
            continue;
        if (max_frame > 0 && i > max_frame)
            break;

        if (i % 5 == 0)
        {
            // convert to grayscale

            cvtColor(frame1, gray1, cv::COLOR_BGR2GRAY);
            cvtColor(frame2, gray2, cv::COLOR_BGR2GRAY);

            vector<Point2f> corners1, corners2;
            success1 = detectSquares(gray1, square_pattern_size, corners1);
            success2 = detectSquares(gray2, square_pattern_size, corners2);

            if (!success1 && !success2)
            {
                fprintf(stdout, "Corners not found in camera 1 and 2\n");
                fflush(stdout);
            }
            else if (!success1)
            {
                fprintf(stdout, "Corners not found in camera 1\n");
                fflush(stdout);
            }
            else if (!success2)
            {
                fprintf(stdout, "Corners not found in camera 2\n");
                fflush(stdout);
            }
            else
            {
                fprintf(stdout, "Corners found!\n");
                fflush(stdout);
                // imwrite(cv::format("/home/acl/Documents/DATA/Stereo_Calibration/losa_nave/result_images/orig_cam1_%d.png", i), frame1);
                // imwrite(cv::format("/home/acl/Documents/DATA/Stereo_Calibration/losa_nave/result_images/orig_cam2_%d.png", i), frame2);
                drawChessboardCorners(frame1, square_pattern_size, Mat(corners1), true);
                drawChessboardCorners(frame2, square_pattern_size, Mat(corners2), true);

                imwrite(output_path + cv::format("cam1_%d.png", i), frame1);
                imwrite(output_path + cv::format("cam2_%d.png", i), frame2);

                img1Points.push_back(corners1);
                img2Points.push_back(corners2);

                //                runCalibration(frame1, K_cam1, dist_coeffs_cam1, objPoints,
                //                        img1Points, corners1, reprojErrs1, totalAvgErr1);

                //                runCalibration(frame2, K_cam2, dist_coeffs_cam2, objPoints,
                //                        img2Points, corners2, reprojErrs2, totalAvgErr2);
            }
        }
    }

    Mat R, T, E, F;

    objPoints.resize(img1Points.size(), objPoints[0]);
    double rms = stereoCalibrate(objPoints, img1Points, img2Points, K_cam1,
                                 dist_coeffs_cam1, K_cam2, dist_coeffs_cam2, img1Size, R, T, E, F,
                                 CALIB_FIX_INTRINSIC, TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 100, 1e-3));

    Mat R1_, R2_, P1, P2, Q;
    cv::stereoRectify(K_cam1, dist_coeffs_cam1, K_cam2, dist_coeffs_cam2, img1Size, R, T,
                      R1_, R2_, P1, P2, Q);

    // string filename = path + "config_data_cam1_cam2.xml";

    ROS_DEBUG("opennig output file");
    fs.open(output_file, FileStorage::WRITE);
    // Save file
    fs << "K_cam1" << K_cam1;
    fs << "dist_coeffs_cam1" << dist_coeffs_cam1;
    fs << "img_width_cam1" << img_width_cam1;
    fs << "img_height_cam1" << img_height_cam1;
    fs << "K_cam2" << K_cam2;
    fs << "dist_coeffs_cam2" << dist_coeffs_cam2;
    fs << "img_width_cam2" << img_width_cam2;
    fs << "img_height_cam2" << img_height_cam2;
    fs << "R" << R;
    fs << "T" << T;
    fs << "E" << E;
    fs << "F" << F;
    fs << "R1" << R1_;
    fs << "R2" << R2_;
    fs << "P1" << P1;
    fs << "P2" << P2;
    fs << "rms" << rms;
    fs.release();

    cap1.release();
    cap2.release();

    Mat R1, T1, R2, T2;
    bool found_pose;
    vector<Point3f> camPoints1, camPoints2;

    for (int j = 0; j < img1Points.size(); j++)
    {
        found_pose = solvePnP(objPoints[0], img1Points[j], K_cam1, dist_coeffs_cam1, R1, T1);
        found_pose = solvePnP(objPoints[0], img2Points[j], K_cam2, dist_coeffs_cam2, R2, T2);

        convertObjPoints2CamPoints(objPoints[0], camPoints1, R1, T1);
        convertObjPoints2CamPoints(objPoints[0], camPoints2, R2, T2);

        cout << "Check result: " << endl;

        R.convertTo(R, CV_32F);
        T.convertTo(T, CV_32F);

        for (int i = 0; i < camPoints1.size(); i++)
        {

            cout << "p1: " << camPoints1[i] << endl;
            cout << "p2: " << camPoints2[i] << endl;
            Mat p1 = Mat::zeros(3, 1, CV_32F);

            p1.at<float>(0, 0) = camPoints1[i].x;
            p1.at<float>(1, 0) = camPoints1[i].y;
            p1.at<float>(2, 0) = camPoints1[i].z;

            Mat p2 = R * p1 + T;

            // cout << "Object point: " << p.t() << endl;
            cout << "Recalculated p2 " << p2.t() << endl;
        }
    }

    cout << "rms: " << rms << endl;

    ROS_INFO("stereo calibration from video ok");
    return 0;
}
