#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/videoio/videoio.hpp>
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

#define PARAM_IMAGES_PATH "/stereo_camera_calibration/calibration/intrinsic/images_path"
#define PARAM_OUTPUT_FILE "/stereo_camera_calibration/calibration/intrinsic/output_file"
#define PARAM_NUM_CENTERS_X "/stereo_camera_calibration/calibration/number_of_centers_x"
#define PARAM_NUM_CENTERS_Y "/stereo_camera_calibration/calibration/number_of_centers_y"
#define PARAM_SQUARE_SIZE "/stereo_camera_calibration/calibration/square_size"

std::string images_path;
std::string output_file;
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

    if (pattern_found)
    {

        cornerSubPix(gray, corners, Size(5, 5), Size(-1, -1),
                     TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
    }
    /*bool success = false;
        if(pattern_found){
            vector<Point2f> points = {corners[0], corners[pattern_size.width - 1],
            corners[pattern_size.width*(pattern_size.height -1)],
            corners[pattern_size.width*pattern_size.height - 1]};
            int offsetx = gray.cols*0.2;
            int offsety = gray.rows*0.2;
            vector<Point2f> new_points = {Point2f(offsetx,offsety), Point2f(gray.cols -offsetx , offsety),
            Point2f(offsetx, gray.rows -offsety), Point2f(gray.cols-offsetx, gray.rows- offsety)};

            Mat H = getPerspectiveTransform(points, new_points);

            Mat dst;
            warpPerspective(gray, dst, H, Size(gray.cols, gray.rows));

            namedWindow("dst", WINDOW_NORMAL);
            resizeWindow("dst", dst.cols/2, dst.rows/2);
            imshow("dst", dst);



            vector<Point2f> corners_aux;
             success = findChessboardCorners(dst, pattern_size, corners_aux,
                    CALIB_CB_ADAPTIVE_THRESH |  CALIB_CB_NORMALIZE_IMAGE);

            if(success){
                corners.clear();

                cornerSubPix(dst, corners_aux, Size(5, 5), Size(-1, -1),
                        TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                cv::perspectiveTransform(corners_aux,corners,H.inv());
            }else{
                cout << "Pattern was not detected in unwarped image" << endl;
            }

        }else{
            cout << "Pattern was not detected" << endl;
    }*/

    return pattern_found;
}

bool runCalibration(Mat &img, Mat &K, Mat &dist_coeffs,
                    vector<vector<Point3f>> &objPoints, vector<vector<Point2f>> &imgPoints,
                    Size &square_pattern_size, Mat &stdDeviationsIntrinsics, Mat &stdDeviationsExtrinsics,
                    Mat &perViewErrors, double &totalAvgErr)
{
    bool success;

    Mat gray;
    cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    vector<Point2f> corners;
    success = detectSquares(gray, square_pattern_size, corners);

    if (success)
    {

        drawChessboardCorners(img, square_pattern_size, corners, success);
        namedWindow("corners", WINDOW_NORMAL);
        resizeWindow("corners", img.cols / 2, img.rows / 2);
        imshow("corners", img);
        waitKey(500);
        // waitKey(0);

        vector<Mat> rvecs, tvecs;

        imgPoints.push_back(corners);
        objPoints.resize(imgPoints.size(), objPoints[0]);

        double rms;

        cv::Mat diff;
        cv::compare(K, Mat::eye(3, 3, CV_64F), diff, cv::CMP_NE);
        int nz = cv::countNonZero(diff);

        if (nz == 0)
        {
            cout << "initial estimation of focal length" << endl;

            rms = calibrateCamera(objPoints, imgPoints, Size(img.cols, img.rows), K,
                                  dist_coeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics,
                                  perViewErrors, CALIB_FIX_PRINCIPAL_POINT | CALIB_FIX_ASPECT_RATIO | CALIB_ZERO_TANGENT_DIST | CALIB_FIX_K1 | CALIB_FIX_K2 | CALIB_FIX_K3);
        }
        else if (objPoints.size() == 2)
        {
            cout << "initial estimation of distortion" << endl;
            rms = calibrateCamera(objPoints, imgPoints, Size(img.cols, img.rows), K,
                                  dist_coeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics,
                                  perViewErrors, CALIB_USE_INTRINSIC_GUESS | CALIB_FIX_PRINCIPAL_POINT | CALIB_FIX_ASPECT_RATIO | CALIB_ZERO_TANGENT_DIST);
        }
        else
        {
            rms = calibrateCamera(objPoints, imgPoints, Size(img.cols, img.rows), K,
                                  dist_coeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics,
                                  perViewErrors, CALIB_USE_INTRINSIC_GUESS | CALIB_FIX_ASPECT_RATIO | CALIB_ZERO_TANGENT_DIST | CALIB_FIX_PRINCIPAL_POINT);
        }

        cout << "K: " << K << endl;
        cout << "Distortion coeffs: " << dist_coeffs << endl;
        // cout << "dist_coeffs: " << dist_coeffs_proj << endl;
        cout << "Re-projection error reported by calibrateCamera: " << rms << endl;
        cout << "Std deviation intrinsics: " << stdDeviationsIntrinsics.t() << endl;
        // cout << "Std deviation extrinsics: " << stdDeviationsExtrinsics.t() << endl;
        cout << "Per view erros " << perViewErrors.t() << endl;
    }

    return success;
}

void drawLines(Mat &img, Size &square_pattern_size)
{
    Mat gray;
    cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    vector<Point2f> corners;
    bool success = findChessboardCorners(gray, square_pattern_size, corners,
                                         CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

    if (success)
    {
        drawChessboardCorners(img, square_pattern_size, corners, success);
        for (int i = 0; i < square_pattern_size.height; i++)
        {

            vector<Point2f> points = {corners.begin() + i * square_pattern_size.width, corners.begin() + i * square_pattern_size.width + square_pattern_size.width - 1};
            Vec4f line_para;
            fitLine(points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);

            // cout << "Line para " << line_para << endl;

            Point2f point0;
            point0.x = line_para[2]; // point on the line
            point0.y = line_para[3];
            double k = line_para[1] / line_para[0]; // slope
            /*double error = 0;
            for(int j=0; j < points.size(); j++){
                Point2f p = points[j];
                Point2f p_;
                p_.y = k * (p.x - point0.x) + point0.y;
                p_.x = p.x;
                error += cv::norm(p-p_);
            }

            cout << "error: " << error << endl;*/

            // calculate the endpoint of the line (y = k(x - x0) + y0)
            Point2f point1, point2;
            point1.x = 0;
            point1.y = k * (0 - point0.x) + point0.y;
            point2.x = img.cols;
            point2.y = k * (img.cols - point0.x) + point0.y;

            line(img, point1, point2, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
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

void getStringParameter(ros::NodeHandle& n, const char *name, std::string &output, bool is_a_file = false)
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
    getStringParameter(nh, PARAM_IMAGES_PATH, images_path);
    getStringParameter(nh, PARAM_OUTPUT_FILE, output_file);
    getIntParameter(nh, PARAM_NUM_CENTERS_X, number_of_centers_x);
    getIntParameter(nh, PARAM_NUM_CENTERS_Y, number_of_centers_y);
    getDoubleParameter(nh, PARAM_SQUARE_SIZE, square_size);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "intrinsic_calibration_node");
    ros::NodeHandle nh;

    loadParameters(nh);
    
    Size square_pattern_size(number_of_centers_x, number_of_centers_y);
    
    cv::Mat K, dist_coeffs;

    K = Mat::eye(3, 3, CV_64F);
    dist_coeffs = Mat::zeros(8, 1, CV_64F);

    vector<Point3f> worldPts;
    vector<vector<Point3f>> objPoints(0);
    for (int i = 0; i < square_pattern_size.height; ++i)
        for (int j = 0; j < square_pattern_size.width; ++j)
            worldPts.push_back(Point3f(float(j * square_size), float(i * square_size), 0));

    objPoints.push_back(worldPts);

    cv::String imgs_path = images_path + std::string("*.png");
    vector<cv::String> fn;

    cv::glob(imgs_path, fn, true); // recurse

    vector<vector<Point2f>> img1Points(0);
    vector<float> reprojErrs1;
    double totalAvgErr1;
    Mat stdDeviationsIntrinsics;
    Mat stdDeviationsExtrinsics;
    Mat perViewErrors;

    Mat img;

    for (int i = 0; i < fn.size(); i += 1)
    {
        cout << fn[i] << endl;
        img = cv::imread(fn[i], IMREAD_COLOR);

        bool a = runCalibration(img, K, dist_coeffs, objPoints, img1Points, square_pattern_size,
                                stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors, totalAvgErr1);
    }

    Size imageSize(img.cols, img.rows);
    Mat view, rview, map1, map2;
    initUndistortRectifyMap(K, dist_coeffs, Mat(),
                            getOptimalNewCameraMatrix(K, dist_coeffs, imageSize, 1, imageSize, 0),
                            imageSize, CV_16SC2, map1, map2);

    for (int i = 0; i < fn.size(); i++)
    {
        view = imread(fn[i], 1);
        if (view.empty())
            continue;
        remap(view, rview, map1, map2, INTER_LINEAR);

        drawLines(rview, square_pattern_size);

        std::string filename = images_path + "/output/" + std::to_string(i + 1) + ".jpeg";

        imwrite(filename, rview);
        imshow("rview", rview);
        waitKey(0);
    }

    // Save file
    cv::FileStorage fs;
    fs.open(output_file, FileStorage::WRITE);
    fs << "K_cam" << K;
    fs << "dist_coeffs_cam" << dist_coeffs;
    fs << "img_width" << imageSize.width;
    fs << "img_height" << imageSize.height;
    fs << "reproj_error_avg" << totalAvgErr1;
    fs << "std_deviation_intrinsics" << stdDeviationsIntrinsics;
    fs << "per_view_errors" << perViewErrors;
    fs.release();

    ROS_INFO("intrinsic calibration ok");
    return 0;
}
