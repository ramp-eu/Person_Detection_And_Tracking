/*
 * File:   main.cpp
 * Author: adrianacostas
 *
 * Created on June 13, 2017, 1:29 PM
 */

#include <opencv2/core/ocl.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/videoio.hpp> // Video write
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core/utility.hpp>

#include <iostream>
#include <fstream>
#include <random>
#include <set>
#include <cmath>
#include <vector>
#include <time.h>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
//juan
#include <mapupdates/NewObstacles.h>
//end juan

#include "Hungarian.h"
#include "TrackerClass.h"
#include "Tracking.h"
#include "CameraIDS.h"
#include "Detection.h"

using namespace cv;
using namespace std;
using namespace cv::dnn;

#define STOP 0
#define INIT 1
#define READING 2
#define DETECT 3
#define TRACK 4
#define PUBLISH 5


 ros::NodeHandle* nh;
ros::Publisher pub_worker_joints;
//juan
ros::Publisher pub_worker_obstacles;
//end juan
geometry_msgs::PoseArray worker_joints_msg;
//juan
mapupdates::NewObstacles newobstacles_msg;
//end juan

////////////////////////////////////////////////////////
/*++++++++++++++++++++ YOLO v3 +++++++++++++++++++++++*/
////////////////////////////////////////////////////////
Net net_yolo;
float confThreshold = 0.7;         // Confidence threshold 0.7
float nmsThreshold = 0.7;          // Non-maximum suppression threshold 0.45
int inpWidth = 416;                // Width of network's input image
int inpHeight = 416 * 1088 / 2048; // Height of network's input image
vector<string> classes;
//Params
int cam1_id, cam2_id;
String cam_calibration_file, cam_map_calibration_file;
String yolo_classes_file, yolo_model_file, yolo_weights_file;
String video_path;
double exp_t1, exp_t2, fps;

volatile int stop = 0;
CameraIDS cam1, cam2;
double start, finish;
int no_frames = 0;
int state;
vector<Detection*> detections_cam1, detections_cam2;
cv::Mat K_cam1, dist_coeffs_cam1, K_cam2, dist_coeffs_cam2, R, T, P1, P2, R1, R2, Q, F;
int img_width_cam1, img_height_cam1, img_width_cam2, img_height_cam2;
Mat Rcam_map, tcam_map;
Mat frame_1, frame_2;
Mat aux_1, aux_2;
Mat undistorted_img1, undistorted_img2;
Mat map1_cam1, map2_cam1, map1_cam2, map2_cam2;
Tracking tracking_node;
       
cv::VideoWriter tracking_video;
bool first = true;

double get_current_time(void) {
    struct timespec time;
    clock_gettime(CLOCK_REALTIME, &time);
    return (time.tv_sec + time.tv_nsec / 1.0e09);
}
void shutdown_tracker(int error) {

    stop = 1;
    cam1.stop = 1;
    cam2.stop = 1;
}


void postprocess(cv::Mat& frame, const vector<cv::Mat>& outs,  vector<Detection*> &detections) {
    vector<int> classIds;
    vector<float> confidences;
    vector<cv::Rect2d> boxes;

    vector<cv::Point2f> boxsPosition;
    vector<cv::Rect2d> boxesInfo;
    for (size_t i = 0; i < outs.size(); ++i) {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*) outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if ((confidence > confThreshold) && (classIdPoint.x == 0)) //////////COPSSSSSSSSSS
            {
                int centerX = (int) (data[0] * frame.cols);
                int centerY = (int) (data[1] * frame.rows);
                int width = (int) (data[2] * frame.cols);
                int height = (int) (data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                //cout<< "we got any box good"<<endl;
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float) confidence);
                boxes.push_back(cv::Rect2d(left, top, width, height));
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

    bool detected = false;
    for (size_t i = 0; i < indices.size(); ++i) {
        detected = true;
        int idx = indices[i];
        cv::Rect2d box = boxes[idx];

        //cout<<box.x<<" "<<box.y<< " "<<box.x + box.width<<" "<<box.y + box.height<<endl;
        box.x -= 50;
        box.y -= 50;
        box.width += 100;
        box.height += 100;


        if (box.x < 0) {
            box.x = 0;
        }
        if (box.y < 0) {
            box.y = 0;
        }
        if (box.x + box.width > frame.cols) {
            box.width = 2048 - box.x;
        }//max range
        if (box.y + box.height > frame.rows) {
            box.height = 1088 - box.y;
        }

        //if we want the center position of boxs, uncomment the next and change centerX instead of box.x
        int centerX = box.x + box.width / 2;
        int centerY = box.y + box.height / 2;
        cv::Point2f box_position(centerX, centerY);

        boxsPosition.push_back(box_position);

        boxesInfo.push_back(box);

        Detection* person = new Detection(box);
        detections.push_back(person);


    }


}

// Get the names of the output layers

vector<string> getOutputsNames(const Net &net)
{
    static vector<string> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<string> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

void peopleDetection(Net& net_, cv::Mat& img, int inpWidth, int inpHeight, vector<Detection*> &detections) {
    // Create a 4D blob from a frame.
    cv::Mat blob;
    blobFromImage(img, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(0, 0, 0), true, false);

    //Sets the input to the network
    net_.setInput(blob);

    // Runs the forward pass to get output of the output layers
    vector<cv::Mat> outs;
    net_.forward(outs, getOutputsNames(net_));
    //cout<<"before postprocces"<<endl;
    // Remove the bounding boxes with low confidence
    postprocess(img, outs, detections);

}

float distancePointLine(cv::Point2f p, cv::Point3f line)
{
    //Line is given as a*x + b*y + c = 0
    //d = (a*p.x + b*p.y+c)/sqrt(a²+b²)
    return std::fabs(line.x * p.x + line.y * p.y + line.z) / std::sqrt(line.x * line.x + line.y * line.y);
}

void stereo_matching(vector<Detection*> &detections_cam1, vector<Detection*> &detections_cam2,
        Mat &F, vector<int> &assignment, vector<vector<double> > &cost_matrix, Mat img1 = Mat(), Mat img2 = Mat()) {


    vector<cv::Point2f> boxsPosition_cam1, boxsPosition_cam2;
    for (int i = 0; i < detections_cam1.size(); i++) {
        boxsPosition_cam1.push_back(detections_cam1[i]->getCenter());
        //cout << "x " << boxsPosition_cam1[i].x << ", y: " << boxsPosition_cam1[i].y << endl;
    }
    for (int i = 0; i < detections_cam2.size(); i++) {
        boxsPosition_cam2.push_back(detections_cam2[i]->getCenter());
        //cout << "x " << boxsPosition_cam2[i].x << ", y: " << boxsPosition_cam2[i].y << endl;
    }

    vector<cv::Point3f> lines_img1, lines_img2;
    //Line is given as a*x + b*y + c = 0
    computeCorrespondEpilines(boxsPosition_cam1, 1, F, lines_img2);
    computeCorrespondEpilines(boxsPosition_cam2, 2, F, lines_img1);


    if (!img1.empty() && !img2.empty()) {
        Mat output_1, output_2;
        img1.copyTo(output_1);
        img2.copyTo(output_2);


        Scalar color;
        for (size_t i = 0; i < detections_cam2.size(); i++) {
            rectangle(output_2, detections_cam2[i]->getBox(), Scalar(0, 255, 0), 4, 1);
            circle(output_2, detections_cam2[i]->getCenter(), 10, Scalar(0, 0, 255), -1);
            putText(output_2, to_string(i), detections_cam2[i]->getOrigin() - Point2f(0, 10), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
        }

        for (size_t i = 0; i < detections_cam1.size(); i++) {

            color = Scalar(rand() % 255, rand() % 255, rand() % 255);

            cv::line(output_2,
                    cv::Point(0, -lines_img2[i].z / lines_img2[i].y),
                    cv::Point(output_2.cols, -(lines_img2[i].z + lines_img2[i].x * output_2.cols) / lines_img2[i].y),
                    color, 3);
            rectangle(output_1, detections_cam1[i]->getBox(), color, 4, 1);
            cv::circle(output_1, detections_cam1[i]->getCenter(), 10, color, -1, cv::LINE_AA);
            putText(output_1, to_string(i), detections_cam1[i]->getOrigin() - Point2f(0, 10), FONT_HERSHEY_SIMPLEX, 1, color, 2);


        }
        imshow("img1", output_1);
        imshow("img2", output_2);


        img1.copyTo(output_1);
        img2.copyTo(output_2);
        for (size_t i = 0; i < detections_cam1.size(); i++) {
            rectangle(output_1, detections_cam1[i]->getBox(), Scalar(0, 255, 0), 4, 1);
            circle(output_1, detections_cam1[i]->getCenter(), 10, Scalar(0, 0, 255), -1);
            putText(output_1, to_string(i), detections_cam1[i]->getOrigin() - Point2f(0, 10), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
        }
        for (size_t i = 0; i < detections_cam2.size(); i++) {

            color = Scalar(rand() % 255, rand() % 255, rand() % 255);

            cv::line(output_1,
                    cv::Point(0, -lines_img1[i].z / lines_img1[i].y),
                    cv::Point(output_1.cols, -(lines_img1[i].z + lines_img1[i].x * output_1.cols) / lines_img1[i].y),
                    color, 3);
            rectangle(output_2, detections_cam2[i]->getBox(), color, 4, 1);
            cv::circle(output_2, detections_cam2[i]->getCenter(), 10, color, -1, cv::LINE_AA);
            putText(output_2, to_string(i), detections_cam2[i]->getOrigin() - Point2f(0, 10), FONT_HERSHEY_SIMPLEX, 1, color, 2);


        }

        imshow("img1", output_1);
        imshow("img2", output_2);

        
    }


    vector<double> cost_per_line(detections_cam2.size(), 1.);
    cost_matrix.resize(detections_cam1.size(), cost_per_line);


    for (size_t i = 0; i < lines_img2.size(); i++) {
       for (int j = 0; j < boxsPosition_cam2.size(); j++) {

            double dist = distancePointLine(boxsPosition_cam2[j], lines_img2[i]);
            double score = dist;
            //cout << "Box_cam1 " << i << " box_cam2 " << j << " score: " << score << endl;

            cost_matrix[i][j] = score;
        }

    
    }

 

    for (size_t i = 0; i < lines_img1.size(); i++) {
        for (int j = 0; j < boxsPosition_cam1.size(); j++) {

            double dist = distancePointLine(boxsPosition_cam1[j], lines_img1[i]);
            double score = dist;
            //cout << "Box_cam1 " << j << " box_cam2 " << i << " score: " << score << endl;
            cost_matrix[j][i] += score;
        }

    }


    HungarianAlgorithm HungAlgo;
    double cost = HungAlgo.Solve(cost_matrix, assignment);

    //cout << "cost: " << cost << endl;



}


void install_params(){
    
    
    if(!nh->getParam("/tracking/h_cam1",cam1_id)){
        printf("Error, no param h_cam1\n");
        exit(0);
    }
    if(!nh->getParam("/tracking/h_cam2",cam2_id)){
        printf("Error, no param h_cam2\n");
        exit(0);
    }
    
     if(!nh->getParam("/tracking/exp_t1",exp_t1)){
        printf("Error, no param for exp_t1\n");
        exit(0);
    }
    if(!nh->getParam("/tracking/exp_t2",exp_t2)){
        printf("Error, no param for exp_t2\n");
        exit(0);
    }
    
     if(!nh->getParam("/tracking/fps",fps)){
        printf("Error, no param for fps\n");
        exit(0);
    }
    
     if(!nh->getParam("/tracking/cameras_calibration",cam_calibration_file)){
        printf("Error, no param for cameras_calibration\n");
        exit(0);
    }
    
     if(!nh->getParam("/tracking/camera_map_calibration",cam_map_calibration_file)){
        printf("Error, no param for camera_map_calibration\n");
        exit(0);
    }
    
    if(!nh->getParam("/tracking/yolo_classes_file", yolo_classes_file)){
        printf("Error, no param for yolo_classes_file\n");
        exit(0);
    }
   
    if(!nh->getParam("/tracking/yolo_model_file", yolo_model_file)){
        printf("Error, no param for yolo_model_file\n");
        exit(0);
    }
    
    if(!nh->getParam("/tracking/yolo_weights_file", yolo_weights_file)){
        printf("Error, no param for yolo_weights_file\n");
        exit(0);
    }
    
    if(!nh->getParam("/tracking/video_result_path", video_path)){
        printf("Error, no param for video_result_path\n");
        exit(0);
    }
    
    
    
    
}


void run(){

  switch (state){
    case STOP:
    printf("state= STOP\n");  

    cout <<  "System starts in 5 seconds" <<  endl;
    usleep(5000000);
    state = INIT;
      break;

    case INIT:{
      printf("state= INIT\n");
       int success1 = cam1.initialize_camera(LIVE, fps, exp_t1);
     int success2 = cam2.initialize_camera(LIVE, fps, exp_t2);
   

    if (success1 && success2) {
      
        cam1.startCapture();
        usleep(5000000);
        cam2.startCapture();
        state = READING;
        start = get_current_time();
    }
      
      break;
      }
    case READING:{
      //printf("state= READING\n");		
	  
      if (cam1.isFrame() && cam2.isFrame()) {

        no_frames++;
        fprintf(stdout, "-");
        fflush(stdout);

        frame_1 = cam1.getFrame();
        frame_2 = cam2.getFrame();
        
        
        if (no_frames == int(fps)) {
            finish = get_current_time();
            double time  = finish - start ;
            fprintf(stdout, "no_frames: %d \n", no_frames);
            fprintf(stdout, "time: %f \n", finish - start);
            start = get_current_time();
            no_frames = 0;

            if((first && fabs(time - 1.) < 0.05) ){                
                state = DETECT;
                first = false;
            }            
                
        }
        
        if(!first)
             state = DETECT;	
        
      }         
           
      break;
      }
    case DETECT:{
      
        printf("state= DETECT\n");    
       
        remap(frame_1, undistorted_img1, map1_cam1, map2_cam1, INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0));
        remap(frame_2, undistorted_img2, map1_cam2, map2_cam2, INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0));
            
        undistorted_img1.copyTo(aux_1);
        undistorted_img2.copyTo(aux_2);

        detections_cam1.clear();
        detections_cam2.clear();
    
        peopleDetection(net_yolo, undistorted_img1, inpWidth, inpHeight, detections_cam1);
        peopleDetection(net_yolo, undistorted_img2, inpWidth, inpHeight, detections_cam2);       
      
	
        state = TRACK;
       
        break;
    }
    case TRACK:{
      
      printf("state= TRACK\n");

      if(detections_cam1.size() > 0 && detections_cam2.size() > 0){
          if (tracking_node.getTrackers().size() == 0) {
            vector<int> assignment;
            vector<vector<double> > cost_matrix;

            stereo_matching(detections_cam1, detections_cam2, F, assignment, cost_matrix);

            for (unsigned int x = 0; x < assignment.size(); x++) {
                //cout << "**" << endl;
                //std::cout << "Assignment: box_cam1 " << x << ", box_cam2 : " << assignment[x] << endl;

                if (assignment[x] == -1)
                    continue;

                if (cost_matrix[x][assignment[x]] > 50) {
                    //cout << "Bad assigment -> cost:  " << cost_matrix[x][assignment[x]] << endl;
                    continue;
                }

                Point2f stereo_dist = detections_cam2[assignment[x]]->getCenter() - detections_cam1[x]->getCenter();
                double stereo_dist_norm = sqrt(pow(stereo_dist.x, 2) + pow(stereo_dist.y, 2));

               
                double area_ratio = (detections_cam2[assignment[x]]->getHeight() * detections_cam2[assignment[x]]->getWidth()) / (detections_cam1[x]->getHeight() * detections_cam1[x]->getWidth());
                //cout << "height ratio " << area_ratio << " " << 1. / area_ratio << endl;
                if (stereo_dist_norm > 800 || area_ratio < 0.8 || 1. / area_ratio < 0.8) {

                    //cout << "Bad assignment " << stereo_dist_norm << endl;
                    continue;
                }

                
                TrackerClass* tracker = new TrackerClass(detections_cam1[x]->getBox(), detections_cam2[assignment[x]]->getBox());
                tracking_node.add(tracker);
                 //cout << "Creatig new tracker " << tracker->getId() << endl;
            }

            //cout << "Trackers size: " << tracking_node.getTrackers().size() << endl;
            
             if (tracking_node.getTrackers().size() > 0){
            tracking_node.compute3DPoints(K_cam1, K_cam2, R1, R2, P1, P2);
            }
#ifdef SHOW_IMAGES
            for (int i = 0; i < tracking_node.getTrackers().size(); i++) {
                rectangle(undistorted_img1, tracking_node.getTrackers()[i]->getBoxCam1(), tracking_node.getTrackers()[i]->getColor(), 4, 1);
                cv::circle(undistorted_img1, tracking_node.getTrackers()[i]->getBoxCam1Center(), 10, tracking_node.getTrackers()[i]->getColor(), -1, cv::LINE_AA);
                rectangle(undistorted_img2, tracking_node.getTrackers()[i]->getBoxCam2(), tracking_node.getTrackers()[i]->getColor(), 4, 1);
                cv::circle(undistorted_img2, tracking_node.getTrackers()[i]->getBoxCam2Center(), 10, tracking_node.getTrackers()[i]->getColor(), -1, cv::LINE_AA);
                putText(undistorted_img1, to_string(tracking_node.getTrackers()[i]->getId()), Point(tracking_node.getTrackers()[i]->getBoxCam1().x, tracking_node.getTrackers()[i]->getBoxCam1().y - 10), FONT_HERSHEY_SIMPLEX, 1, tracking_node.getTrackers()[i]->getColor(), 2);
                putText(undistorted_img2, to_string(tracking_node.getTrackers()[i]->getId()), Point(tracking_node.getTrackers()[i]->getBoxCam2().x, tracking_node.getTrackers()[i]->getBoxCam2().y - 10), FONT_HERSHEY_SIMPLEX, 1, tracking_node.getTrackers()[i]->getColor(), 2);


                Mat tracking_stereo;
                hconcat(undistorted_img1, undistorted_img2, tracking_stereo);

            }
            tracking_video.write(tracking_stereo);
            imshow("img1", undistorted_img1);
            imshow("img2", undistorted_img2);
#endif
          
             
        } else {
            //Predict
            tracking_node.predict();


            //Project 3D point into images           
            tracking_node.computeProjectedPoints(K_cam1, K_cam2, R, T);

#ifdef SHOW_IMAGES
            //Show predictions and detections
            for (int i = 0; i < detections_cam1.size(); i++) {
                rectangle(undistorted_img1, detections_cam1[i]->getBox(), Scalar(0, 255, 0), 4, 1);
                circle(undistorted_img1, detections_cam1[i]->getCenter(), 10, Scalar(0, 0, 255), -1);
                putText(undistorted_img1, to_string(i), detections_cam1[i]->getOrigin() - Point2f(0,10), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);

            }

            for (int i = 0; i < detections_cam2.size(); i++) {
                rectangle(undistorted_img2, detections_cam2[i]->getBox(), Scalar(0, 255, 0), 4, 1);
                circle(undistorted_img2, detections_cam2[i]->getCenter(), 10, Scalar(0, 0, 255), -1);
                putText(undistorted_img2, to_string(i), detections_cam2[i]->getOrigin() - Point2f(0,10), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);

            }

            for (int i = 0; i < tracking_node.getTrackers().size(); i++) {
                cv::circle(undistorted_img1, tracking_node.getTrackers()[i]->getReprojectedPointCam1(), 10, tracking_node.getTrackers()[i]->getColor(), -1, cv::LINE_AA);
                cv::circle(undistorted_img2, tracking_node.getTrackers()[i]->getReprojectedPointCam2(), 10, tracking_node.getTrackers()[i]->getColor(), -1, cv::LINE_AA);
            }

            imshow("img1", undistorted_img1);
            imshow("img2", undistorted_img2);
#endif
          

            //Assignment between YOLO detection and projected points.

            vector<Detection*> detections_not_assigned_cam1, detections_not_assigned_cam2;
            
            tracking_node.hungarianTracking(detections_cam1, detections_cam2,
                    detections_not_assigned_cam1, detections_not_assigned_cam2);


            if (detections_not_assigned_cam1.size() > 0 && detections_not_assigned_cam2.size() > 0) {
                //Assignment of not tracked boxes              
                vector<int> assignment;
                vector<vector<double> > cost_matrix;
                stereo_matching(detections_not_assigned_cam1, detections_not_assigned_cam2, F, assignment, cost_matrix);

               
                for (unsigned int x = 0; x < assignment.size(); x++) {
                    //cout << "**" << endl;
                    //std::cout << "Assignment: box_cam1 " << x << ", box_cam2 : " << assignment[x] << endl;

                    if (assignment[x] == -1)
                        continue;

                    if (cost_matrix[x][assignment[x]] > 120) {
                        //cout << "Bad assigment -> cost:  " << cost_matrix[x][assignment[x]] << endl;
                        continue;
                    }

                    Point2f stereo_dist = detections_not_assigned_cam2[assignment[x]]->getCenter() - detections_not_assigned_cam1[x]->getCenter();
                    double stereo_dist_norm = sqrt(pow(stereo_dist.x, 2) + pow(stereo_dist.y, 2));

                    double area_ratio = (detections_not_assigned_cam2[assignment[x]]->getHeight() * detections_not_assigned_cam2[assignment[x]]->getWidth()) / (detections_not_assigned_cam1[x]->getHeight() * detections_not_assigned_cam1[x]->getWidth());
                    //cout << "height ratio " << area_ratio << " " << 1. / area_ratio << endl;
                    if (stereo_dist_norm > 850 || area_ratio < 0.6 || 1. / area_ratio < 0.6) {

                        //cout << "Bad assignment " << stereo_dist_norm << endl;
                        continue;
                    }

                    TrackerClass* tracker = new TrackerClass(detections_not_assigned_cam1[x]->getBox(), detections_not_assigned_cam2[assignment[x]]->getBox());
                    tracking_node.add(tracker);
                   
                }

                if(tracking_node.getTrackers().size() > 0)
                {
                 
                    tracking_node.compute3DPoints(K_cam1, K_cam2, R1, R2, P1, P2);
                }    
                             
            }
            
            if(tracking_node.getTrackers().size() > 0)
            {
                        //compute 3D points from assignments
                tracking_node.compute3DPoints(K_cam1, K_cam2, R1, R2, P1, P2);

                tracking_node.deleteNotUsedTrackers();

                tracking_node.correct();
            }

#ifdef SHOW_IMAGES

            for (int i = 0; i < tracking_node.getTrackers().size(); i++) {
                rectangle(undistorted_img1, tracking_node.getTrackers()[i]->getBoxCam1(), tracking_node.getTrackers()[i]->getColor(), 4, 1);
                putText(undistorted_img1, to_string(tracking_node.getTrackers()[i]->getId()),
                        Point(tracking_node.getTrackers()[i]->getBoxCam1().x, tracking_node.getTrackers()[i]->getBoxCam1().y - 10),
                        FONT_HERSHEY_SIMPLEX, 1, tracking_node.getTrackers()[i]->getColor(), 2);

                rectangle(undistorted_img2, tracking_node.getTrackers()[i]->getBoxCam2(), tracking_node.getTrackers()[i]->getColor(), 4, 1);
                putText(undistorted_img2, to_string(tracking_node.getTrackers()[i]->getId()),
                        Point(tracking_node.getTrackers()[i]->getBoxCam2().x, tracking_node.getTrackers()[i]->getBoxCam2().y - 10),
                        FONT_HERSHEY_SIMPLEX, 1, tracking_node.getTrackers()[i]->getColor(), 2);

            }

            imshow("img1", undistorted_img1);
            imshow("img2", undistorted_img2);

        
            Mat concat;
            hconcat(undistorted_img1, undistorted_img2, concat);
            tracking_video.write(concat);
#endif


            }
        }else{
    
            for(int i=0; i < tracking_node.getTrackers().size(); i++){
                tracking_node.addNotUsed(tracking_node.getTrackers()[i]->getId());
            }
             tracking_node.deleteNotUsedTrackers();
        
        }
        state = PUBLISH;
        

       
       break;
    }
   
    case PUBLISH:{
      //revisar añadido varias cosas juan
      printf("state= PUBLISH\n");

        worker_joints_msg.header.frame_id = "map";
        newobstacles_msg.header.frame_id = "map";
        worker_joints_msg.poses.resize(tracking_node.getTrackers().size());
        newobstacles_msg.x.resize(tracking_node.getTrackers().size());
        newobstacles_msg.y.resize(tracking_node.getTrackers().size());
        for(int i=0; i<tracking_node.getTrackers().size(); i++){
            
            cv::Mat point = (cv::Mat_<float>(3,1) << tracking_node.getTrackers()[i]->getPosition().x, tracking_node.getTrackers()[i]->getPosition().y, tracking_node.getTrackers()[i]->getPosition().z);
            point = Rcam_map * point + tcam_map;
            point /= 1000;
            
            worker_joints_msg.poses[i].position.x = point.at<float>(0,0);
            worker_joints_msg.poses[i].position.y = point.at<float>(1,0);
            worker_joints_msg.poses[i].position.z = point.at<float>(2,0);
            
            newobstacles_msg.x[i] = point.at<float>(0,0);
            newobstacles_msg.y[i] = point.at<float>(1,0);
        }
        pub_worker_joints.publish(worker_joints_msg);
        pub_worker_obstacles.publish(newobstacles_msg);
       
      state = READING;
       break;
    }
      
   }
 }
int main(int argc, char **argv)
{
  
  
    ros::init(argc,argv,"tracking_stereo", ros::init_options::NoSigintHandler);
    nh = new ros::NodeHandle();
    pub_worker_joints=nh->advertise<geometry_msgs::PoseArray>("worker_joints",1);
    //juan
    pub_worker_obstacles=nh->advertise<mapupdates::NewObstacles>("worker/newObstacles",1);
    //end juan
    
    signal(SIGINT, shutdown_tracker);
    signal(SIGTERM, shutdown_tracker);
         
    exp_t1 = 30.;
    exp_t2 = 30.;
    fps = 13;
    cam1_id = 1;
    cam2_id = 2;
    state = STOP;


    install_params();
 
    //Load calibration params
    cv::FileStorage fs;
    fs.open(cam_calibration_file, FileStorage::READ);
   
    fs["K_cam1"] >> K_cam1;
    fs["dist_coeffs_cam1"] >> dist_coeffs_cam1;
    fs["img_width_cam1"] >> img_width_cam1;
    fs["img_height_cam1"] >> img_height_cam1;
    fs["K_cam2"] >> K_cam2;
    fs["dist_coeffs_cam2"] >> dist_coeffs_cam2;
    fs["img_width_cam2"] >> img_width_cam2;
    fs["img_height_cam2"] >> img_height_cam2;
    fs["R"] >> R;
    fs["T"] >> T;
    fs["F"] >> F;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    
    fs.release();

    cv::FileStorage fs_transf;
    fs_transf.open(cam_map_calibration_file, cv::FileStorage::READ);
    fs_transf["Rcam_map"] >> Rcam_map;
    fs_transf["tcam_map"] >> tcam_map;

    fs_transf.release();
    
    
    // Load names of classes
    ifstream ifs(yolo_classes_file.c_str());
    string line;
    while (getline(ifs, line))
        classes.push_back(line);


    // Load the network
    net_yolo = readNetFromDarknet(yolo_model_file, yolo_weights_file);
    net_yolo.setPreferableBackend(DNN_BACKEND_CUDA);
    net_yolo.setPreferableTarget(DNN_TARGET_CUDA);
    
    
    tracking_video.open(video_path, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps,
                        cv::Size(img_width_cam1 * 2, img_height_cam1), 1);
     
#ifdef SHOW_IMAGES
    namedWindow("img1", WINDOW_NORMAL);
    resizeWindow("img1", 2048 / 2, 1088 / 2);
    namedWindow("img2", WINDOW_NORMAL);
    resizeWindow("img2", 2048 / 2, 1088 / 2);

    cv::startWindowThread();
#endif


    initUndistortRectifyMap(K_cam1, dist_coeffs_cam1, Mat(), K_cam1, Size(img_width_cam1, img_height_cam1), CV_32F, map1_cam1, map2_cam1);
  
    initUndistortRectifyMap(K_cam2, dist_coeffs_cam2, Mat(), K_cam2, Size(img_width_cam2, img_height_cam2), CV_32F, map1_cam2, map2_cam2);
     
    CameraIDS::getCamerasInfo();
    
    cam1.setCameraId(cam1_id);
    cam2.setCameraId(cam2_id);
     
    ros::Rate loop_rate(100);
    while(!stop){	 

      run();	
      ros::spinOnce();
      loop_rate.sleep();  

    }
#ifdef SHOW_IMAGES
    cv::destroyWindow("img1");
    cv::destroyWindow("img2");
#endif
   
    cam1.exitCamera();
    cam2.exitCamera();
    tracking_video.release();
     

}
