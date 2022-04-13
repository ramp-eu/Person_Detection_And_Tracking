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
#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include "Hungarian.h"
#include "TrackerClass.h"
#include "Tracking.h"
#include "Detection.h"

#define STOP 0
#define INIT 1
#define READING 2
#define DETECT 3
#define TRACK 4
#define PUBLISH 5

using namespace cv;
using namespace std;
using namespace cv::dnn;

ros::NodeHandle* nh;
ros::Publisher pub_worker_joints;
geometry_msgs::PoseArray worker_joints_msg;

////////////////////////////////////////////////////////
/*++++++++++++++++++++ YOLO v3 +++++++++++++++++++++++*/
////////////////////////////////////////////////////////
Net net_yolo;
float confThreshold = 0.7;         // Confidence threshold 0.7
float nmsThreshold = 0.7;          // Non-maximum suppression threshold 0.45
int inpWidth = 416;                // Width of network's input image
int inpHeight = 416 * 1088 / 2048; // Height of network's input image
vector<string> classes;


volatile int stop = 0;
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
Tracking tracking_node;

// Read video
VideoCapture video_1("/home/acl/NetBeansProjects/Hungarian_Stereo_Matching/data/video_1_prueba1.avi");
VideoCapture video_2("/home/acl/NetBeansProjects/Hungarian_Stereo_Matching/data/video_2_prueba1.avi");

// Write video
cv::VideoWriter tracking_video, detection_video;

double get_current_time(void) {
    struct timespec time;
    clock_gettime(CLOCK_REALTIME, &time);
    return (time.tv_sec + time.tv_nsec / 1.0e09);
}

void shutdown_tracker(int error) {

    stop = 1;

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
        cout << "x " << boxsPosition_cam1[i].x << ", y: " << boxsPosition_cam1[i].y << endl;
    }
    for (int i = 0; i < detections_cam2.size(); i++) {
        boxsPosition_cam2.push_back(detections_cam2[i]->getCenter());
        cout << "x " << boxsPosition_cam2[i].x << ", y: " << boxsPosition_cam2[i].y << endl;
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

        waitKey(0);

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

        waitKey(0);
    }


    vector<double> cost_per_line(detections_cam2.size(), 1.);
    cost_matrix.resize(detections_cam1.size(), cost_per_line);

    //    double max_dist = 0;
    //
    //    for (size_t i = 0; i < lines_img2.size(); i++) {
    //        for (int j = 0; j < boxsPosition_cam2.size(); j++) {
    //            double dist = distancePointLine(boxsPosition_cam2[j], lines_img2[i]);
    //            if (max_dist < dist)
    //                max_dist = dist;
    //
    //        }
    //    }



    for (size_t i = 0; i < lines_img2.size(); i++) {
        // vector<double> cost_per_line;
        for (int j = 0; j < boxsPosition_cam2.size(); j++) {

            double dist = distancePointLine(boxsPosition_cam2[j], lines_img2[i]);

            double score = dist;
            cout << "Box_cam1 " << i << " box_cam2 " << j << " score: " << score << endl;
            //cost_per_line.push_back(score);

            cost_matrix[i][j] = score;
        }

        //cost_matrix.push_back(cost_per_line);
    }

    //    max_dist = 0;
    //    for (size_t i = 0; i < lines_img1.size(); i++) {
    //        for (int j = 0; j < boxsPosition_cam1.size(); j++) {
    //            double dist = distancePointLine(boxsPosition_cam1[j], lines_img1[i]);
    //            if (max_dist < dist)
    //                max_dist = dist;
    //
    //        }
    //    }

    for (size_t i = 0; i < lines_img1.size(); i++) {

        for (int j = 0; j < boxsPosition_cam1.size(); j++) {

            double dist = distancePointLine(boxsPosition_cam1[j], lines_img1[i]);

            double score = dist;
            cout << "Box_cam1 " << j << " box_cam2 " << i << " score: " << score << endl;


            cost_matrix[j][i] += score;
        }


    }



    HungarianAlgorithm HungAlgo;
    double cost = HungAlgo.Solve(cost_matrix, assignment);

    cout << "cost: " << cost << endl;



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
     state = READING;
      
      break;
      }
    case READING:{
      printf("state= READING\n");		

      if (video_1.read(frame_1) && video_2.read(frame_2)) {


        undistort(frame_1, undistorted_img1, K_cam1, dist_coeffs_cam1);
        undistort(frame_2, undistorted_img2, K_cam2, dist_coeffs_cam2);

       
	state = DETECT;

      }else{
	 printf("Video is finished\n");		
	stop = 1;
      }
           
      
     
      break;
      }
    case DETECT:{
      
	printf("state= DETECT\n");
        undistorted_img1.copyTo(aux_1);
        undistorted_img2.copyTo(aux_2);

       detections_cam1.clear();
       detections_cam2.clear();


        peopleDetection(net_yolo, undistorted_img1, inpWidth, inpHeight, detections_cam1);
        peopleDetection(net_yolo, undistorted_img2, inpWidth, inpHeight, detections_cam2);
      
	if(detections_cam1.size() > 0 && detections_cam2.size() > 0){
	  state = TRACK;
	}else{
	  state = READING;
	}
	

       
       break;
    }
    case TRACK:{
      
      printf("state= TRACK\n");

       if (tracking_node.getTrackers().size() == 0) {
            vector<int> assignment;
            vector<vector<double> > cost_matrix;

            stereo_matching(detections_cam1, detections_cam2, F, assignment, cost_matrix);

   

            for (unsigned int x = 0; x < assignment.size(); x++) {
                cout << "**" << endl;
                std::cout << "Assignment: box_cam1 " << x << ", box_cam2 : " << assignment[x] << endl;

                if (assignment[x] == -1)
                    continue;

                if (cost_matrix[x][assignment[x]] > 50) {
                    cout << "Bad assigment -> cost:  " << cost_matrix[x][assignment[x]] << endl;
                    continue;
                }

                Point2f stereo_dist = detections_cam2[assignment[x]]->getCenter() - detections_cam1[x]->getCenter();
                double stereo_dist_norm = sqrt(pow(stereo_dist.x, 2) + pow(stereo_dist.y, 2));

               
                double area_ratio = (detections_cam2[assignment[x]]->getHeight() * detections_cam2[assignment[x]]->getWidth()) / (detections_cam1[x]->getHeight() * detections_cam1[x]->getWidth());
                cout << "height ratio " << area_ratio << " " << 1. / area_ratio << endl;
                if (stereo_dist_norm > 800 || area_ratio < 0.8 || 1. / area_ratio < 0.8) {

                    cout << "Bad assignment " << stereo_dist_norm << endl;
                    continue;
                }

                // assigned_detections_cam1.push_back(detections_cam1[x]);
                // assigned_detections_cam2.push_back(detections_cam2[assignment[x]]);

                TrackerClass* tracker = new TrackerClass(detections_cam1[x]->getBox(), detections_cam2[assignment[x]]->getBox());
                tracking_node.add(tracker);
            }



             if (tracking_node.getTrackers().size() > 0){

            tracking_node.compute3DPoints(K_cam1, K_cam2, R1, R2, P1, P2);

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
            //tracking_video.write(tracking_stereo);
            imshow("img1", undistorted_img1);
            imshow("img2", undistorted_img2);
             //waitKey(0);
             }

           
        } else {
            //Predict
            tracking_node.predict();


            //Project 3D point into images           
            tracking_node.computeProjectedPoints(K_cam1, K_cam2, R, T);

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

           // waitKey(0);

            //Assignment between YOLO detection and projected points.

            vector<Detection*> detections_not_assigned_cam1, detections_not_assigned_cam2;
            tracking_node.hungarianTracking(detections_cam1, detections_cam2,
                    detections_not_assigned_cam1, detections_not_assigned_cam2);
/*
            compute 3D points from assignments
            tracking_node.compute3DPoints(K_cam1, K_cam2, R1, R2, P1, P2, true);


            tracking_node.deleteNotUsedTrackers();

            Update trackers
            tracking_node.correct();*/

            if (detections_not_assigned_cam1.size() > 0 && detections_not_assigned_cam2.size() > 0) {
                //Assignment of not tracked boxes              
                vector<int> assignment;
                vector<vector<double> > cost_matrix;
                stereo_matching(detections_not_assigned_cam1, detections_not_assigned_cam2, F, assignment, cost_matrix);

               

                for (unsigned int x = 0; x < assignment.size(); x++) {
                    cout << "**" << endl;
                    std::cout << "Assignment: box_cam1 " << x << ", box_cam2 : " << assignment[x] << endl;

                    if (assignment[x] == -1)
                        continue;

                    if (cost_matrix[x][assignment[x]] > 120) {
                        cout << "Bad assigment -> cost:  " << cost_matrix[x][assignment[x]] << endl;
                        continue;
                    }

                    Point2f stereo_dist = detections_not_assigned_cam2[assignment[x]]->getCenter() - detections_not_assigned_cam1[x]->getCenter();
                    double stereo_dist_norm = sqrt(pow(stereo_dist.x, 2) + pow(stereo_dist.y, 2));

                    double area_ratio = (detections_not_assigned_cam2[assignment[x]]->getHeight() * detections_not_assigned_cam2[assignment[x]]->getWidth()) / (detections_not_assigned_cam1[x]->getHeight() * detections_not_assigned_cam1[x]->getWidth());
                    cout << "height ratio " << area_ratio << " " << 1. / area_ratio << endl;
                    if (stereo_dist_norm > 850 || area_ratio < 0.6 || 1. / area_ratio < 0.6) {

                        cout << "Bad assignment " << stereo_dist_norm << endl;
                        continue;
                    }


                    TrackerClass* tracker = new TrackerClass(detections_not_assigned_cam1[x]->getBox(), detections_not_assigned_cam2[assignment[x]]->getBox());
                    tracking_node.add(tracker);

                   
                }

                
         
            tracking_node.compute3DPoints(K_cam1, K_cam2, R1, R2, P1, P2);

           
            }






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

            //waitKey(1);


            Mat concat;
            hconcat(undistorted_img1, undistorted_img2, concat);
            tracking_video.write(concat);


            //Create new trackers if needed


        }
        
        if (tracking_node.getTrackers().size() == 0){
            state = PUBLISH;
        }else{
            state = READING;
        }
       
       break;
    }
   
    case PUBLISH:{
      
      printf("state= PUBLISH\n");

      worker_joints_msg.header.frame_id = "map";
        worker_joints_msg.poses.resize(tracking_node.getTrackers().size());
        for(int i=0; i<tracking_node.getTrackers().size(); i++){
            
            cv::Mat point = (cv::Mat_<float>(3,1) << tracking_node.getTrackers()[i]->getPosition().x, tracking_node.getTrackers()[i]->getPosition().y, tracking_node.getTrackers()[i]->getPosition().z);
            point = Rcam_map * point + tcam_map;
            point /= 1000;
            
            worker_joints_msg.poses[i].position.x = point.at<float>(0,0);
            worker_joints_msg.poses[i].position.y = point.at<float>(1,0);
            worker_joints_msg.poses[i].position.z = point.at<float>(2,0);
        }
        pub_worker_joints.publish(worker_joints_msg);
       
        state = READING;
       break;
    }
      
   }
 }



int main(int argc, char **argv)
{
    ros::init(argc,argv,"read_stereo");
    nh=new ros::NodeHandle();
    pub_worker_joints=nh->advertise<geometry_msgs::PoseArray>("worker_joints",1);

    // Load names of classes
    string classesFile = "/home/arta/catkin_ws/src/read_stereo/src/coco.names";

    //string classesFile = "/home/acl/Documents/DATA/Sistema-seguridad/Seguimiento de personas/YOLOv4/coco.names";
    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line))
        classes.push_back(line);

    // Give the configuration and weight files for the model
    cv::String modelConfiguration = "/home/arta/catkin_ws/src/read_stereo/src/yolov3.cfg";
    cv::String modelWeights = "/home/arta/catkin_ws/src/read_stereo/src/yolov3.weights";
    // cv::String modelConfiguration = "/home/acl/Documents/DATA/Sistema-seguridad/Seguimiento de personas/YOLOv4/yolov4.cfg";
    // cv::String modelWeights = "/home/acl/Documents/DATA/Sistema-seguridad/Seguimiento de personas/YOLOv4/yolov4.weights";

    // Load the network
    net_yolo = readNetFromDarknet(modelConfiguration, modelWeights);
    net_yolo.setPreferableBackend(DNN_BACKEND_CUDA);
    net_yolo.setPreferableTarget(DNN_TARGET_CUDA);

    double fps = video_1.get(CAP_PROP_FPS);

    tracking_video.open("/home/arta/catkin_ws/src/tracking/result/tracking.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps,
                        cv::Size(2048 * 2, 1088), 1);

    detection_video.open("/home/arta/catkin_ws/src/tracking/result/detection.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps,
                         cv::Size(2048, 1088), 1);

    if (!tracking_video.isOpened() || !detection_video.isOpened())
    {
        cout << "Could not open the output video for write" << endl;
        return -1;
    }
    // Exit if video is not opened
    if (!video_1.isOpened())
    {
        cout << "Could not read video 1 file" << endl;
        return 1;
    }

    if (!video_2.isOpened())
    {
        cout << "Could not read video 2 file" << endl;
        return 1;
    }

    string filename = "/home/arta/catkin_ws/src/visual_tracking/data/config_data_cam1&cam2_v3.xml";
    cv::FileStorage fs;

    fs.open(filename, FileStorage::READ);
   

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
    fs_transf.open("/home/arta/catkin_ws/src/read_stereo/data/H_map_cam1.xml", cv::FileStorage::READ);
    fs_transf["Rcam_map"] >> Rcam_map;
    fs_transf["tcam_map"] >> tcam_map;

    fs_transf.release();

    
    state = STOP;
    signal(SIGINT, shutdown_tracker);
    signal(SIGTERM, shutdown_tracker);

    //just if we need start the video in other time
    int contador = 0;
    Mat ax1, ax2;
    while (contador < 420)
    {
        contador++;
        video_1.read(ax1);
        video_2.read(ax2);
    }

    // Read first frame
   
    //bool ok = video.read(frame);

    namedWindow("img1", WINDOW_NORMAL);
    resizeWindow("img1", 2048 / 2, 1088 / 2);
    namedWindow("img2", WINDOW_NORMAL);
    resizeWindow("img2", 2048 / 2, 1088 / 2);

      cv::startWindowThread();

    
    ros::Rate loop_rate(20);
    while(!stop){	 

      run();	
      ros::spinOnce();
      loop_rate.sleep();  

    }
    
        cv::destroyWindow("img1");
      cv::destroyWindow("img2");
    tracking_video.release();
cout << "Closing video" << endl;
}
