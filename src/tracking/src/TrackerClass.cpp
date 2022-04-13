/* 
 * File:   Tracker.cpp
 * Author: adrianacostas
 * 
 * Created on August 24, 2021, 10:51 AM
 */

#include "Tracking.h"
unsigned int TrackerClass::idGenerator = 0;

TrackerClass::TrackerClass() {
    cout << "IdGenerator: " << TrackerClass::idGenerator << endl;
    TrackerClass::id = TrackerClass::idGenerator++;

    this->color = Scalar(rand() % 255, rand() % 255, rand() % 255);
}

TrackerClass::TrackerClass(Point3f position3D, Rect2d box_cam1, Rect2d box_cam2) {
    
    this->position3D = position3D;
    
  cout << "IdGenerator: " << TrackerClass::idGenerator << endl;
    TrackerClass::id = TrackerClass::idGenerator++;

    this->assigned_cam1 = false;
    this->assigned_cam2 = false;
    this->box_cam1 = box_cam1;
    this->box_cam2 = box_cam2;
    this->assigned_box_cam1 = box_cam1;
    this->assigned_box_cam2 = box_cam2;
    this->exists_previous_3D_position = true;


    
    this->KF = new cv::KalmanFilter(6, 3, 0);

    this->KF->transitionMatrix = (Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
            0, 1, 0, 0, 1, 0,
            0, 0, 1, 0, 0, 1,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1);

    this->KF->statePre.at<float>(0) = position3D.x;
    this->KF->statePre.at<float>(1) = position3D.y;
    this->KF->statePre.at<float>(2) = position3D.z;
    this->KF->statePre.at<float>(3) = 0;
    this->KF->statePre.at<float>(4) = 0;
    this->KF->statePre.at<float>(5) = 0;

    this->KF->statePost.at<float>(0) = position3D.x;
    this->KF->statePost.at<float>(1) = position3D.y;
    this->KF->statePost.at<float>(2) = position3D.z;
    this->KF->statePost.at<float>(3) = 0;
    this->KF->statePost.at<float>(4) = 0;
    this->KF->statePost.at<float>(5) = 0;



    setIdentity(KF->measurementMatrix);
    setIdentity(KF->processNoiseCov, cv::Scalar::all(2));
    setIdentity(KF->measurementNoiseCov, cv::Scalar::all(10));

    setIdentity(KF->errorCovPost, cv::Scalar::all(2));
    setIdentity(KF->errorCovPre, cv::Scalar::all(2));

    this->color = Scalar(rand() % 255, rand() % 255, rand() % 255);

    // cout << "Measurement mat: " << KF->measurementMatrix <<endl;
    //cout << "Measurement noise cov: " <<KF->measurementNoiseCov << endl;
    //cout << "process noise cov: " << KF->processNoiseCov << endl;

   
}

TrackerClass::TrackerClass(Rect2d box_cam1, Rect2d box_cam2) {
    cout << "IdGenerator: " << TrackerClass::idGenerator << endl;
    TrackerClass::id = TrackerClass::idGenerator++;

    this->assigned_cam1 = false;
    this->assigned_cam2 = false;
    this->box_cam1 = box_cam1;
    this->box_cam2 = box_cam2;
    this->assigned_box_cam1 = box_cam1;
    this->assigned_box_cam2 = box_cam2;
    this->exists_previous_3D_position = false;
   
 

    this->color = Scalar(rand() % 255, rand() % 255, rand() % 255);

    // cout << "Measurement mat: " << KF->measurementMatrix <<endl;
    //cout << "Measurement noise cov: " <<KF->measurementNoiseCov << endl;
    //cout << "process noise cov: " << KF->processNoiseCov << endl;
}

TrackerClass::TrackerClass(const TrackerClass& orig) {
}

TrackerClass::~TrackerClass() {
}

void TrackerClass::initializeKalmanFilter(){
       this->KF = new cv::KalmanFilter(6, 3, 0);

    this->KF->transitionMatrix = (Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
            0, 1, 0, 0, 1, 0,
            0, 0, 1, 0, 0, 1,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1);

    this->KF->statePre.at<float>(0) = position3D.x;
    this->KF->statePre.at<float>(1) = position3D.y;
    this->KF->statePre.at<float>(2) = position3D.z;
    this->KF->statePre.at<float>(3) = 0;
    this->KF->statePre.at<float>(4) = 0;
    this->KF->statePre.at<float>(5) = 0;

    this->KF->statePost.at<float>(0) = position3D.x;
    this->KF->statePost.at<float>(1) = position3D.y;
    this->KF->statePost.at<float>(2) = position3D.z;
    this->KF->statePost.at<float>(3) = 0;
    this->KF->statePost.at<float>(4) = 0;
    this->KF->statePost.at<float>(5) = 0;



    setIdentity(KF->measurementMatrix);
    setIdentity(KF->processNoiseCov, cv::Scalar::all(2));
    setIdentity(KF->measurementNoiseCov, cv::Scalar::all(10));

    setIdentity(KF->errorCovPost, cv::Scalar::all(2));
    setIdentity(KF->errorCovPre, cv::Scalar::all(2));
}
