/* 
 * File:   Tracker.h
 * Author: adrianacostas
 *
 * Created on August 24, 2021, 10:51 AM
 */

#include <opencv2/opencv.hpp>

#include <math.h>
#ifndef TRACKERCLASS_H
#define	TRACKERCLASS_H

using namespace std;
using namespace cv;

class TrackerClass {
public:
    
     bool exists_previous_3D_position;
     
    TrackerClass();
    TrackerClass(Point3f position3D, Rect2d box_cam1, Rect2d box_cam2);
     TrackerClass(Rect2d box_cam1, Rect2d box_cam2);
    TrackerClass(const TrackerClass& orig);
    virtual ~TrackerClass();

    unsigned int getId() {
        return this->id;
    }

    Point3f getPosition() {
        return this->position3D;
    }

    Scalar getColor() {
        return this->color;
    }

    KalmanFilter* getKalmanFilter() {
        return this->KF;
    }

    Rect2d getBoxCam1() {
        return this->box_cam1;
    }

    Point2f getBoxCam1Center() {
        Point2f center;
        center.x = this->box_cam1.x + this->box_cam1.width / 2.;
        center.y = this->box_cam1.y + this->box_cam1.height / 2.;
        return center;
    }

    Rect2d getBoxCam2() {
        return this->box_cam2;
    }

    Point2f getBoxCam2Center() {
        Point2f center;
        center.x = this->box_cam2.x + this->box_cam2.width / 2.;
        center.y = this->box_cam2.y + this->box_cam2.height / 2.;
        return center;
    }
    
     Rect2d getAssignedBoxCam1() {
        return this->assigned_box_cam1;
    }



    Rect2d getAssignedBoxCam2() {
        return this->assigned_box_cam2;
    }


    Point2f getReprojectedPointCam1() {
        return this->reprojected_pt_cam1;
    }

    Point2f getReprojectedPointCam2() {
        return this->reprojected_pt_cam2;
    }

    bool getAssignedCam1() {
        return this->assigned_cam1;
    }

    bool getAssignedCam2() {
        return this->assigned_cam2;
    }


    void setPosition(Point3f position3D) {
        this->position3D = position3D;
    }

    void setBoxCam1(Rect2d box) {
        this->box_cam1 = box;
    }

    void setBoxCam2(Rect2d box) {
        this->box_cam2 = box;
    }

    void setAssignedBoxCam1(Rect2d box) {
        this->assigned_box_cam1 = box;
    }

    void setAssignedBoxCam2(Rect2d box) {
        this->assigned_box_cam2 = box;
    }

    void setReprojectedPtCam1(Point2f pt) {
        this->reprojected_pt_cam1 = pt;
    }

    void setReprojectedPtCam2(Point2f pt) {
        this->reprojected_pt_cam2 = pt;
    }

    void setAssignedCam1(bool a) {
        this->assigned_cam1 = a;
    }

    void setAssignedCam2(bool a) {
        this->assigned_cam2 = a;
    }



    void initializeKalmanFilter();

   

private:
    static unsigned int idGenerator;

    unsigned int id;
    Point3f position3D;
    Rect2d box_cam1;
    Rect2d box_cam2;
    Rect2d assigned_box_cam1;
    Rect2d assigned_box_cam2;
    Point2f reprojected_pt_cam1;
    Point2f reprojected_pt_cam2;
    KalmanFilter* KF;
    Scalar color;
    bool assigned_cam1;
    bool assigned_cam2;
   
   

};

#endif	/* TRACKER_H */

