/* 
 * File:   VectorOfTrackers.h
 * Author: adrianacostas
 *
 * Created on August 24, 2021, 1:24 PM
 */
#include "TrackerClass.h"
#include "Hungarian.h"
#include "Detection.h"
#include <unordered_map>


#ifndef VECTOROFTRACKERS_H
#define	VECTOROFTRACKERS_H

using namespace std;
using namespace cv;

class Tracking {
public:
    Tracking();
    Tracking(const Tracking& orig);
    virtual ~Tracking();

    vector<TrackerClass*> getTrackers() {
        return this->trackers;
    }

    void setTrackers(vector<TrackerClass*> trackers) {

        this->trackers = trackers;
    }

    void getTrackersPositions(vector<Point2f> &boxsPosition_cam1, vector<Point2f> &boxsPosition_cam2) {
        for (int i = 0; i < this->trackers.size(); i++) {
            boxsPosition_cam1.push_back(this->getBoxCenter(trackers[i]->getBoxCam1()));
            boxsPosition_cam2.push_back(this->getBoxCenter(trackers[i]->getBoxCam2()));
        }
    }

    Point2f getBoxCenter(Rect2d box) {
        Point2f center;
        center.x = box.x + box.width / 2.;
        center.y = box.y + box.height / 2.;
        return center;
    }

    float distancePointLine(cv::Point2f p, cv::Point3f line) {
        //Line is given as a*x + b*y + c = 0
        //d = (a*p.x + b*p.y+c)/sqrt(a²+b²)
        return std::fabs(line.x * p.x + line.y * p.y + line.z)
                / std::sqrt(line.x * line.x + line.y * line.y);
    }

    void add(TrackerClass* tracker) {
        this->trackers.push_back(tracker);
        this->trackers_not_used_cam1.insert({tracker->getId(), 0});
        this->trackers_not_used_cam2.insert({tracker->getId(), 0});
    }

    void addNotUsed(unsigned int id){
    	trackers_not_used_cam1[id] += 1;
	trackers_not_used_cam2[id] += 1;
    }
    void deleteNotUsedTrackers();

    void hungarianTracking(vector<Detection*> &detections_cam1, vector<Detection*> &detections_cam2,
    vector<Detection*> &detections_not_assigned_cam1, vector<Detection*> &detections_not_assigned_cam2);

    void predict();

    void correct();

    void computeProjectedPoints(Mat K_cam1, Mat K_cam2, Mat R, Mat T);

    float compute2DDistance(Point2f p1, Point2f p2) {

        float dist = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
        return dist;
    }

    float compute3DDistance(Point3f p1, Point3f p2) {

        float dist = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
        return dist;
    }

    void compute3DPoints(Mat &K_cam1, Mat &K_cam2, Mat &R1, Mat &R2, Mat &P1, Mat &P2, bool verification = false);

    float distBetweenBoxes(Rect2d box1, Rect2d box2) {
        Point2f center1, center2;
        center1 = this->getBoxCenter(box1);
        center2 = this->getBoxCenter(box2);

        Point2f v = center2 - center1;
        float dist = sqrt(pow(v.x, 2) + pow(v.y, 2));
        return dist;
    }

   
private:
    vector<TrackerClass*> trackers;
    //vector<unsigned int> trackers_not_used;
    unordered_map<unsigned int, int> trackers_not_used_cam1;
    unordered_map<unsigned int, int> trackers_not_used_cam2;
};

#endif	/* VECTOROFTRACKERS_H */

