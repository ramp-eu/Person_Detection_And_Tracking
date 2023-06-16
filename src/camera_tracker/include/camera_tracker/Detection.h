/* 
 * File:   Detection.h
 * Author: adrianacostas
 *
 * Created on September 24, 2021, 12:39 PM
 */
#include <opencv2/opencv.hpp>

#include <math.h>

#ifndef DETECTION_H
#define	DETECTION_H
using namespace std;
using namespace cv;
class Detection {
public:
  Detection(Rect2d box);
    Detection(Rect2d box, int tracker_id, Scalar color);
    Detection(const Detection& orig);
    virtual ~Detection();

    Rect2d getBox(){
        return this->box;
    }
    
    void setBox(Rect2d box){
        this->box = box;
    }
    
    void setJoints(vector<Point2f> points){
        this->joints = points;
    }
    
    vector<Point2f> getJoints(){
        return this->joints;
    }
    Point2f getCenter(){
        Point2f center;
        center.x = this->box.x + this->box.width / 2.;
        center.y = this->box.y + this->box.height / 2.;
        return center;
    }
    
int getTrackerId(){
return this->tracker_id;
}
    Point2f getOrigin(){
       
        Point2f corner (this->box.tl());
        return corner;
    }
    Scalar getColor(){
      return this->color;
    }
    double getHeight(){
        return this->box.height;
    }
        double getWidth(){
        return this->box.width;
    }
private:
    static unsigned int idGenerator;
    Rect2d box;
    vector<Point2f> joints;
    int tracker_id;
    Scalar color;
};

#endif	/* DETECTION_H */

