/* 
 * File:   Detection.cpp
 * Author: adrianacostas
 * 
 * Created on September 24, 2021, 12:39 PM
 */

#include <camera_tracker/Detection.h>
unsigned int Detection::idGenerator = 0;
Detection::Detection(Rect2d box) {
    this->box = box;

}
Detection::Detection(Rect2d box, int tracker_id, Scalar color) {
    this->box = box;
    this->tracker_id = tracker_id;
    this->color = color;
}

Detection::Detection(const Detection& orig) {
}

Detection::~Detection() {
}

