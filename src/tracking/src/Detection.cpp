/* 
 * File:   Detection.cpp
 * Author: adrianacostas
 * 
 * Created on September 24, 2021, 12:39 PM
 */

#include "Detection.h"
unsigned int Detection::idGenerator = 0;
Detection::Detection(Rect2d box) {
    this->box = box;
}

Detection::Detection(const Detection& orig) {
}

Detection::~Detection() {
}

