/* 
 * File:   VectorOfTrackers.cpp
 * Author: adrianacostas
 * 
 * Created on August 24, 2021, 1:24 PM
 */

#include <camera_tracker/Tracking.h>

Tracking::Tracking() {
}

Tracking::Tracking(const Tracking& orig) {
}

Tracking::~Tracking() {
}

void Tracking::deleteNotUsedTrackers() {
    vector<TrackerClass*> aux;

    for (int i = 0; i < this->trackers.size(); i++) {

        if (trackers_not_used_cam1[this->trackers[i]->getId()] > 6 || trackers_not_used_cam2[this->trackers[i]->getId()] > 6) { //If it is not used in more than 6 frames
            trackers_not_used_cam1.erase(this->trackers[i]->getId());
            trackers_not_used_cam2.erase(this->trackers[i]->getId());
            cout << "***********Removing tracker " << this->trackers[i]->getId() << endl;

        } else {
            aux.push_back(this->trackers[i]);
        }

    }

    this->trackers = aux;
}

void Tracking::hungarianTracking(vector<Detection*> &detections_cam1, vector<Detection*> &detections_cam2,
         vector<Detection*> &detections_not_assigned_cam1, vector<Detection*> &detections_not_assigned_cam2) {

    //For cam1

    HungarianAlgorithm HungAlgo;
    double cost = 0;

    
    vector<Detection*> inlier_detections_cam1;
    //Remove outlier before Assignment
    for (int i = 0; i < detections_cam1.size(); i++) {
        double min_dist = 700000;
        for (int j = 0; j < trackers.size(); j++) {
            double dist = compute2DDistance(detections_cam1[i]->getCenter(), trackers[j]->getReprojectedPointCam1());
            if (min_dist > dist)
                min_dist = dist;
        }

        if (min_dist < 100) {
            inlier_detections_cam1.push_back(detections_cam1[i]);
        } else {
            detections_not_assigned_cam1.push_back(detections_cam1[i]);
            cout << "*************************Cam1 Box " << i << " is an outlier: " << min_dist << endl;
        }
    }

    if (inlier_detections_cam1.size() > 0) {
        vector<double> cost_per_tracker(inlier_detections_cam1.size(), 500000.);
        vector<vector<double> > cost_matrix(trackers.size(), cost_per_tracker);
        for (size_t i = 0; i < trackers.size(); i++) {
            for (int j = 0; j < inlier_detections_cam1.size(); j++) {

                double score = compute2DDistance(inlier_detections_cam1[j]->getCenter(), trackers[i]->getReprojectedPointCam1());
                cout << "Box_cam1 " << j << " Reprojected pt cam1 tracker " << trackers[i]->getId() << " score: " << score << endl;
                cost_matrix[i][j] = score;
            }
        }


        vector<int> assignment;
        cost = HungAlgo.Solve(cost_matrix, assignment);

        cout << "cost: " << cost << endl;


        for (int x = 0; x < assignment.size(); x++) {
            std::cout << "Assignment: tracker " << trackers[x]->getId() << ", box_cam1 : " << assignment[x] << " cost: " << cost_matrix[x][assignment[x]] << endl;

            if (assignment[x] == -1) {
                cout << trackers[x]->getId() << endl;
                trackers[x]->setAssignedCam1(false);
                trackers_not_used_cam1[trackers[x]->getId()] += 1;

            }//        else if (cost_matrix[x][assignment[x]] > 40 * (trackers_not_used_cam1[trackers[x]->getId()] + 1)) {
                //            trackers_not_used_cam1[trackers[x]->getId()] += 1;
                //            trackers[x]->setAssignedCam1(false);
                //            cout << "Bad assignment " << endl;
                //        } 
            else {
                //trackers_not_used_cam1[trackers[x]->getId()] = 0;
                trackers[x]->setAssignedCam1(true);
                trackers[x]->setAssignedBoxCam1(inlier_detections_cam1[assignment[x]]->getBox());
            }



        }

        for (int i = 0; i < inlier_detections_cam1.size(); i++) {
            bool exists = false;
            for (int j = 0; j < assignment.size(); j++) {
                if (assignment[j] == i && cost_matrix[j][assignment[j]] < 100 * (trackers_not_used_cam1[trackers[j]->getId()] + 1)) {
                    exists = true;
                }
            }
            if (!exists) {
                detections_not_assigned_cam1.push_back(inlier_detections_cam1[i]);
            }

        }

    } else {
        for (int i = 0; i < trackers.size(); i++) {
            trackers_not_used_cam1[trackers[i]->getId()] += 1;
            trackers[i]->setAssignedCam1(false);
        }
    }
    //For cam2
  


    vector<Detection*> inlier_detections_cam2;
    //Remove outlier before Assignment
    for (int i = 0; i < detections_cam2.size(); i++) {
        double min_dist = 700000;
        for (int j = 0; j < trackers.size(); j++) {
            double dist = compute2DDistance(detections_cam2[i]->getCenter(), trackers[j]->getReprojectedPointCam2());
            if (min_dist > dist)
                min_dist = dist;
        }

        if (min_dist < 100) {
            inlier_detections_cam2.push_back(detections_cam2[i]);
        } else {
            cout << "************************Cam2 Box " << i << " is an outlier: " << min_dist << endl;
            detections_not_assigned_cam2.push_back(detections_cam2[i]);
        }
    }

    if (inlier_detections_cam2.size() > 0) {
          vector<double> cost_per_tracker_cam2(inlier_detections_cam2.size(), 50000.);
    vector<vector<double> > cost_matrix_cam2(trackers.size(), cost_per_tracker_cam2);
        vector<int> assignment_cam2;
        for (size_t i = 0; i < trackers.size(); i++) {
            // vector<double> cost_per_line;
            for (int j = 0; j < inlier_detections_cam2.size(); j++) {
                double score = compute2DDistance(inlier_detections_cam2[j]->getCenter(), trackers[i]->getReprojectedPointCam2());
                cout << "Box_cam2 " << j << " Reprojected pt cam2 tracker " << trackers[i]->getId() << " score: " << score << endl;
                cost_matrix_cam2[i][j] = score;
            }

            //cost_matrix.push_back(cost_per_line);
        }

        cost = HungAlgo.Solve(cost_matrix_cam2, assignment_cam2);

        cout << "cost: " << cost << endl;

        for (int x = 0; x < assignment_cam2.size(); x++) {
            std::cout << "Assignment: tracker " << trackers[x]->getId() << ", box_cam2 : " << assignment_cam2[x] << " cost: " << cost_matrix_cam2[x][assignment_cam2[x]] << endl;
            cout << assignment_cam2[x] << endl;
            if (assignment_cam2[x] == -1) {
                trackers_not_used_cam2[trackers[x]->getId()] += 1;
                trackers[x]->setAssignedCam2(false);
                // boxes_not_assigned_cam2.push_back(boxes_cam2[assignment_cam2[x]]);
            }//        else if (cost_matrix_cam2[x][assignment_cam2[x]] > 40 * (trackers_not_used_cam2[trackers[x]->getId()] + 1)) {
                //            trackers_not_used_cam2[trackers[x]->getId()] += 1;
                //            trackers[x]->setAssignedCam2(false);
                //            cout << "Bad assignment " << cost_matrix_cam2[x][assignment_cam2[x]] << endl;
                //            
                //        } 
            else {
                //trackers_not_used_cam2[trackers[x]->getId()] = 0;
                trackers[x]->setAssignedCam2(true);
                trackers[x]->setAssignedBoxCam2(inlier_detections_cam2[assignment_cam2[x]]->getBox());
            }



        }

        for (int i = 0; i < inlier_detections_cam2.size(); i++) {
            bool exists = false;
            for (int j = 0; j < assignment_cam2.size(); j++) {
                if (assignment_cam2[j] == i && cost_matrix_cam2[j][assignment_cam2[j]] < 100 * (trackers_not_used_cam2[trackers[j]->getId()] + 1)) {
                    exists = true;
                }
            }
            if (!exists) {
                detections_not_assigned_cam2.push_back(inlier_detections_cam2[i]);
            }

        }
    } else {
        for (int i = 0; i < trackers.size(); i++) {
            trackers_not_used_cam2[trackers[i]->getId()] += 1;
            trackers[i]->setAssignedCam2(false);
        }
    }





}

void Tracking::predict() {
    for (int i = 0; i < trackers.size(); i++) {

        //cout << trackers[i]->getKalmanFilter()->statePre.at<float>(0) << ", " << trackers[i]->getKalmanFilter()->statePre.at<float>(1) << endl;
        // First predict, to update the internal statePre variable
        Mat prediction = trackers[i]->getKalmanFilter()->predict();

        Point3f new_position(prediction.at<float>(0), prediction.at<float>(1), prediction.at<float>(2));


        cout << "TRACKER " << trackers[i]->getId() << ": " << trackers[i]->getPosition().x << ", " << trackers[i]->getPosition().y << ", " << trackers[i]->getPosition().z;
        cout << " Predicted center " << new_position.x << ", " << new_position.y << ", " << new_position.z << endl;
        cout << "PRED VELOCITY: " << prediction.at<float>(3) << ", " << prediction.at<float>(4) << ", " << prediction.at<float>(5) << endl;


        trackers[i]->setPosition(new_position);

        //projection on image planes       

    }
}

void Tracking::correct() {
    Mat_<float> measurement(3, 1);

    for (int i = 0; i < trackers.size(); i++) {
        if (trackers_not_used_cam1[trackers[i]->getId()] == 0 && trackers_not_used_cam2[trackers[i]->getId()] == 0) {
            // cout << "CORRECTION" << endl;
            measurement(0) = trackers[i]->getPosition().x;
            measurement(1) = trackers[i]->getPosition().y;
            measurement(2) = trackers[i]->getPosition().z;

            //cout << "MEASUREMENT TRACKER " << trackers[x]->getId() << ": " << measurement(0) << ", " << measurement(1) << endl;

            // The "correct" phase that is going to use the predicted value and our measurement
            Mat estimated = trackers[i]->getKalmanFilter()->correct(measurement);

        }

    }

}

void Tracking::computeProjectedPoints(Mat K_cam1, Mat K_cam2, Mat R, Mat T) {

    vector<Point3f> points_3D;
    for (int i = 0; i < trackers.size(); i++) {
        points_3D.push_back(trackers[i]->getPosition());

    }

    Mat rvec = Mat::zeros(3, 1, CV_32F);
    Mat tvec = Mat::zeros(3, 1, CV_32F);


    vector<Point2f> projected_pts_cam1, projected_pts_cam2;
    projectPoints(points_3D, rvec, tvec, K_cam1, Mat(), projected_pts_cam1);

    Rodrigues(R, rvec);
    projectPoints(points_3D, rvec, T, K_cam2, Mat(), projected_pts_cam2);

    vector<TrackerClass*> aux;

    for (int i = 0; i < trackers.size(); i++) {

        if (projected_pts_cam1[i].x > K_cam1.at<double>(0, 2)*2 || projected_pts_cam1[i].y > K_cam1.at<double>(1, 2)*2
                || projected_pts_cam2[i].x > K_cam2.at<double>(0, 2)*2 || projected_pts_cam2[i].y > K_cam2.at<double>(1, 2)*2) {
            trackers_not_used_cam1.erase(this->trackers[i]->getId());
            trackers_not_used_cam2.erase(this->trackers[i]->getId());
            cout << "***********Removing tracker (out of bounds) " << this->trackers[i]->getId() << endl;
        } else {
            trackers[i]->setReprojectedPtCam1(projected_pts_cam1[i]);
            trackers[i]->setReprojectedPtCam2(projected_pts_cam2[i]);

            Rect2d bbox(projected_pts_cam1[i].x - trackers[i]->getBoxCam1().width / 2.0,
                    projected_pts_cam1[i].y - trackers[i]->getBoxCam1().height / 2.0,
                    trackers[i]->getBoxCam1().width, trackers[i]->getBoxCam1().height);

            if(bbox.x < 0)
                bbox.x = 0;
            if(bbox.y < 0)
                bbox.y = 0;
            if((bbox.x + bbox.width) >  K_cam1.at<double>(0, 2)*2)
                bbox.width = K_cam1.at<double>(0, 2)*2 - bbox.x;
            if((bbox.y + bbox.height) >  K_cam1.at<double>(1, 2)*2)
                bbox.height = K_cam1.at<double>(1, 2)*2 - bbox.y;
            
            trackers[i]->setBoxCam1(bbox);

            bbox = Rect2d(projected_pts_cam2[i].x - trackers[i]->getBoxCam2().width / 2.0,
                    projected_pts_cam2[i].y - trackers[i]->getBoxCam2().height / 2.0,
                    trackers[i]->getBoxCam2().width, trackers[i]->getBoxCam2().height);

            if(bbox.x < 0)
                bbox.x = 0;
            if(bbox.y < 0)
                bbox.y = 0;
            if((bbox.x + bbox.width) >  K_cam2.at<double>(0, 2)*2)
                bbox.width = K_cam2.at<double>(0, 2)*2 - bbox.x;
            if((bbox.y + bbox.height) >  K_cam2.at<double>(1, 2)*2)
                bbox.height = K_cam2.at<double>(1, 2)*2 - bbox.y;
            
            trackers[i]->setBoxCam2(bbox);

            aux.push_back(trackers[i]);
        }

    }
    this->trackers = aux;
}

void Tracking::compute3DPoints(Mat &K_cam1, Mat &K_cam2, Mat &R1, Mat &R2, Mat &P1, Mat &P2, bool verification) {

    vector<Point2f> pts_cam1, pts_cam2;

    for (int i = 0; i < trackers.size(); i++) {
        pts_cam1.push_back(this->getBoxCenter(trackers[i]->getAssignedBoxCam1()));
        pts_cam2.push_back(this->getBoxCenter(trackers[i]->getAssignedBoxCam2()));

    }

    vector<Point2f> rec_points_cam1, rec_points_cam2;
    undistortPoints(pts_cam1, rec_points_cam1, K_cam1, Mat(), R1, P1);
    undistortPoints(pts_cam2, rec_points_cam2, K_cam2, Mat(), R2, P2);

    P1.convertTo(P1, CV_32F);
    P2.convertTo(P2, CV_32F);
    Mat point4D;
    triangulatePoints(P1, P2, rec_points_cam1, rec_points_cam2, point4D);

    Mat row = (cv::Mat_<float>(1, 4) << 0., 0., 0., 1.);
    Mat col = (cv::Mat_<float>(3, 1) << 0., 0., 0.);
    Mat R1_;
    R1.convertTo(R1_, CV_32F);
    hconcat(R1_, col, R1_);
    vconcat(R1_, row, R1_);

    point4D = R1_.inv() * point4D;

    //cout << point4D.t() << endl;
    vector<Point3f> points_3D(rec_points_cam1.size());


    for (int i = 0; i < point4D.cols; i++) {
        points_3D[i] = Point3f(point4D.at<float>(0, i) / point4D.at<float>(3, i),
                point4D.at<float>(1, i) / point4D.at<float>(3, i),
                point4D.at<float>(2, i) / point4D.at<float>(3, i));
    }
    //here we have a points 3D with reference on Cam1
    cout << points_3D << endl;




  
    for (int i = 0; i < points_3D.size(); i++) {
        if (trackers[i]->exists_previous_3D_position == false) {
            trackers[i]->setPosition(points_3D[i]);
            trackers[i]->initializeKalmanFilter();
            trackers[i]->exists_previous_3D_position = true;
        }else {
            
            float dist3D = this->compute3DDistance(points_3D[i], trackers[i]->getPosition());

            bool assigned = false;

            if (trackers[i]->getAssignedCam2() && trackers[i]->getAssignedCam1()) {
                assigned = true;
            }

            int not_used = 0;
            if (trackers_not_used_cam1[trackers[i]->getId()] == trackers_not_used_cam2[trackers[i]->getId()])
                not_used = trackers_not_used_cam1[trackers[i]->getId()];
            if (trackers_not_used_cam1[trackers[i]->getId()] > trackers_not_used_cam2[trackers[i]->getId()])
                not_used = trackers_not_used_cam1[trackers[i]->getId()];
            if (trackers_not_used_cam1[trackers[i]->getId()] < trackers_not_used_cam2[trackers[i]->getId()])
                not_used = trackers_not_used_cam2[trackers[i]->getId()];



            float dist_cam1 = this->distBetweenBoxes(trackers[i]->getAssignedBoxCam1(), trackers[i]->getBoxCam1());
            float dist_cam2 = this->distBetweenBoxes(trackers[i]->getAssignedBoxCam2(), trackers[i]->getBoxCam2());


            Point3f displacement = trackers[i]->getPosition() - points_3D[i];


            cout << "tracker " << trackers[i]->getId() << " Dist3D " << dist3D << " not used " << not_used << endl;
            cout << "displacement " << displacement.x << ", " << displacement.y << ", " << displacement.z << endl;



            float max_disp = 75;

            if (assigned) {
                // if (dist3D < 250 * (not_used + 1) && fabs(dist_cam1 - dist_cam2) < 80) {
                if (fabs(displacement.x) < max_disp * (not_used + 1) && fabs(displacement.y) < max_disp * (not_used + 1) &&
                        fabs(displacement.y) < max_disp * (not_used + 1) && fabs(dist_cam1 - dist_cam2) < 80) {


                    trackers[i]->setPosition(points_3D[i]);
                    trackers_not_used_cam1[trackers[i]->getId()] = 0;
                    trackers_not_used_cam2[trackers[i]->getId()] = 0;
                    trackers[i]->setBoxCam1(trackers[i]->getAssignedBoxCam1());
                    trackers[i]->setBoxCam2(trackers[i]->getAssignedBoxCam2());


                } else {
                    trackers_not_used_cam1[trackers[i]->getId()] += 1;
                    trackers_not_used_cam2[trackers[i]->getId()] += 1;
                    cout << "Bad assignment 3D" << endl;

                }

            } else {
                cout << "Not assigned" << endl;
            }
        }
    }
       
}

