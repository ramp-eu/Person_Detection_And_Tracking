#include <camera_tracker/camera_tracker.h>

using namespace cv;
using namespace std;
using namespace cv::dnn;

namespace camera_tracker
{

    CameraTracker::CameraTracker(ros::NodeHandle &n, ros::NodeHandle &p)
    {
        this->nh = n;
        this->pn = p;
        ROS_DEBUG("Install parameters");
        this->install_params();

        // Load calibration params
        ROS_DEBUG("Load calibration parameters");
        this->loadCalibrationParameters();

        // Load names of classes
        ROS_DEBUG("Load name of classes");
        ifstream ifs(yolo_classes_file.c_str());
        string line;
        while (getline(ifs, line))
            classes.push_back(line);

        // Load the network
        ROS_DEBUG("Load the network");
        net_yolo = cv::dnn::readNetFromDarknet(yolo_model_file, yolo_weights_file);
        net_yolo.setPreferableBackend(DNN_BACKEND_CUDA);
        net_yolo.setPreferableTarget(DNN_TARGET_CUDA);

        ROS_DEBUG("initUndistortRectifyMap");
        cv::initUndistortRectifyMap(K_cam1, dist_coeffs_cam1, Mat(), K_cam1, Size(img_width_cam1, img_height_cam1), CV_32F, map1_cam1, map2_cam1);
        cv::initUndistortRectifyMap(K_cam2, dist_coeffs_cam2, Mat(), K_cam2, Size(img_width_cam2, img_height_cam2), CV_32F, map1_cam2, map2_cam2);

        this->state = STOP;
        this->output_image_1.reset(new cv_bridge::CvImage);
        this->output_image_2.reset(new cv_bridge::CvImage);

        ROS_DEBUG("Topics configuration");
        this->pub_worker_obstacles= this->nh.advertise<mapupdates_msgs::NewObstacles>("/worker/newObstacles", 1);
        this->pub_worker_position = this->nh.advertise<geometry_msgs::PoseArray>("/worker_position", 1);
        this->pub_traking_image_1 = this->pn.advertise<sensor_msgs::Image>("tracking_image_1", 1);
        this->pub_traking_image_2 = this->pn.advertise<sensor_msgs::Image>("tracking_image_2", 1);
        
        this->sub_cam_1_image = this->nh.subscribe<sensor_msgs::Image>("/camera1/image_raw", 1, &CameraTracker::camera1ImageCb, this);
        this->sub_cam_2_image = this->nh.subscribe<sensor_msgs::Image>("/camera2/image_raw", 1, &CameraTracker::camera2ImageCb, this);

        // this->timer = this->nh.createTimer(ros::Duration(1./(2. * this->fps)), &CameraTracker::runStateMachineCb, this);
        this->timer = this->nh.createTimer(ros::Duration(this->timer_rate), &CameraTracker::runStateMachineCb, this);

        ROS_DEBUG("Initialization OK");
    }

    CameraTracker::~CameraTracker()
    {
        this->state = STOP;
        this->shutdown_tracker(0);
    }

    double CameraTracker::get_current_time(void)
    {
        struct timespec time;
        clock_gettime(CLOCK_REALTIME, &time);
        return (time.tv_sec + time.tv_nsec / 1.0e09);
    }

    void CameraTracker::shutdown_tracker(int error)
    {
        this->stop = 1;
    }

    void CameraTracker::postprocess(cv::Mat &frame, const vector<cv::Mat> &outs, vector<Detection *> &detections)
    {
        vector<int> classIds;
        vector<float> confidences;
        vector<cv::Rect2d> boxes;

        vector<cv::Point2f> boxsPosition;
        vector<cv::Rect2d> boxesInfo;
        for (size_t i = 0; i < outs.size(); ++i)
        {
            // Scan through all the bounding boxes output from the network and keep only the
            // ones with high confidence scores. Assign the box's class label as the class
            // with the highest score for the box.
            float *data = (float *)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
            {
                cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                cv::Point classIdPoint;
                double confidence;
                // Get the value and location of the maximum score
                minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                if ((confidence > confThreshold) && (classIdPoint.x == 0)) //////////COPSSSSSSSSSS
                {
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;
                    // cout<< "we got any box good"<<endl;
                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(cv::Rect2d(left, top, width, height));
                }
            }
        }

        // Perform non maximum suppression to eliminate redundant overlapping boxes with
        // lower confidences
        vector<int> indices;
        NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

        bool detected = false;
        for (size_t i = 0; i < indices.size(); ++i)
        {
            detected = true;
            int idx = indices[i];
            cv::Rect2d box = boxes[idx];

            // cout<<box.x<<" "<<box.y<< " "<<box.x + box.width<<" "<<box.y + box.height<<endl;
            box.x -= 50;
            box.y -= 50;
            box.width += 100;
            box.height += 100;

            if (box.x < 0)
            {
                box.x = 0;
            }
            if (box.y < 0)
            {
                box.y = 0;
            }
            if (box.x + box.width > frame.cols)
            {
                box.width = 2048 - box.x;
            } // max range
            if (box.y + box.height > frame.rows)
            {
                box.height = 1088 - box.y;
            }

            // if we want the center position of boxs, uncomment the next and change centerX instead of box.x
            int centerX = box.x + box.width / 2;
            int centerY = box.y + box.height / 2;
            cv::Point2f box_position(centerX, centerY);

            boxsPosition.push_back(box_position);

            boxesInfo.push_back(box);

            Detection *person = new Detection(box);
            detections.push_back(person);
        }
    }

    // Get the names of the output layers
    vector<string> CameraTracker::getOutputsNames(const Net &net)
    {
        static vector<string> names;
        if (names.empty())
        {
            // Get the indices of the output layers, i.e. the layers with unconnected outputs
            vector<int> outLayers = net.getUnconnectedOutLayers();

            // get the names of all the layers in the network
            vector<string> layersNames = net.getLayerNames();

            // Get the names of the output layers in names
            names.resize(outLayers.size());
            for (size_t i = 0; i < outLayers.size(); ++i)
                names[i] = layersNames[outLayers[i] - 1];
        }
        return names;
    }

    void CameraTracker::peopleDetection(Net &net_, cv::Mat &img, int inpWidth, int inpHeight, vector<Detection *> &detections)
    {
        // Create a 4D blob from a frame.
        cv::Mat blob;
        blobFromImage(img, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(0, 0, 0), true, false);

        // Sets the input to the network
        net_.setInput(blob);

        // Runs the forward pass to get output of the output layers
        vector<cv::Mat> outs;
        net_.forward(outs, getOutputsNames(net_));
        // cout<<"before postprocces"<<endl;
        //  Remove the bounding boxes with low confidence
        postprocess(img, outs, detections);
    }

    float CameraTracker::distancePointLine(cv::Point2f p, cv::Point3f line)
    {
        // Line is given as a*x + b*y + c = 0
        // d = (a*p.x + b*p.y+c)/sqrt(a²+b²)
        return std::fabs(line.x * p.x + line.y * p.y + line.z) / std::sqrt(line.x * line.x + line.y * line.y);
    }

    void CameraTracker::stereo_matching(vector<Detection *> &detections_cam1, vector<Detection *> &detections_cam2,
                                              Mat &F, vector<int> &assignment, vector<vector<double>> &cost_matrix, Mat img1 = Mat(), Mat img2 = Mat())
    {

        vector<cv::Point2f> boxsPosition_cam1, boxsPosition_cam2;
        for (int i = 0; i < detections_cam1.size(); i++)
        {
            boxsPosition_cam1.push_back(detections_cam1[i]->getCenter());
            // cout << "x " << boxsPosition_cam1[i].x << ", y: " << boxsPosition_cam1[i].y << endl;
        }
        for (int i = 0; i < detections_cam2.size(); i++)
        {
            boxsPosition_cam2.push_back(detections_cam2[i]->getCenter());
            // cout << "x " << boxsPosition_cam2[i].x << ", y: " << boxsPosition_cam2[i].y << endl;
        }

        vector<cv::Point3f> lines_img1, lines_img2;
        // Line is given as a*x + b*y + c = 0
        computeCorrespondEpilines(boxsPosition_cam1, 1, F, lines_img2);
        computeCorrespondEpilines(boxsPosition_cam2, 2, F, lines_img1);

        vector<double> cost_per_line(detections_cam2.size(), 1.);
        cost_matrix.resize(detections_cam1.size(), cost_per_line);

        for (size_t i = 0; i < lines_img2.size(); i++)
        {
            for (int j = 0; j < boxsPosition_cam2.size(); j++)
            {

                double dist = distancePointLine(boxsPosition_cam2[j], lines_img2[i]);
                double score = dist;
                // cout << "Box_cam1 " << i << " box_cam2 " << j << " score: " << score << endl;

                cost_matrix[i][j] = score;
            }
        }

        for (size_t i = 0; i < lines_img1.size(); i++)
        {
            for (int j = 0; j < boxsPosition_cam1.size(); j++)
            {

                double dist = distancePointLine(boxsPosition_cam1[j], lines_img1[i]);
                double score = dist;
                // cout << "Box_cam1 " << j << " box_cam2 " << i << " score: " << score << endl;
                cost_matrix[j][i] += score;
            }
        }

        HungarianAlgorithm HungAlgo;
        double cost = HungAlgo.Solve(cost_matrix, assignment);

        // cout << "cost: " << cost << endl;
    }

    void CameraTracker::loadCalibrationParameters()
    {
        try
        {
            cv::FileStorage fs(cv::String(this->cam_calibration_file.c_str()), FileStorage::READ);
            ROS_DEBUG("read file OK");
            fs["K_cam1"] >> this->K_cam1;
            fs["dist_coeffs_cam1"] >> this->dist_coeffs_cam1;
            fs["img_width_cam1"] >> this->img_width_cam1;
            fs["img_height_cam1"] >> this->img_height_cam1;
            fs["K_cam2"] >> this->K_cam2;
            fs["dist_coeffs_cam2"] >> this->dist_coeffs_cam2;
            fs["img_width_cam2"] >> this->img_width_cam2;
            fs["img_height_cam2"] >> this->img_height_cam2;
            fs["R"] >> this->R;
            fs["T"] >> this->T;
            fs["F"] >> this->F;
            fs["P1"] >> this->P1;
            fs["P2"] >> this->P2;
            fs["R1"] >> this->R1;
            fs["R2"] >> this->R2;
            fs.release();
        }
        catch (cv::Exception &ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        catch (std::exception &ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        catch (...)
        {
            ROS_ERROR("error loading calibration parameters");
        }
    }

    void CameraTracker::install_params()
    {
        if (!this->nh.getParam("/tracking/h_cam1", this->cam1_id))
        {
            ROS_ERROR("no param /tracking/h_cam1");
            exit(0);
        }
        ROS_DEBUG("/tracking/h_cam1 = %d", this->cam1_id);

        if (!this->nh.getParam("/tracking/h_cam2", this->cam2_id))
        {
            ROS_ERROR("no param /tracking/h_cam2");
            exit(0);
        }
        ROS_DEBUG("/tracking/h_cam2 = %d", this->cam2_id);

        {
            XmlRpc::XmlRpcValue tmp;
            if (!this->nh.getParam("/tracking/exp_t1", tmp))
            {
                ROS_ERROR("no param /tracking/exp_t1");
                exit(0);
            }
            ROS_ASSERT(tmp.getType() == XmlRpc::XmlRpcValue::TypeDouble);
            this->exp_t1 = tmp;
            ROS_DEBUG("/tracking/exp_t1 = %f", this->exp_t1);
        }

        {
            XmlRpc::XmlRpcValue tmp;
            if (!this->nh.getParam("/tracking/exp_t2", tmp))
            {
                ROS_ERROR("no param /tracking/exp_t2");
                exit(0);
            }
            ROS_ASSERT(tmp.getType() == XmlRpc::XmlRpcValue::TypeDouble);
            this->exp_t2 = tmp;
            ROS_DEBUG("/tracking/exp_t2 = %f", this->exp_t2);
        }

        {
            XmlRpc::XmlRpcValue tmp;
            if (!this->nh.getParam("/tracking/fps", tmp))
            {
                ROS_ERROR("no param /tracking/fps");
                exit(0);
            }
            ROS_ASSERT(tmp.getType() == XmlRpc::XmlRpcValue::TypeDouble);
            this->fps = tmp;
            ROS_DEBUG("/tracking/fps = %f", this->fps);
        }

        {
            XmlRpc::XmlRpcValue tmp;
            if (!this->nh.getParam("/tracking/timer_rate", tmp))
            {
                ROS_ERROR("no param /tracking/timer_rate");
                exit(0);
            }
            ROS_ASSERT(tmp.getType() == XmlRpc::XmlRpcValue::TypeDouble);
            this->timer_rate = tmp;
            ROS_DEBUG("/tracking/timer_rate = %f", this->timer_rate);
        }

        if (!this->nh.getParam("/tracking/cameras_calibration", this->cam_calibration_file))
        {
            ROS_ERROR("no param /tracking/cameras_calibration");
            exit(0);
        }
        this->checkIfFileExists(this->cam_calibration_file);
        ROS_DEBUG("/tracking/cameras_calibration = %s", this->cam_calibration_file.c_str());

        if (!this->nh.getParam("/tracking/yolo_classes_file", this->yolo_classes_file))
        {
            ROS_ERROR("no param /tracking/yolo_classes_file");
            exit(0);
        }
        this->checkIfFileExists(this->yolo_classes_file);
        ROS_DEBUG("/tracking/yolo_classes_file = %s", this->yolo_classes_file.c_str());

        if (!this->nh.getParam("/tracking/yolo_model_file", this->yolo_model_file))
        {
            ROS_ERROR("no param /tracking/yolo_model_file");
            exit(0);
        }
        this->checkIfFileExists(this->yolo_model_file);
        ROS_DEBUG("/tracking/yolo_model_file = %s", this->yolo_model_file.c_str());

        if (!this->nh.getParam("/tracking/yolo_weights_file", this->yolo_weights_file))
        {
            ROS_ERROR("no param /tracking/yolo_weights_file");
            exit(0);
        }
        this->checkIfFileExists(this->yolo_weights_file);
        ROS_DEBUG("/tracking/yolo_weights_file = %s", this->yolo_weights_file.c_str());
    }

    void CameraTracker::run()
    {
        // State Machine
        // STOP -> INIT -> READING -> DETECT -> TRACK -> READING
        //                                           L-> PUBLISH -> READING
        switch (state)
        {
        case STOP:
        {
            ROS_DEBUG("STOP");
            ROS_INFO("System starts in 5 seconds");
            usleep(5000000);
            this->state = INIT;
            break;
        }
        case INIT:
        {
            ROS_DEBUG("INIT");
            // usleep(5000000);
            this->start = get_current_time();
            this->state = READING;
            break;
        }
        case READING:
        {
            ROS_DEBUG("READING");
            if (this->camera_msg_1 != nullptr && this->camera_msg_2 != nullptr)
            {
                ROS_DEBUG("\t get images");
                this->rosMsgToCvImg(this->camera_msg_1, this->input_image_1);
                this->rosMsgToCvImg(this->camera_msg_2, this->input_image_2);
                this->camera_msg_1 = nullptr;
                this->camera_msg_2 = nullptr;
                // this->state = DETECT;
                
                // // if (this->first)
                {
                    this->no_frames++;
                    this->finish = get_current_time();
                    double time = finish - start;
                    if (time >= 1.)
                    {
                        if (this->no_frames <= this->fps + 1)
                            this->first = false;

                        ROS_DEBUG("\t num_frames: %d - time: %f", no_frames, time);
                        this->start = get_current_time();
                        this->no_frames = 0;
                    }
                }
                // else // this->first == false
                if(!this->first)
                {
                    this->state = DETECT;
                }

                // check num_frames == 30 and time < 1
                // no_frames++;
                // if (no_frames == int(fps))
                // {
                //     finish = get_current_time();
                //     double time = finish - start;
                //     ROS_DEBUG("READING - num_frames: %d - time: %f", no_frames, time);

                //     this->start = get_current_time();
                //     this->no_frames = 0;

                //     ROS_DEBUG("%f", fabs(time - 1.));

                //     if ((first && fabs(time - 1.) < this->sample_time_limit))
                //     {
                //         this->state = DETECT;
                //         this->first = false;
                //     }
                //     else
                //     {
                //         ROS_DEBUG("READING - faster!");
                //     }
                // }

                // if (!this->first)
                //     this->state = DETECT;
            }
            break;
        }
        case DETECT:
        {
            ROS_DEBUG("DETECT");

            remap(this->input_image_1->image, this->output_image_1->image, map1_cam1, map2_cam1, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
            remap(this->input_image_2->image, this->output_image_2->image, map1_cam2, map2_cam2, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));

            this->detections_cam1.clear();
            this->detections_cam2.clear();

            peopleDetection(this->net_yolo, this->output_image_1->image, this->inpWidth, this->inpHeight, this->detections_cam1);
            peopleDetection(this->net_yolo, this->output_image_2->image, this->inpWidth, this->inpHeight, this->detections_cam2);

            this->state = TRACK;

            break;
        }
        case TRACK:
        {
            ROS_DEBUG("TRACK");

            if (detections_cam1.size() > 0 && detections_cam2.size() > 0)
            {
                if (tracking_node.getTrackers().size() == 0)
                {
                    vector<int> assignment;
                    vector<vector<double>> cost_matrix;

                    stereo_matching(detections_cam1, detections_cam2, F, assignment, cost_matrix);

                    for (unsigned int x = 0; x < assignment.size(); x++)
                    {
                        // cout << "**" << endl;
                        // std::cout << "Assignment: box_cam1 " << x << ", box_cam2 : " << assignment[x] << endl;

                        if (assignment[x] == -1)
                            continue;

                        if (cost_matrix[x][assignment[x]] > 50)
                        {
                            // cout << "Bad assigment -> cost:  " << cost_matrix[x][assignment[x]] << endl;
                            continue;
                        }

                        Point2f stereo_dist = detections_cam2[assignment[x]]->getCenter() - detections_cam1[x]->getCenter();
                        double stereo_dist_norm = sqrt(pow(stereo_dist.x, 2) + pow(stereo_dist.y, 2));

                        double area_ratio = (detections_cam2[assignment[x]]->getHeight() * detections_cam2[assignment[x]]->getWidth()) / (detections_cam1[x]->getHeight() * detections_cam1[x]->getWidth());
                        // cout << "height ratio " << area_ratio << " " << 1. / area_ratio << endl;
                        if (stereo_dist_norm > 800 || area_ratio < 0.8 || 1. / area_ratio < 0.8)
                        {

                            // cout << "Bad assignment " << stereo_dist_norm << endl;
                            continue;
                        }

                        TrackerClass *tracker = new TrackerClass(detections_cam1[x]->getBox(), detections_cam2[assignment[x]]->getBox());
                        tracking_node.add(tracker);
                        // cout << "Creatig new tracker " << tracker->getId() << endl;
                    }

                    // cout << "Trackers size: " << tracking_node.getTrackers().size() << endl;

                    if (tracking_node.getTrackers().size() > 0)
                    {
                        tracking_node.compute3DPoints(K_cam1, K_cam2, R1, R2, P1, P2);
                    }
#ifdef SHOW_IMAGES

                    for (int i = 0; i < tracking_node.getTrackers().size(); i++)
                    {
                        rectangle(this->output_image_1->image, tracking_node.getTrackers()[i]->getBoxCam1(), tracking_node.getTrackers()[i]->getColor(), 4, 1);
                        cv::circle(this->output_image_1->image, tracking_node.getTrackers()[i]->getBoxCam1Center(), 10, tracking_node.getTrackers()[i]->getColor(), -1, cv::LINE_AA);
                        rectangle(this->output_image_2->image, tracking_node.getTrackers()[i]->getBoxCam2(), tracking_node.getTrackers()[i]->getColor(), 4, 1);
                        cv::circle(this->output_image_2->image, tracking_node.getTrackers()[i]->getBoxCam2Center(), 10, tracking_node.getTrackers()[i]->getColor(), -1, cv::LINE_AA);
                        putText(this->output_image_1->image, to_string(tracking_node.getTrackers()[i]->getId()), Point(tracking_node.getTrackers()[i]->getBoxCam1().x, tracking_node.getTrackers()[i]->getBoxCam1().y - 10), FONT_HERSHEY_SIMPLEX, 1, tracking_node.getTrackers()[i]->getColor(), 2);
                        putText(this->output_image_2->image, to_string(tracking_node.getTrackers()[i]->getId()), Point(tracking_node.getTrackers()[i]->getBoxCam2().x, tracking_node.getTrackers()[i]->getBoxCam2().y - 10), FONT_HERSHEY_SIMPLEX, 1, tracking_node.getTrackers()[i]->getColor(), 2);
                    }

                    this->publishOutputImages();
#endif
                }
                else
                {
                    // Predict
                    tracking_node.predict();

                    // Project 3D point into images
                    tracking_node.computeProjectedPoints(K_cam1, K_cam2, R, T);

                    // Assignment between YOLO detection and projected points.

                    vector<Detection *> detections_not_assigned_cam1, detections_not_assigned_cam2;

                    tracking_node.hungarianTracking(detections_cam1, detections_cam2,
                                                    detections_not_assigned_cam1, detections_not_assigned_cam2);

                    if (detections_not_assigned_cam1.size() > 0 && detections_not_assigned_cam2.size() > 0)
                    {
                        // Assignment of not tracked boxes
                        vector<int> assignment;
                        vector<vector<double>> cost_matrix;
                        stereo_matching(detections_not_assigned_cam1, detections_not_assigned_cam2, F, assignment, cost_matrix);

                        for (unsigned int x = 0; x < assignment.size(); x++)
                        {
                            // cout << "**" << endl;
                            // std::cout << "Assignment: box_cam1 " << x << ", box_cam2 : " << assignment[x] << endl;

                            if (assignment[x] == -1)
                                continue;

                            if (cost_matrix[x][assignment[x]] > 120)
                            {
                                // cout << "Bad assigment -> cost:  " << cost_matrix[x][assignment[x]] << endl;
                                continue;
                            }

                            Point2f stereo_dist = detections_not_assigned_cam2[assignment[x]]->getCenter() - detections_not_assigned_cam1[x]->getCenter();
                            double stereo_dist_norm = sqrt(pow(stereo_dist.x, 2) + pow(stereo_dist.y, 2));

                            double area_ratio = (detections_not_assigned_cam2[assignment[x]]->getHeight() * detections_not_assigned_cam2[assignment[x]]->getWidth()) / (detections_not_assigned_cam1[x]->getHeight() * detections_not_assigned_cam1[x]->getWidth());
                            // cout << "height ratio " << area_ratio << " " << 1. / area_ratio << endl;
                            if (stereo_dist_norm > 850 || area_ratio < 0.6 || 1. / area_ratio < 0.6)
                            {

                                // cout << "Bad assignment " << stereo_dist_norm << endl;
                                continue;
                            }

                            TrackerClass *tracker = new TrackerClass(detections_not_assigned_cam1[x]->getBox(), detections_not_assigned_cam2[assignment[x]]->getBox());
                            tracking_node.add(tracker);
                        }

                        if (tracking_node.getTrackers().size() > 0)
                        {

                            tracking_node.compute3DPoints(K_cam1, K_cam2, R1, R2, P1, P2);
                        }
                    }

                    if (tracking_node.getTrackers().size() > 0)
                    {
                        // compute 3D points from assignments
                        tracking_node.compute3DPoints(K_cam1, K_cam2, R1, R2, P1, P2);

                        tracking_node.deleteNotUsedTrackers();

                        tracking_node.correct();
                    }

#ifdef SHOW_IMAGES

                    for (int i = 0; i < tracking_node.getTrackers().size(); i++)
                    {
                        rectangle(this->output_image_1->image, tracking_node.getTrackers()[i]->getBoxCam1(), tracking_node.getTrackers()[i]->getColor(), 4, 1);
                        putText(this->output_image_1->image, to_string(tracking_node.getTrackers()[i]->getId()),
                                Point(tracking_node.getTrackers()[i]->getBoxCam1().x, tracking_node.getTrackers()[i]->getBoxCam1().y - 10),
                                FONT_HERSHEY_SIMPLEX, 1, tracking_node.getTrackers()[i]->getColor(), 2);

                        rectangle(this->output_image_2->image, tracking_node.getTrackers()[i]->getBoxCam2(), tracking_node.getTrackers()[i]->getColor(), 4, 1);
                        putText(this->output_image_2->image, to_string(tracking_node.getTrackers()[i]->getId()),
                                Point(tracking_node.getTrackers()[i]->getBoxCam2().x, tracking_node.getTrackers()[i]->getBoxCam2().y - 10),
                                FONT_HERSHEY_SIMPLEX, 1, tracking_node.getTrackers()[i]->getColor(), 2);
                    }

                    this->publishOutputImages();
#endif
                }
            }
            else
            {
                for (int i = 0; i < tracking_node.getTrackers().size(); i++)
                {
                    tracking_node.addNotUsed(tracking_node.getTrackers()[i]->getId());
                }
                tracking_node.deleteNotUsedTrackers();
            }

            if (tracking_node.getTrackers().size() > 0)
            {
                state = PUBLISH;
            }
            else
            {
                ROS_DEBUG("No trackers active");
                state = READING;
            }

            break;
        }

        case PUBLISH:
        {
            ROS_DEBUG("PUBLISH");

            newobstacles_msg.header.frame_id = "camera1";
            newobstacles_msg.x.resize(tracking_node.getTrackers().size());
            newobstacles_msg.y.resize(tracking_node.getTrackers().size());  

            worker_positions_msg.header.frame_id = "camera1";
            worker_positions_msg.poses.resize(tracking_node.getTrackers().size());

            for (int i = 0; i < tracking_node.getTrackers().size(); i++)
            {

                cv::Mat point = (cv::Mat_<float>(3, 1) << tracking_node.getTrackers()[i]->getPosition().x, tracking_node.getTrackers()[i]->getPosition().y, tracking_node.getTrackers()[i]->getPosition().z);

                point /= 1000;

                newobstacles_msg.x[i] = point.at<float>(0,0);
                newobstacles_msg.y[i] = point.at<float>(1,0);

                worker_positions_msg.poses[i].position.x = point.at<float>(0, 0);
                worker_positions_msg.poses[i].position.y = point.at<float>(1, 0);
                worker_positions_msg.poses[i].position.z = point.at<float>(2, 0);
            }

            // Publish 3D points
            pub_worker_obstacles.publish(newobstacles_msg);
            pub_worker_position.publish(worker_positions_msg);

            state = READING;
            break;
        }
        }
    }

    inline void CameraTracker::checkIfFileExists(const std::string &file) const
    {
        struct stat buffer;
        ROS_DEBUG("Checking if file exists: %s", file.c_str());
        ROS_ASSERT(stat(file.c_str(), &buffer) == 0);
    }

    void CameraTracker::publishOutputImages()
    {
        this->output_image_1->encoding = this->input_image_1->encoding;
        this->output_image_2->encoding = this->input_image_2->encoding;
        this->output_image_1->header = this->input_image_1->header;
        this->output_image_2->header = this->input_image_2->header;
        this->pub_traking_image_1.publish(this->output_image_1->toImageMsg());
        this->pub_traking_image_2.publish(this->output_image_2->toImageMsg());
    }

    void CameraTracker::publishInputImages()
    {
        this->pub_traking_image_1.publish(this->input_image_1->toImageMsg());
        this->pub_traking_image_2.publish(this->input_image_2->toImageMsg());
    }

    void CameraTracker::camera1ImageCb(const sensor_msgs::Image::ConstPtr &msg)
    {
        // ROS_DEBUG("hz = %f", 1./(get_current_time() - this->start) );
        this->camera_msg_1 = msg;
        // this->start = get_current_time();
    }

    void CameraTracker::camera2ImageCb(const sensor_msgs::Image::ConstPtr &msg)
    {
        this->camera_msg_2 = msg;
    }

    void CameraTracker::rosMsgToCvImg(const sensor_msgs::Image::ConstPtr &msg, cv_bridge::CvImagePtr &img)
    {
        // ROS_DEBUG("rosMsgToCvImg");
        try
        {
            img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void CameraTracker::runStateMachineCb(const ros::TimerEvent &)
    {
        // ROS_DEBUG(">RUN CICLE");
        this->run();        
    }
} // namespace camera_tracker
