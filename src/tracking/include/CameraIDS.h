/* 
 * File:   CameraIDS.h
 * Author: adrianacostas
 *
 * Created on September 22, 2021, 3:58 PM
 */
#include <uEye.h>
#include <pthread.h>
#include <opencv2/core/core.hpp>
#include <signal.h>
#include <iostream>

#ifndef CAMERAIDS_H
#define	CAMERAIDS_H

#define CAMERAS_NOT_PLUGGED 2
#define LIVE 0
#define SNAPSHOT 1

using namespace std;
using namespace cv;

class CameraIDS {
public:
    pthread_t cam_thread;
    pthread_mutex_t cam_mutex;
    SENSORINFO sensor_info;
    char* img_mem;
    int img_id;
    volatile int stop;

    CameraIDS();
    CameraIDS(const CameraIDS& orig);
    virtual ~CameraIDS();

    HIDS getCameraId() {
        return this->id;
    }

    int getMode() {
        return this->mode;
    }

    Mat getFrame() {
        Mat img;
        if (this->mode == LIVE) {
            pthread_mutex_lock(&this->cam_mutex);
	    this->frame.copyTo(img);
            this->received = false;
            pthread_mutex_unlock(&this->cam_mutex);
        } else if (this->mode == SNAPSHOT) {
            if (is_FreezeVideo(this->id, IS_WAIT) != IS_SUCCESS) {
                //cout<<is_FreezeVideo(cam_list[0].uci[0].dwCameraID, IS_WAIT)<<endl;
                fprintf(stdout, "Error on FreezeVideo\n");
                fflush(stdout);
            }
            img = Mat(this->sensor_info.nMaxWidth, this->sensor_info.nMaxHeight, CV_8UC3, this->img_mem);
        }




        return img;
    }

    bool isFrame() {
        bool isReceived = false;

        pthread_mutex_lock(&this->cam_mutex);
        isReceived = this->received;
        pthread_mutex_unlock(&this->cam_mutex);
        return isReceived;
    }

    void setCameraId(HIDS id) {
        this->id = id;
    }

    void setMode(int mode) {
        this->mode = mode;
        if (this->mode == LIVE) {
	  fprintf(stdout, "Configuring live mode\n");
	  fflush(stdout);
	  
            is_SetExternalTrigger(this->id, IS_SET_TRIGGER_OFF);
            is_EnableEvent(this->id, IS_SET_EVENT_FRAME);
        }
    }

    void setIsFrame(bool received) {
        
        this->received = received;
       

    }
    void setFrame(Mat img){
      this->frame = img;
      //img.copyTo(this->frame);
    }
    
    int initialize_camera(int mode, float fps = 0.0, float exp_t = 0.0);

    void startCapture() {
        fprintf(stdout, "Initializing thread for cam %d\n", this->id);
        fflush(stdout);
          this->stop = 0;
        pthread_create(&cam_thread, 0, read_camera, (void*) this);
	 
	 
    }

    void stopCapture() {
        if (this->mode == LIVE) {
            if (is_StopLiveVideo(this->id, IS_WAIT) == IS_SUCCESS) {
                fprintf(stdout, "\nCamera %d correctly stopped.\n", this->id);
                fflush(stdout);
            }

            pthread_kill(cam_thread, 1);
        }
    }

    void exitCamera() {
        if (is_FreeImageMem(this->id, this->img_mem, this->img_id) == IS_SUCCESS) {
            fprintf(stdout, "\nCamera %d correctly free.\n", this->id);
            fflush(stdout);
        }


        if (is_ExitCamera(this->id) == IS_SUCCESS) {
            fprintf(stdout, "\nCamera %d correctly closed.\n", this->id);
            fflush(stdout);
        }
    }

    static void getCamerasInfo() {
    int num_cam, i;
    DWORD ver = is_GetDLLVersion();
    PUEYE_CAMERA_LIST cam_list;

    if (is_GetNumberOfCameras(&num_cam) != IS_SUCCESS) {
        fprintf(stdout, "Could not get the number of cameras\n");
        exit(-1);
    }

    fprintf(stdout, "\nOpenCV library %d.%d\n", ver >> 24, ver >> 16 & 0xFF);

    //cam_ids = (unsigned int*) malloc(num_cam * sizeof (unsigned int));
    cam_list = (PUEYE_CAMERA_LIST) malloc(sizeof (ULONG) + num_cam * sizeof (UEYE_CAMERA_INFO));
    cam_list->dwCount = num_cam;
    if (is_GetCameraList(cam_list) == IS_SUCCESS) {
        for (i = 0; i < num_cam; i++) {


            fprintf(stdout, "---- CAMERA %d INFO ----\n", i + 1);
            fprintf(stdout, "\t Camera ID: %d\n", cam_list->uci[i].dwCameraID);
            fprintf(stdout, "\t Device ID: %d\n", cam_list->uci[i].dwDeviceID);
            fprintf(stdout, "\t Sensor ID: %d\n", cam_list->uci[i].dwSensorID);
            fprintf(stdout, "\t Is in use: %d\n", cam_list->uci[i].dwInUse);
            fprintf(stdout, "\t Serial number: %s\n", cam_list->uci[i].SerNo);
            fprintf(stdout, "\t Model: %s\n", cam_list->uci[i].Model);
            fprintf(stdout, "\t Status: %d\n", cam_list->uci[i].dwStatus);
            fprintf(stdout, "\t Reserved: %d, %d\n", cam_list->uci[i].dwReserved[0],
                    cam_list->uci[i].dwReserved[1]);
            fprintf(stdout, "\t Model name: %s\n", cam_list->uci[i].FullModelName);


        }



    } else {
        fprintf(stdout, "Error getting the cameras list\n");
        exit(-1);
    }
}
    


private:

   

    HIDS id;
    Mat frame;
    int mode;
    volatile bool received;
    
     static void* read_camera(void* arg) {

        CameraIDS* cam = reinterpret_cast<CameraIDS*> (arg);
	HIDS h_cam = cam->getCameraId();

	fprintf(stdout, "Thread cam %d\n", h_cam);
	fflush(stdout);

	       
        int ret = 0;
        ret = is_CaptureVideo(h_cam, IS_WAIT);
        if (ret != IS_SUCCESS) {
            fprintf(stdout, "Error on CaptureVideo\n");
            fflush(stdout);
        }
	
  
// 	usleep(5000000);
// 	
// 	 double tFPS = 0;
// 	 is_GetFramesPerSecond (h_cam, &tFPS);
// 	 cout << "fps: " << tFPS << endl;
  
        Mat frame;
        while (!cam->stop) {
	   //cout << "Waiting event cam " << h_cam << " ";
	   
            if (is_WaitEvent(h_cam, IS_SET_EVENT_FRAME, 100) == IS_SUCCESS) {
                //fprintf(stdout, "Frame received from camera %d\n", h_cam);
                //fflush(stdout);

                pthread_mutex_lock(&cam->cam_mutex);

                is_LockSeqBuf(h_cam, IS_IGNORE_PARAMETER, cam->img_mem);
		
		
		cam->setFrame(Mat(cam->sensor_info.nMaxHeight, cam->sensor_info.nMaxWidth, CV_8UC3, cam->img_mem));
		
                is_UnlockSeqBuf(h_cam, IS_IGNORE_PARAMETER, cam->img_mem);
                cam->setIsFrame(true);

                pthread_mutex_unlock(&cam->cam_mutex);
		
		
            }
           
        }
         cam->stopCapture();
        

    }



};

#endif	/* CAMERAIDS_H */

