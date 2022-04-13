/* 
 * File:   CameraIDS.cpp
 * Author: adrianacostas
 * 
 * Created on September 22, 2021, 3:58 PM
 */

#include "CameraIDS.h"

CameraIDS::CameraIDS() {
    
    this->received = false;
    this->stop = 0;
}

CameraIDS::CameraIDS(const CameraIDS& orig) {
}

CameraIDS::~CameraIDS() {
}

int CameraIDS::initialize_camera(int mode, float fps, float exp_t) {

    HIDS h_cam = this->id;
    
    fprintf(stdout, "initializing camera %d \n", h_cam);
    fflush(stdout);

    if (h_cam != 0) {
        is_FreeImageMem(h_cam, img_mem, img_id);
        is_ExitCamera(h_cam);
    } else {
        fprintf(stdout, "Camera %d not identified yet\n", h_cam);
        return 0;
    }

    if (is_InitCamera(&h_cam, NULL) != IS_SUCCESS) {
        fprintf(stdout, "It is not possible to initialize the camera with id %d\n", h_cam);
        fflush(stdout);
        return 0;
    }

    if (is_GetSensorInfo(h_cam, &(sensor_info)) != IS_SUCCESS) {
        fprintf(stdout, "It is not possible to get the sensor information for the camera with id %d\n",
               h_cam);
        fflush(stdout);
        return 0;
    }


    if (is_SetColorMode(h_cam, IS_CM_BGR8_PACKED) != IS_SUCCESS) {
        fprintf(stdout, "It's not possible to set the color mode for the camera with id %d\n",
                h_cam);
        fflush(stdout);
        return 0;
    }

    if (is_AllocImageMem(h_cam, sensor_info.nMaxWidth, sensor_info.nMaxHeight, 24,
            &(img_mem), &(img_id)) != IS_SUCCESS) {
        fprintf(stdout, "It's not possible to allocate memory for the camera with id %d\n",
               h_cam);
        fflush(stdout);
        return 0;
    }


    if (is_SetImageMem(h_cam, img_mem, img_id) != IS_SUCCESS) {
        fprintf(stdout, "It's not possible to set the image memory for the camera with id %d\n",
                h_cam);
        fflush(stdout);
        return 0;

    }    
    
  
      if(fabs(fps) > 0.){    
      if(is_SetFrameRate(h_cam, fps, NULL)!= IS_SUCCESS){
	fprintf(stdout, "It's not possible to set framerate for the camera with id %d\n",
	      h_cam);
      fflush(stdout);
      }else{
	fprintf(stdout, " set framerate for the camera with id %d to %f\n",
	      h_cam, fps);
      fflush(stdout);
      }
    }
    
    if(fabs(exp_t) > 0.) 
      is_Exposure(h_cam, IS_EXPOSURE_CMD_SET_EXPOSURE, &exp_t, 8) != IS_SUCCESS;
     
    
     if (mode == LIVE) {
	  fprintf(stdout, "Configuring live mode\n");
	  fflush(stdout);
	  
            is_SetExternalTrigger(h_cam, IS_SET_TRIGGER_OFF);
            is_EnableEvent(h_cam, IS_SET_EVENT_FRAME);
        }
	    
    fprintf(stdout, "Camera %d initialized...\n", h_cam);
    fflush(stdout);

    return 1;
}

 