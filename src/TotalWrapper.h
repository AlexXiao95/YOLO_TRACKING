#ifndef _Total_WRAPPER_H_
#define _Total_WRAPPER_H_

/*
#include <iostream>
#include "opencv/cv.h"
#include <string.h>
//#include <unordered_map>
#include <ctime>
#include <assert.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv/cvaux.hpp>
#include <algorithm>
#include <cmath>
#include "Vector2.hpp"
//#include "Matrix.h"
*/


//#include "opencv/cv.h"
//#include <opencv2/highgui/highgui_c.h>
//using namespace cv;
//using namespace std;


struct tagTracking;
#ifdef __cplusplus
extern "C" {
#endif
struct tagTracking *GetInstance(void);
void ReleaseInstance(struct tagTracking **ppInstance);
//extern int trackOneFrame(struct tagTracking *pTracking, int i, cv::Mat& res, cv::Mat& image,vector<vector<int> > output);
extern int trackOneFrame(struct tagTracking *pTracking, int i, float **res, int peopleNum, int ***output);
#ifdef __cplusplus
};
#endif
#endif