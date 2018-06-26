#include "TotalWrapper.h"
#include "total.h"

/*
#include <iostream>
#include <iomanip>
#include "opencv/cv.h"
#include <math.h>
#include <fstream>
#include <iterator>
#include <algorithm>
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


using namespace cv;
using namespace std;
*/

#ifdef __cplusplus
extern "C" {
#endif
struct tagTracking
{
	Tracking MOT;
};
struct tagTracking *GetInstance(void)
{
	return new struct tagTracking;
}
void ReleaseInstance(struct tagTracking **ppInstance)
{
	delete *ppInstance;
	*ppInstance = 0;
	
}
//int trackOneFrame(struct tagTracking *pTracking, int i, Mat& res, Mat& image,vector<vector<int> > output)
//{
//	return pTracking->MOT.trackOneFrame(i, res, image, output);
//}

int trackOneFrame(struct tagTracking *pTracking, int i, float **res, int peopleNum, int ***output){
	return pTracking->MOT.trackOneFrame(i, res, peopleNum, output);
}

#ifdef __cplusplus
};
#endif
