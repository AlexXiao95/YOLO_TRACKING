#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "getHogFromCpp.h"

using namespace std;
using namespace cv;

void getHogFromCpp(IplImage *Ipl, float *feat)
{
	Mat imageMat = cvarrToMat(Ipl, true);
	HOGDescriptor *hog = new HOGDescriptor(cvSize(48,128), cvSize(16,16), cvSize(8,8), cvSize(8,8), 9);
	vector<float> descriptors;
	hog->compute(imageMat, descriptors, Size(2,2), Size(0,0));
	//cout << "feature size: " << descriptors.size() << endl; 
	for(int i = 0; i < 2700; ++i){
		feat[i] = descriptors[i];
	}
} 