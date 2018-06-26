//
//  total.cpp
//  MOT
//
//  Created by Hans on 4/30/16.
//  Copyright © 2016 Hans. All rights reserved.
//

#include "total.h"

// ************ Dataset Predefine *********** //
//#define ADL_Rundle_1
//#define ADL_Rundle_3
//#define AVG_TownCentre
//#define KITTI_16
//#define KITTI_19
//#define ETH_Crossing
//#define ETH_Jelmoli
//#define ETH_Linthescher
//#define Venice_1
//#define PETS09_S2L2
//#define TUD_Crossing
//#define canteenres
#define tianmuluv5
//#define zhuangyuange02
// ***************** End **************** //


#if defined(AVG_TownCentre)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(1920,1080);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(1920,1080);

string DS = "AVG-TownCentre";
#endif

#if defined(ADL_Rundle_1)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(1920,1080);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(1920,1080);

string DS = "ADL-Rundle-1";
#endif

#if defined(ADL_Rundle_3)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(1920,1080);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(1920,1080);

string DS = "ADL-Rundle-3";
#endif

#if defined(Venice_1)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(1920,1080);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(1920,1080);

string DS = "Venice-1";
#endif

#if defined(KITTI_19)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(1238,374);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(1238,374);

string DS = "KITTI-19";
#endif

#if defined(KITTI_16)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(1238,374);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(1238,374);

string DS = "KITTI-16";
#endif

#if defined(ETH_Jemoli)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(640,480);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(640,480);

string DS = "ETH-Jemoli";
#endif

#if defined(ETH_Crossing)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(640,480);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(640,480);

string DS = "ETH-Crossing";
#endif

#if defined(ETH_Linthescher)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(640,480);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(640,480);

string DS = "ETH-Linthescher";
#endif

#if defined(TUD_Crossing)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(640,480);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(640,480);

string DS = "TUD-Crossing";
#endif

#if defined(tianmuluv5)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(640,480);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(640,480);

string DS = "tianmuluv5";
#endif

#if defined(canteenres)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(640,480);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(640,480);

string DS = "canteenres";
#endif

#if defined(zhuangyuange02)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(640,480);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(640,480);

string DS = "zhuangyuange02";
#endif

#if defined(PETS09_S2L2)
const CvPoint ROI_LeftTop = cvPoint(0,0);
const CvPoint ROI_RightDown = cvPoint(768,576);

const CvPoint Border_LeftTop = cvPoint(0,0);
const CvPoint Border_RightDown = cvPoint(768,576);

string DS = "PETS09-S2L2";
#endif


tracklet::tracklet(vector<tracklet>& tracklet_pool, int& tracklet_id){
    velocity=0;lambda1=0.5;lambda2=0.5;delete_counting=0;printbool=1;tracklet_weight=1;
    relation.resize((int)(tracklet_pool.size())+1,0);
    Edge_type_class tmp(-1);
    edgeType.resize(int(tracklet_pool.size())+1,tmp);
    edgeWeights.resize(int(tracklet_pool.size())+1,1);
    id = tracklet_id ++;
    tracklet_weight = 1;
    finished_sign = 0;
}

tracklet::tracklet(PointVar *target, vector<tracklet>& tracklet_pool, int& tracklet_id):velocity(0),lambda1(0.5),lambda2(0.5)
{
    storage.push_back(target);
    delete_counting=0;
    printbool=1;
    relation.resize(int(tracklet_pool.size())+1,0);
    Edge_type_class tmp(-1);
    edgeType.resize(int(tracklet_pool.size())+1,tmp);
    edgeWeights.resize(int(tracklet_pool.size())+1,1);
    area = double(target->height * target->width);
    tracklet_weight = 1;
    id = tracklet_id ++;
    finished_sign = 0;
}

