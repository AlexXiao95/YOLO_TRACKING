//
//  total.hpp
//  MOT
//
//  Created by Hans on 4/30/16.
//  Copyright © 2016 Hans. All rights reserved.
//

#ifndef total_h
#define total_h

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
#include "Vector2.h"
//#include "Matrix.h"
#include <stdlib.h>
#include <cstdio>

#pragma comment(lib, "mclmcrrt.lib")
#pragma comment(lib, "mclmcr.lib")
#pragma comment(lib, "CFTracker.lib")

using namespace cv;
using namespace std;

/**************************** Global Variable Declararion ****************************/
//Dataset
extern string DS;

//boarders
extern const CvPoint ROI_LeftTop;
extern const CvPoint ROI_RightDown;
extern const CvPoint Border_LeftTop;
extern const CvPoint Border_RightDown;


/**********************************End Variable************************************/
/**********************************************************************************/
/*****************************Sturcture Declaration********************************/
//Reading Data Structure
//class FILEPARAM{
//public:
//    std::vector<string> *PicArray;
//    std::vector<string> *TxtArray;
//};
//Detection Structure
class PointVar{
private:
    bool delet;
public:
    Vector2<double> position;   //t is the center of the detection(width)
    double trust; //Size is the width
    int width, height,frame,id;
    bool Use;
    double* apfeature;
    int tracklet_id;
    PointVar(const PointVar& pt) 
    {
        position = pt.position;
        trust = pt.trust;
        width = pt.width;
        height = pt.height;
        frame = pt.frame;
        id = pt.id;
        Use = pt.Use;
        apfeature = pt.apfeature;
        tracklet_id = pt.tracklet_id;
    }
    void print(){
        std::cout<<"Frame:"<<frame<<" ID:"<<id<<" x:"<<position.x<<" y:"<<position.y<<" width:"<<width<<" height:"<<height<<" confidence: "<<trust<<'\n';
    }
    void drawprint(){
        std::cout<<"Frame:"<<frame<<" x:"<<position.x-width/2<<" y:"<<position.y-height/2<<" width:"<<width<<" height:"<<height<<'\n';
    }
    double* output(){
        double* returnvalue = new double[4];
        returnvalue[0] = position.x-width/2;
        returnvalue[1] = position.y-height/2;
        returnvalue[2] = width;
        returnvalue[3] = height;
        return returnvalue;
    }
    PointVar(int frame1,double x1, double y1, double width1, double height1, double trust1, int ID1){
        cv::Point Border_lt = Border_LeftTop;
        cv::Point Border_rd = Border_RightDown;
        tracklet_id = -1;
        frame=frame1;
        id=ID1;
        int row = (int)(y1 + height1/2);
        int col = (int)(x1 + width1/2);
        position.x=col;
        position.y=row;
        width=(int)width1;
        height=(int)height1;
        trust=trust1;
        Use=false;
        delet=false;
        apfeature=NULL;
        //if(width < 10 || height < 10) delet=true;
        if ((! ((int)x1 > Border_lt.x && (int)x1 < Border_rd.x && (int)y1 > Border_lt.y && (int)y1 < Border_rd.y)))
            delet=true;
    }
    PointVar(){}
    bool delete_judge(){return delet;}
    ~PointVar(){
    }
    
};
//Edgetype structure
class Edge_type_class{
public:
    int type;
    Vector2<double> pa;
    Edge_type_class(int t1){
        type = t1;
        pa.x = 0;
        pa.y = 0;
    }
    Edge_type_class(int t1, Vector2<double> p1){
        type = t1;
        pa = p1;
    }
    void print(){
        cout<<type<<"/"<<pa.x<<"/"<<pa.y<<"\t";
    }
};
//Tracklet Structure
class tracklet{
public:
    int delete_counting; //buffer detection frames
    int printbool;  // indicates whether to print in final results or not
    Vector2<double> velocity;
    std::vector <PointVar*> storage;
    std::vector<double> relation;
    std::vector<Edge_type_class> edgeType;
    std::vector<double> edgeWeights;
    double lambda1;
    double lambda2;
    int id;
    bool finished_sign;
public:
    //double current_app[1024];
    double area;
    double tracklet_weight;
    tracklet(vector<tracklet>& tracklet_pool, int& tracklet_id);
    tracklet(PointVar *target, vector<tracklet>& tracklet_pool, int& tracklet_id);
    bool operator <(const tracklet& next) const{
        return id < next.id;
    }
    //~tracklet();
};
class tmp_sort{
public:
    int index;
    int x;
    tmp_sort(int index1, int x1){
        index = index1;
        x = x1;
    }
    bool operator >(const tmp_sort& Pinfo) const{
        return x>Pinfo.x;
    }
    bool operator <(const tmp_sort& Pinfo) const{
        return x<Pinfo.x;
    }
};
class Endpoint{
public:
    int start, end, id;
    bool pre,next;
    Endpoint(int s, int e,int i, int finalFrame) {
        start = s;
        end = e;
        id = i;
        pre = false;
        next = false;
        if (s == 1) pre = true;
        if (e == finalFrame) next = true;
    }
    Endpoint(){;}
    void Setvalue(int s, int e,int i, int finalFrame) {
        start = s;
        end = e;
        id = i;
        pre = false;
        next = false;
        if (s == 1) pre = true;
        if (e == finalFrame) next = true;
    }
};
class GTnode{
public:
    int frame, x, y, width, height, id;
    GTnode(int a, int b, int c, int d, int e ,int f) {
        frame = a;
        x = b;
        y = c;
        width = d;
        height = e;
        id = f;
    }
};


/****************************End Declaration of Structure***************************/


class Tracking{
public:
    std::vector<CvScalar> sVec;

    //relationship threshold
    double edge_threshold;

    //the parameter for sigmoid function
    double width;
    double translation;

    //all hyph (dont change)
    vector<vector<int> > hyp_all;
    int hyp_all_count;
    int last_numtmp_hyp;

    //max plan
    vector<int> best_plan;

    //max hyp_plan
    double max_hyp_plan;
    vector<int> best_hyp_plan;

    //simiINDEX
    double* simiIndex;
    double* simiEdgeIndex;
    //Variable used in print results, if detections in a tracklet is less than it, not print
    int Delete_Less_Than;

    //buffer of lost in frames
    int GLOBAL_DELETE_BUFFER;
    int OUTPUT_BUFFER;
    int HYPERPLANE_GAP;
    //boundary of the neighbourhood
    int bound;

    //+1 flag
    int complete_flag;

    //motion enable
    int MOTION_ENABLE;

    //tracklet id
    int tracklet_id;

    //filter enable
    int Filtering_enable;

    int max_plan;

    std::vector<std::vector<PointVar* > > DetectionArray;

    std::vector<tracklet> tracklet_pool;

    std::vector<tracklet> all_tracklet;

    int mapping[10000];
    int mapIndex;
    Tracking(){
        GetScalar(sVec);

        tracklet_id = 1;

        edge_threshold = 5;
        width = 1;
        translation = 0;

        hyp_all_count = 0;
        last_numtmp_hyp = -1;

        max_hyp_plan = -10000;

        simiIndex = NULL;
        simiEdgeIndex = NULL;

        Delete_Less_Than = 7;
        GLOBAL_DELETE_BUFFER = 7;
        HYPERPLANE_GAP = 10;
        bound = 30;
        complete_flag = 1;
        Filtering_enable = 0;
        MOTION_ENABLE = 1;
        OUTPUT_BUFFER = 5;
        for (int i = 0; i <= 9999; i ++) {
            mapping[i] = -1;
        }
        mapIndex = 1;
        //vector<PointVar> tmDt;
        //DetectionArray.resize(100,tmDt)
    }

    void mypause(){
        cout<<"Enter to continue...\n";
        cin.get();
    }

    int ReadMat(int k, float **input, int peopleNum)
    {
        int detectionCount = peopleNum;
        int currentIndex = (int)DetectionArray.size() - 1;
        vector<PointVar*> OneFrameDetection;
        
        DetectionArray.push_back(OneFrameDetection);
        
        //printf("num: %i\n", detectionCount);
        //printf("%i %i\n",detectionCount, input.cols);

        /*
        if (detectionCount == 1 && input.cols == 1)
        {
            printf("hello\n");
            return 1;
        }
        */

        for (int i = 0; i < detectionCount; i ++)
        {
            int frame;
            double x, y, width, height, trust = -1;
            frame = k;
            //x = input.at<double>(i,0);
            //y = input.at<double>(i,1);
            //width = input.at<double>(i,2) - x;
            //height = input.at<double>(i,3) - y;

            x = input[i][0];
            y = input[i][1];
            width = input[i][2] - input[i][0];
            height = input[i][3] - input[i][1];

            if (currentIndex != -1 && DetectionArray[currentIndex].size() != 0)
            {
                int nexFrame = DetectionArray[currentIndex][0]->frame;
                if (nexFrame != frame - 1) {
                    cout << "<ERROR in ReadTxt>: frame not match!" << endl;
                    return -1;
                }
            }

            PointVar* tmp;
            tmp = new PointVar(frame, x, y, width, height, trust, i);
            DetectionArray[currentIndex + 1].push_back(tmp);

            double* featureVector = new double[2700];
            for (int j = 0 ; j < 2700; j ++)
            {
                featureVector[j] = input[i][j+4];
            }
            DetectionArray[currentIndex + 1][i]->apfeature = featureVector;
        }
        return 0;
    }

    void UpdateTrackletID(int currentFrame, vector<PointVar*>& FrameOutput)
    {
        if (currentFrame < OUTPUT_BUFFER) return;
        for (int i = 0; i < all_tracklet.size(); i ++)
        {
            if (all_tracklet[i].finished_sign == 0)
            {
                int startFrame = all_tracklet[i].storage[0]->frame;
                int endFrame = (int)all_tracklet[i].storage.size() + startFrame - 1;
                if (endFrame < startFrame + Delete_Less_Than)
                {
                    all_tracklet[i].finished_sign = 1;
                    continue;
                }
                
                int outputFrame = currentFrame - OUTPUT_BUFFER;
                int indexFrame = outputFrame - startFrame;
                
                if (indexFrame < 0 || indexFrame >= all_tracklet[i].storage.size())
                    continue;
                int tmpIndexFrame = indexFrame;
                if (mapping[all_tracklet[i].id] == -1) 
                    mapping[all_tracklet[i].id] = mapIndex++;
                all_tracklet[i].storage[indexFrame]->tracklet_id = mapping[all_tracklet[i].id];
                if (all_tracklet[i].storage.size() >= 7 && indexFrame >= 3 && indexFrame < int(all_tracklet[i].storage.size()) - 3) {
                    double xTmp = 0, yTmp = 0, wTmp = 0, hTmp = 0;
                    for (int q = -3; q <= 3 ; q ++) {
                        xTmp += all_tracklet[i].storage[indexFrame + q]->position.x;
                        yTmp += all_tracklet[i].storage[indexFrame + q]->position.y;
                        wTmp += all_tracklet[i].storage[indexFrame + q]->width;
                        hTmp += all_tracklet[i].storage[indexFrame + q]->height;
                    }
                    all_tracklet[i].storage[indexFrame]->position.x = int( xTmp / 7 );
                    all_tracklet[i].storage[indexFrame]->position.y = int( yTmp / 7 );
                    all_tracklet[i].storage[indexFrame]->width = int( wTmp / 7 );
                    all_tracklet[i].storage[indexFrame]->height = int( hTmp / 7 );
                    //cout << int( xTmp / 7 ) << int( yTmp / 7 ) << int( wTmp / 7 ) << int( hTmp / 7 );
                }
                FrameOutput.push_back(all_tracklet[i].storage[indexFrame]);
                if (indexFrame == all_tracklet[i].storage.size() - 1) {
                    all_tracklet[i].finished_sign = 1;
                }
            }
        }
        
        for (int i = 0; i < tracklet_pool.size(); i ++)
        {
            int outputFrame = currentFrame - OUTPUT_BUFFER;
            int indexFrame = outputFrame - tracklet_pool[i].storage[0]->frame;
            
            if (indexFrame >= 0 && indexFrame < tracklet_pool[i].storage.size())
            {
                if (mapping[tracklet_pool[i].id] == -1)
                    mapping[tracklet_pool[i].id] = mapIndex ++;
                tracklet_pool[i].storage[indexFrame]->tracklet_id = mapping[tracklet_pool[i].id];
                if (tracklet_pool[i].storage.size() >= 7 && indexFrame >= 3 && indexFrame < int(tracklet_pool[i].storage.size()) - 3) {
                    double xTmp = 0, yTmp = 0, wTmp = 0, hTmp = 0;
                    for (int q = -3; q <= 3 ; q ++) {
                        xTmp += tracklet_pool[i].storage[indexFrame + q]->position.x;
                        yTmp += tracklet_pool[i].storage[indexFrame + q]->position.y;
                        wTmp += tracklet_pool[i].storage[indexFrame + q]->width;
                        hTmp += tracklet_pool[i].storage[indexFrame + q]->height;
                    }
                    tracklet_pool[i].storage[indexFrame]->position.x = int( xTmp / 7 );
                    tracklet_pool[i].storage[indexFrame]->position.y = int( yTmp / 7 );
                    tracklet_pool[i].storage[indexFrame]->width = int( wTmp / 7 );
                    tracklet_pool[i].storage[indexFrame]->height = int( hTmp / 7 );
                }
                FrameOutput.push_back(tracklet_pool[i].storage[indexFrame]);
            }
        }
        
    }

    void drawOneFrame(vector<PointVar*>& TrackedDetection, Mat& src, int currentFrame, vector<CvScalar> &sVec)
    {
       std::stringstream ss;
       std::string FrameIndex;
       ss << currentFrame + 1;
       ss >> FrameIndex;    // frame index
       
       putText(src, FrameIndex, Point(100, 100), CV_FONT_HERSHEY_COMPLEX, 2, Scalar(0, 0, 255));
       int framesize = (int) TrackedDetection.size();
       for (int i=0;i<framesize;i++){
           int x,y,width,height,index;
           width = TrackedDetection[i]->width;
           height = TrackedDetection[i]->height;
           x = TrackedDetection[i]->position.x - width / 2;
           y = TrackedDetection[i]->position.y - height / 2;
           index = TrackedDetection[i]->tracklet_id;
           
           Rect r;
           r.x = x;
           r.y = y;
           r.width = width;
           r.height = height;
           
           Point t2;
           t2.x = x;
           t2.y = y;
           
           CvScalar s = sVec[index % int(sVec.size())];
           rectangle(src, r, s,1);
           
           std::stringstream ss;
           ss << index;
           std::string strIndex;
           ss >> strIndex;
           
           putText(src, strIndex, t2, CV_FONT_HERSHEY_SIMPLEX, 0.8, s);     //draw index
       }
       
       // string out_dir_new, checkdir;
       // cvWaitKey(100);
       // out_dir_new=result_img + NumToString2(i+1) + ".jpg";
       // imwrite(out_dir_new,src);
       imshow("outWindow",src);
       cvWaitKey(1);
    }

    void GetScalar(std::vector<CvScalar> &sVec){
        sVec.push_back(cvScalar(0,0,255));
        sVec.push_back(cvScalar(255,0,0));
        sVec.push_back(cvScalar(0,255,0));
        sVec.push_back(cvScalar(255,0,255));
        sVec.push_back(cvScalar(0,255,255));
        sVec.push_back(cvScalar(255,255,0));
        sVec.push_back(cvScalar(128,0,255));
        sVec.push_back(cvScalar(0,128,255));
        sVec.push_back(cvScalar(128,255,0));
        sVec.push_back(cvScalar(0,255,128));
        sVec.push_back(cvScalar(255,128,0));
        sVec.push_back(cvScalar(255,0,128));
        sVec.push_back(cvScalar(128,128,255));
        sVec.push_back(cvScalar(128,255,128));
        sVec.push_back(cvScalar(255,128,128));
    }

    void global_push(tracklet &tmp){
        for (int i = 0; i < (int)tracklet_pool.size(); ++i)
        {
            //It is reasonable to assume there is no reltion between two targets without evidence
            tracklet_pool[i].relation.push_back(0);
            Edge_type_class tmp1(-1);
            tracklet_pool[i].edgeType.push_back(tmp1);
            //cout<<tracklet_pool.size()<<"\t"<<tracklet_pool[i].edgeType.size()<<endl;
            tracklet_pool[i].edgeWeights.push_back(1);
        }

        tracklet_pool.push_back(tmp);
    }

    int global_delete(int k){
        if (++tracklet_pool[k].delete_counting > GLOBAL_DELETE_BUFFER ){
            all_tracklet.push_back(tracklet_pool[k]);
            for (int i = 0; i < (int)tracklet_pool.size(); ++i){
                if ((int)tracklet_pool.size()==0) break;
                vector<double>:: iterator iter=tracklet_pool[i].relation.begin();
                iter += k;
                tracklet_pool[i].relation.erase(iter);
                
                vector<Edge_type_class>:: iterator iter1=tracklet_pool[i].edgeType.begin();
                iter1 += k;
                tracklet_pool[i].edgeType.erase(iter1);
                
                vector<double>:: iterator iter2=tracklet_pool[i].edgeWeights.begin();
                iter2 += k;
                tracklet_pool[i].edgeWeights.erase(iter2);
            }
            vector<tracklet>::iterator iter3=tracklet_pool.begin();
            iter3+=k;
            tracklet_pool.erase(iter3);
            return 1;
        }
        else return 0;
    }

    void generate_best_plan(vector<vector<int> > &candidate,vector<int> &plan,vector<int> one_to_one,int trackletID){
        generate_all_possibility2(candidate, 0, plan, one_to_one, trackletID);
    }

    void generate_all_possibility2(vector<vector<int> > &candidate,
                                   int pos, vector<int> &plan, vector<int> one_to_one, int trackletID)
    {
        //bool flag=false;
        if (pos>=(int)candidate.size()) {
            double obj_current = compute_gain(DetectionArray[trackletID],plan, candidate,trackletID,simiIndex);
            if (obj_current > max_plan){
                best_plan = plan;
                max_plan = obj_current;
            }
            //hyp_all.push_back(plan);
            return;
        }

        for (int i = 0; i < candidate[pos].size(); i ++) {
            if (candidate[pos][i] == -1 || one_to_one[candidate[pos][i]] != 0) {
                plan[pos] = -1;
                generate_all_possibility2(candidate, pos + 1, plan, one_to_one, trackletID);
            }
            else if (one_to_one[candidate[pos][i]] == 0){
                plan[pos] = candidate[pos][i];
                one_to_one[candidate[pos][i]]=1;
                generate_all_possibility2(candidate, pos+1, plan, one_to_one,trackletID);
                one_to_one[candidate[pos][i]]=0;
            }
            else {
                mypause();
            }
        }
        
        return;
    }

    double compute_gain(std::vector<PointVar*> &detection,vector<int> &plan,
                    vector<vector<int> > &candidate, int frame,double* simiIndex)
    {
        double gain = 0;
        int size = (int)tracklet_pool.size();
        double lambda;
        PointVar* target_tmp, *target_tmp2, *target1_track2, *target2_track2;
        PointVar* target1,*target2,*target3,*target4;
        
        //compute the node gain
        for (int i = 0; i < size; ++i)
        {
            if (plan[i]!=-1){
                target_tmp = tracklet_pool[i].storage.back();
                target_tmp2 = detection[plan[i]];
                double tmp1 = target_tmp->position.x;
                double tmp2 = target_tmp2->position.x;
                
                double debug_gain = simiIndex[i*(int(detection.size()))+plan[i]];
                gain+=simiIndex[i*(int(detection.size()))+plan[i]];
                
                double tmp_gain = 0;
                int tmp_counting = 0;
                //frame is the frame before wrong, target_tmp is the previous, target_tmp2 is the present
                // if (frame == 951 && target_tmp->position_id == 1 && target_tmp2->position_id == 1){
                //     cout<<"1"<<endl;
                // }
                //Edge similarity
                for (int j = 0 ; j < i; j ++) {
                    if (tracklet_pool[i].edgeType[j].type != -1 && plan[j] != -1) {
                        Edge_type_class type = tracklet_pool[i].edgeType[j];
                        target1_track2=tracklet_pool[j].storage.back();
                        target2_track2 = detection[plan[j]];
                        
                        //i large, j small
                        if (tracklet_pool[i].id == 270 && tracklet_pool[j].id == 265 && target1_track2->frame == 951){// && target_tmp2->position_id == 3 && target2_track2->position_id == 4 ) {
                            cout<< target2_track2->frame;
                        }
                        
                        double tmp_return = compute_distance_variation_version2(target_tmp, target_tmp2, target1_track2,target2_track2, type);
                        if (tmp_return != -2) {
                            tmp_gain += tmp_return * tracklet_pool[i].edgeWeights[j];
                            tmp_counting ++;
                        }
                    }
                }
                if (tmp_counting != 0){
                   // cout << tmp_gain/tmp_counting << " " << tmp_gain << " " << tmp_counting << endl;
                    tmp_counting = 1;
                    gain += tmp_gain / tmp_counting;
                }
            }
        }     
        //cout<<gain<<endl;

        return gain;
    }

    double compute_distance_variation_version2(const PointVar *tracklet1_a,const PointVar *tracklet1_b,const PointVar *tracklet2_a,const PointVar *tracklet2_b, Edge_type_class Edge)
    {
        double angle_difference, length_difference;
        double distance1,distance2;
        Vector2<double> previous = tracklet1_a->position - tracklet2_a->position;
        Vector2<double> current = tracklet1_b->position - tracklet2_b->position;
        Vector2<double> various = Edge.pa;
        if (Edge.type == 2 || Edge.type == 3){
            previous = previous + various;
        }
        angle_difference = Vector2<double>::dotProduct(previous,current)/(previous.absolute()*current.absolute());
        if (angle_difference>1){
            angle_difference = 0.9999;
        }
        double angle = acos(angle_difference);
        length_difference = current.absolute() - previous.absolute();
        //angle_difference = 1 / 2.0 * angle_difference + 1 / 2.0;
        //1 group, 0 occlusion, 2 getting closer, 3 getting away
        double tmp_gain;
        double denu = (tracklet1_a->width + tracklet2_a->width)/2;
        if (Edge.type == 1){
            tmp_gain = exp(-abs(length_difference)/(denu)) * exp(-abs(angle)*3);
        }
        else if (Edge.type == 0){
            tmp_gain = -2;
            return tmp_gain;
        }
        else if (Edge.type == 2){
            if (length_difference <= 0){
                tmp_gain = exp(-abs(length_difference)/(denu)) * exp(-abs(angle)*3);
            }
            else
                tmp_gain = exp(-abs(length_difference)/(denu)) * exp(-abs(angle)*3);
        }
        else if (Edge.type == 3){
            if (length_difference >= 0)
                tmp_gain = exp(-abs(length_difference)/(denu)) * exp(-abs(angle)*3);
            else
                tmp_gain = exp(-abs(length_difference)/(denu)) * exp(-abs(angle)*3);
        }
        
        if (tmp_gain < 0.1) {
            return -1;
        }
        return tmp_gain;
    }

    double correlation_node(tracklet& track, PointVar* candidate)
    {
        int size;
        PointVar* tmp;
        double simi_motion,result,simi_app;

        size=(int)track.storage.size();
        tmp=track.storage[size-1];
        
        //simi_motion=correlation_motion(track,candidate);
        
        simi_app=CoAppearance(track, candidate);
        
        if (MOTION_ENABLE == 1){
            result = track.lambda1 * simi_motion + track.lambda2 * simi_app;
        }
        else if (MOTION_ENABLE ==0){
            result = simi_app;
        }

        return result;
    }

    double correlation_motion(tracklet *track,PointVar *candidate)
    {
        if ((int)track->storage.size() < 4) return 0;
        double result;
        Vector2<double> vector1,vector2,velocity,velocity_dif;
        PointVar* tmp;
        int scale = 0;
        velocity=track->velocity;
        scale=(int)track->storage.size();
        tmp=track->storage[scale-3];
        vector1=tmp->position;
        Vector2<double> p1 = track->storage[scale-4]->position;
        Vector2<double> p2 = track->storage[scale-2]->position;
        
        p1 = vector1 + (p1 - vector1)/(track->storage[scale-3]->frame - track->storage[scale-4]->frame);
        p2 = vector1 + (p2 - vector1)/(track->storage[scale-2]->frame - track->storage[scale-3]->frame);
        vector1 = (p1 + p2 + vector1)/3.0;
        vector2 = candidate->position;
        vector2 = (vector2-vector1)/(candidate->frame-tmp->frame);
        velocity_dif = vector2 - velocity;
        if (velocity.absolute() == 0 || vector2.absolute() == 0)
            result = 0;
        else
        {
            double angle_dif = Vector2<double>::dotProduct(velocity,vector2)/(velocity.absolute()*vector2.absolute());
            double length_difference = abs(velocity.absolute() - vector2.absolute());
            double denu = (track->storage[scale-1]->width + candidate->width)/2;
            double angle_difference = acos(angle_dif);
            result = exp(-abs(angle_difference)*3) * exp(-abs(length_difference)/(denu));
        }
        result = result * track->tracklet_weight;
        return result;
    }

    double CoAppearance(tracklet& pre, PointVar* next)
    {
        double totalsum = 0;
        double sum1 = 0,sum2 = 0;
        double *preap, *nextap;
        int trackletsize = (int)pre.storage.size();
        preap = pre.storage[trackletsize-1]->apfeature;

        nextap = next->apfeature;
        for (int i=0 ; i < 2700 ; i ++){
            totalsum += preap[i] * nextap[i];
            sum1 += preap[i]*preap[i];
            sum2 += nextap[i]*nextap[i];
            //totalsum += (preap[i] - nextap[i]) * (preap[i] - nextap[i]);
        }
        //totalsum /= 1024;
        //cout << "denu: " << sum1*sum2 << endl;
        totalsum /= sqrt(sum1*sum2);
        double angle_difference = acos(totalsum);
        double tmp_return = exp(-abs(angle_difference)*3);

        tmp_return *= pre.tracklet_weight;
        return tmp_return;
    }

    void add_P2T(tracklet *track, PointVar *newdetection){
        if (track->storage.size() == 0) {
            track->storage.push_back(newdetection);
            update_velocity(track);
            return;
        }
        int currentFrame = newdetection->frame;
        
        int lastFrame = track->storage[track->storage.size() - 1]->frame;
        if (lastFrame + 1 == currentFrame) {
            track->storage.push_back(newdetection);
            update_velocity(track);
        }  
        else {
            int frame1 = track->storage[track->storage.size() - 1]->frame;
            int frame2 = newdetection->frame;
            double width1 = track->storage[track->storage.size() - 1]->width;
            double height1 = track->storage[track->storage.size() - 1]->height;
            double x1 = track->storage[track->storage.size() - 1]->position.x - width1/2;
            double y1 = track->storage[track->storage.size() - 1]->position.y - height1/2;
            double width2 = newdetection->width;
            double height2 = newdetection->height;
            double x2 = newdetection->position.x - width2/2;
            double y2 = newdetection->position.y - height2/2;
            for (int q = 0; q < frame2-frame1-1; q++) {
                PointVar* tmp1 = new PointVar(frame1+q+1,(x2-x1)/(frame2-frame1)*(q+1)+x1,(y2-y1)/(frame2-frame1)*(q+1)+y1,(width2-width1)/(frame2-frame1)*(q+1)+width1,(height2-height1)/(frame2-frame1)*(q+1)+height1,-1,-1);
                track->storage.push_back(tmp1);
            }
            track->storage.push_back(newdetection);
            update_velocity(track);
        }
    }

    void update_edge_node_weight(std::vector<tracklet> &tracklet_pool,std::vector<PointVar*> &detection){
        int size=(int)tracklet_pool.size();
        PointVar* target_tmp, *target_tmp2, *target1_track2, *target2_track2;
        
        //compute the node gain
        for (int i = 0; i < size; ++i){
            if (best_plan[i] == -1){
                tracklet_pool[i].tracklet_weight /= 1.1;
                for (int j = 0; j < i; j ++ ) {
                    if (tracklet_pool[i].edgeType[j].type != -1){
                        tracklet_pool[i].edgeWeights[j] /= 1.1;
                        tracklet_pool[j].edgeWeights[i] /= 1.1;
                    }
                }
            }
            else if (best_plan[i]!= -1){
                target_tmp = tracklet_pool[i].storage.back();
                target_tmp2 = detection[best_plan[i]];
                
                //Node similarity calculation (using index to speed up)!
                double appearance_similaruty = simiIndex[i*(int(detection.size()))+best_plan[i]];
                //cout<<"appearance_similaruty: "<<appearance_similaruty<<endl;
                tracklet_pool[i].tracklet_weight = sigmoid(tracklet_pool[i].tracklet_weight - 1 + (appearance_similaruty - 0.2) * 5, 0, -1,2);
                //Edge similarity
                for (int j = 0 ; j < i; j ++) {
                    if (tracklet_pool[i].edgeType[j].type != -1 && best_plan[j] != -1) {
                        int type = tracklet_pool[i].edgeType[j].type;
                        target1_track2 = tracklet_pool[j].storage.back();
                        target2_track2 = detection[best_plan[j]];
                        double edge_similarity = compute_distance_variation_version2(target_tmp, target_tmp2, target1_track2,target2_track2, type);
                        //cout<<edge_similarity;
                        if (edge_similarity != -2){
                            tracklet_pool[i].edgeWeights[j] = sigmoid(tracklet_pool[i].edgeWeights[j] - 1 + (edge_similarity - 0.5) * 5, 0, -1,2);
                            tracklet_pool[j].edgeWeights[i] = tracklet_pool[i].edgeWeights[j];
                            //cout<<"edge_similarity: "<<edge_similarity<<endl;
                        }
                    }
                    else if (tracklet_pool[i].edgeType[j].type != -1 && best_plan[j] == -1) {
                        tracklet_pool[i].edgeWeights[j] /= 1.1;
                        tracklet_pool[j].edgeWeights[i] /= 1.1;
                    }
                }
            }
        }
    }

    void update_relation(std::vector<tracklet> &tracklet_pool){
        //need to check
        PointVar* target1;
        PointVar* target2;
        PointVar* target3;
        PointVar* target4;
        
        int num1,num2;
        
        double correlation=0;
        
        int scale=(int)tracklet_pool.size();
        for (int i = 0; i < scale; ++i){
            for (int j = 0; j < i; ++j){
                num1=(int)tracklet_pool[i].storage.size();
                num2=(int)tracklet_pool[j].storage.size();
                if (num1>=2 && num2>=2){
                    target1=tracklet_pool[i].storage[num1-2];
                    target2=tracklet_pool[i].storage[num1-1];
                    target3=tracklet_pool[j].storage[num2-2];
                    target4=tracklet_pool[j].storage[num2-1];
                    correlation=compute_distance_variation(target1,target2,target3,target4);
                    
                    if (correlation>=edge_threshold){
                        tracklet_pool[i].relation[j]-=1;
                        tracklet_pool[j].relation[i]-=1;
                    }
                    else{
                        tracklet_pool[i].relation[j]+=1;
                        tracklet_pool[j].relation[i]+=1;
                    }
                }
                else
                    return;
            }
        }
        return;
    }
	double compute_distance_variation(const PointVar *tracklet1_a,const PointVar *tracklet1_b,const PointVar *tracklet2_a,const PointVar *tracklet2_b){
		double difference;
		double distance1,distance2;
		Vector2<double> previous=tracklet1_a->position-tracklet2_a->position;
		Vector2<double> current=tracklet1_b->position-tracklet2_b->position;
		difference = Vector2<double>::dotProduct(previous,current)/(previous.absolute()*current.absolute());
	//    cout<<difference<<endl;
		return difference;
	}
    void update_edgetype(std::vector<tracklet> &tracklet_pool,int frame){
        int scale=(int)tracklet_pool.size();
        int num1, num2, frame1, frame2;
        if (scale == 0) return;
        for (int i = 0; i <= scale - 1; i++){
            for (int j = 0; j <= i - 1; j++){
                num1=(int)tracklet_pool[i].storage.size();
                num2=(int)tracklet_pool[j].storage.size();
                Vector2<double> speed1, speed2, location1, location2;
                double width1, height1, width2, height2;
                speed1 = tracklet_pool[i].velocity;
                speed2 = tracklet_pool[j].velocity;
                frame1 = tracklet_pool[i].storage[num1-1]->frame;
                frame2 = tracklet_pool[j].storage[num2-1]->frame;
                location1 = tracklet_pool[i].storage[num1-1]->position;
                location2 = tracklet_pool[j].storage[num2-1]->position;
                width1 = tracklet_pool[i].storage[num1-1]->width;
                height1 = tracklet_pool[i].storage[num1-1]->height;
                width2 = tracklet_pool[j].storage[num2-1]->width;
                height2 = tracklet_pool[j].storage[num2-1]->height;
                if (tracklet_pool[i].velocity.absolute() == 0 || tracklet_pool[j].velocity.absolute() == 0){
                    if (abs(location1.x - location2.x) < (width1 + width2)*1 && abs(location1.y - location2.y) <(width1+width2)*1){
                        tracklet_pool[i].edgeType[j].type = 1;
                        tracklet_pool[j].edgeType[i].type = 1;
                        continue;
                    }
                    tracklet_pool[i].edgeType[j].type = -1;
                    tracklet_pool[j].edgeType[i].type = -1;
                    continue;
                }

                if (num1 >= 2 && num1>=2){
                    double speed_variation = Vector2<double>::dotProduct(speed1, speed2)/(speed2.absolute() * speed1.absolute());
                    double speed_abs_va = abs(speed1.absolute() / speed2.absolute());
                    Vector2<double> relative_velcity = speed1 - speed2;
                    Vector2<double> relative_position = location2 - location1;
                    double oreantation = Vector2<double>::dotProduct(relative_velcity, relative_position)/(relative_velcity.absolute() * relative_position.absolute());
                    // -1 not connected, 1 group, 0 occlusion, 2 getting closer, 3 getting away
                    if (abs(location1.x - location2.x) < (width1 + width2) * 1.5 && abs(location1.y - location2.y) <(width1+width2)*1.5 && speed_variation >= 0.75 && speed_abs_va <= 1.5 && speed_abs_va >= 0.67) {
                        tracklet_pool[i].edgeType[j].type = 1;
                        tracklet_pool[j].edgeType[i].type = 1;
                    }
                    else if(abs(location1.x - location2.x) < (width1 + width2)/2 && abs(location1.y - location2.y) <(width1+width2)/2 && tracklet_pool[i].edgeType[j].type == 2){
                        tracklet_pool[i].edgeType[j].type = 1;
                        tracklet_pool[j].edgeType[i].type = 1;
                    }
                    else if(abs(location1.x - location2.x) < (width1 + width2) * 3 && abs(location1.y - location2.y) <(width1 + width2) * 3 /*&& speed_variation <= -0.8*/ && oreantation >= 0.5 && tracklet_pool[i].edgeType[j].type != 3){
                        tracklet_pool[i].edgeType[j].type = 2;
                        tracklet_pool[j].edgeType[i].type = 2;
                        tracklet_pool[i].edgeType[j].pa = speed1 * (frame - frame1 + 1) - speed2 * (frame - frame2 + 1);
                        tracklet_pool[j].edgeType[i].pa = speed2 * (frame - frame2 + 1) - speed1 * (frame - frame1 + 1);
                    }
                    else if(abs(location1.x - location2.x) < (width1 + width2) * 3 && abs(location1.y - location2.y) <(width1 + width2) * 3 /*&& speed_variation <= -0.8*/ && oreantation <= -0.5){
                        tracklet_pool[i].edgeType[j].type = 3;
                        tracklet_pool[j].edgeType[i].type = 3;
                        tracklet_pool[i].edgeType[j].pa = speed1 * (frame - frame1 + 1) - speed2 * (frame - frame2 + 1);
                        tracklet_pool[j].edgeType[i].pa = speed2 * (frame - frame2 + 1) - speed1 * (frame - frame1 + 1);
                    }
                    else if(abs(location1.x - location2.x) >= (width1 + width2) * 2){
                        tracklet_pool[i].edgeType[j].type = -1;
                        tracklet_pool[j].edgeType[i].type = -1;
                    }
                }
                else {
                    tracklet_pool[i].edgeType[j].type = -1;
                    tracklet_pool[j].edgeType[i].type = -1;
                }
            }
        }
    }

    void update_velocity(tracklet *track){
        int size=(int)track->storage.size();
        if (size <= 6) return;
        if (size <= 6) {
            track->velocity = (track->storage[size-1]->position - track->storage[0]->position) / (track->storage[size-1]->frame - track->storage[0]->frame);
            return;
        }
        //cout<<"track->storage[size-1]->frame:"<<track->storage[size-1]->frame<<'\n';
        //cout<<"track->storage[size-2]->frame:"<<track->storage[size-2]->frame<<'\n';
        Vector2<double> position_queue[6];
        position_queue[0] = track->storage[size-1]->position;//1
        position_queue[1] = track->storage[size-2]->position;//2
        position_queue[2] = track->storage[size-3]->position;//3
        position_queue[3] = track->storage[size-4]->position;//4
        position_queue[4] = track->storage[size-5]->position;//5
        position_queue[5] = track->storage[size-6]->position;//6
        
        position_queue[0] = position_queue[1] + (position_queue[0] - position_queue[1])/(track->storage[size-1]->frame - track->storage[size-2]->frame);
        position_queue[2] = position_queue[1] + (position_queue[2] - position_queue[1])/(track->storage[size-2]->frame - track->storage[size-3]->frame);
        position_queue[3] = position_queue[4] + (position_queue[3] - position_queue[4])/(track->storage[size-3]->frame - track->storage[size-4]->frame);
        position_queue[5] = position_queue[4] + (position_queue[5] - position_queue[4])/(track->storage[size-4]->frame - track->storage[size-5]->frame);
        
        
        track->velocity=((position_queue[0]+position_queue[1]+position_queue[2])/3.0-(position_queue[3]+position_queue[4]+position_queue[5])/3.0)/(track->storage[size-2]->frame-track->storage[size-5]->frame);
        //cout<<track->velocity<<endl;
        return;
    }

	double sigmoid(double x,double a,double b,double c){
	    double denominator;
	    denominator=1+exp((x-a)/b);
	    return c/denominator;
	}

    //int trackOneFrame(int i, Mat& res, Mat& image,vector<vector<int> > output){
    int trackOneFrame(int i, float ** res, int peopleNum, int ***outputAddr){
        ReadMat(i, res, peopleNum);
        vector<int> optimal_hype;
        int target_num = (int)DetectionArray[i].size();
        int tracklet_num = (int)tracklet_pool.size();
        bool *target_link_flag;
        int difference = 0;
        vector<int> plan;
        vector<int> one_to_one;
        printf("num %i %i\n",target_num, tracklet_num);
        //if (target_num == 0) continue;
        target_link_flag=new bool[target_num];
        for (int p = 0; p < target_num; ++p)
        {
            target_link_flag[p]=0;
        }
        vector<vector<int> > candidate;
        candidate.assign(tracklet_num, vector<int>(0,0));
        cout << i + 1 << endl;
        double all_can=1;
        //generate candidates that are in the window
        if (complete_flag == 1) {
            for (int m = 0; m < tracklet_num; m ++) {
                candidate[m].push_back(-1);
            }
        }
        for (int m=0; m<tracklet_num; m++) {
            PointVar *tmp=tracklet_pool[m].storage.back();
            int dele_count = tracklet_pool[m].delete_counting;
            
            for (int n=0; n<target_num; n++) {
                double area1 = double(DetectionArray[i][n]->width * DetectionArray[i][n]->height);
                double area2 = tmp->width * tmp->height;

                if ((abs(DetectionArray[i][n]->position.x - tmp->position.x) < tmp->width * 0.5) && (abs(DetectionArray[i][n]->position.y-tmp->position.y)<tmp->width * 0.5) && (area1/area2>0.5 && area1/area2<2))
                {
                    candidate[m].push_back(n);
                }
            }
            
            all_can *= (candidate[m].size());
        }
        cout << all_can << " " << tracklet_num << " " << target_num << endl << endl;
        plan.assign(tracklet_num,-1);
        one_to_one.assign(target_num,0);
        //hyp_all.assign(0,vector<int>(tracklet_num,-1));
        
        simiIndex = new double [target_num * tracklet_num];
        for (int t1 = 0; t1 < tracklet_num; t1 ++){
            for (int t2 = 0; t2 < target_num; t2 ++){
                simiIndex [ t1 * target_num + t2] = correlation_node(tracklet_pool[t1], DetectionArray[i][t2]);
            }
        }

        max_plan = -10000;
        generate_best_plan(candidate,plan,one_to_one,i);
        optimal_hype = best_plan;
        cout << "plan: ";
        for (int l =0 ; l < optimal_hype.size(); l ++) {
            cout << optimal_hype[l] << " ";
        }
        cout << "\n";
        update_edge_node_weight(tracklet_pool, DetectionArray[i]);
        delete [] simiIndex;
        for (int k = 0; k < tracklet_num; ++k)
        {
            if (optimal_hype[k]!=-1){
                target_link_flag[optimal_hype[k]]=1;
                add_P2T(&(tracklet_pool[k-difference]), DetectionArray[i][optimal_hype[k]] );
                tracklet_pool[k-difference].delete_counting = 0;
            }
            else{
                int tmp=global_delete(k-difference);
                if (tmp == 1)
                    difference++;
            }
        }
        update_relation(tracklet_pool);
        
        for (int q = 0; q < target_num; ++q)
        {
            if (target_link_flag[q]==0)
            {
                tracklet tmp(DetectionArray[i][q], tracklet_pool, tracklet_id);
                global_push(tmp);
            }
        }
        delete []target_link_flag;
        
        update_edgetype(tracklet_pool,i+1);
        vector<PointVar *> FrameOutput;
        if (DetectionArray.size() <= 10)
            return 0;
        UpdateTrackletID(i + 1, FrameOutput);
        //printf("Start tracking\n");
        int **output;
        //output = new int *[FrameOutput.size()];
        output = (int **)calloc((int)FrameOutput.size(), sizeof(int *));
        for (int q = 0; q < FrameOutput.size(); q ++)
        {
            // vector<int> item;
            // item.push_back(FrameOutput[q]->frame);
            // item.push_back(FrameOutput[q]->position[0]);
            // item.push_back(FrameOutput[q]->position[1]);
            // item.push_back(FrameOutput[q]->width);
            // item.push_back(FrameOutput[q]->height);
            // item.push_back(FrameOutput[q]->tracklet_id);
            // output.push_back(item);
            
            //output[q] = new int[6];
            output[q] = (int *)calloc(6, sizeof(int *));
            output[q][0] = FrameOutput[q]->frame;
            output[q][1] = FrameOutput[q]->position.x - FrameOutput[q]->width/2;
            output[q][2] = FrameOutput[q]->position.y - FrameOutput[q]->height/2;
            output[q][3] = FrameOutput[q]->position.x + FrameOutput[q]->width/2;
            output[q][4] = FrameOutput[q]->position.y + FrameOutput[q]->height/2;
            output[q][5] = FrameOutput[q]->tracklet_id;
            cout << FrameOutput[q]->frame << "\t" << FrameOutput[q]->position << "\t" << FrameOutput[q]->width << "\t" <<FrameOutput[q]->height << "\t" << FrameOutput[q]->tracklet_id << endl;
        }
        //cout << "FrameOutput Size: " << FrameOutput.size() << endl;
        (*outputAddr)=output;
        
        /*
        std::stringstream ss;
        if (i >= OUTPUT_BUFFER)
        {
            ss << i + 1 - OUTPUT_BUFFER;
            string impath;
            ss >> impath;
            impath = "./detection/" + impath + ".jpg";
            //Mat image = imread(impath);
            //drawOneFrame(FrameOutput, image, i + 1 - OUTPUT_BUFFER, sVec);
            //imshow("out", image);
		    //cvWaitKey(1);
		    //mypause();
		   return 1;
        }
        */

        return (int)FrameOutput.size();
    }
};



#endif /* total_hpp */
