#ifndef UTILITY_H
#define UTILITY_H

#include <signal.h>
#include <thread>
#include <mutex>
#include <queue>
#include <time.h>
#include <chrono>
#include <utility>
#include <cmath>

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include "planning_image_drawing/set_img_flag.h"
#include "planning_image_drawing/drawing_type.h"
#include "planning_image_drawing/start_calculate_flag.h"
#include "planning_image_drawing/start_drawing_flag.h"

// OpenCV
#include <opencv2/opencv.hpp>
class Path
{
private:
    std::vector<cv::Point2f> _mvPath2f;
    nav_msgs::Path _mPath; // for pub path
public:
    Path(){}
    void insertPoint(cv::Point2f point)
    {
        _mvPath2f.push_back(point);
    }
    void insertPath(std::vector<cv::Point2f> rawPath)
    {
        _mvPath2f = rawPath;
    }
    void createPath(void)
    {
        // re-arrange
        std::vector<cv::Point2f> tmpPath;
        cv::Point2f prevPoint(-1,-1);
        tmpPath.push_back(prevPoint);
        for(int i = 0; i < _mvPath2f.size(); i++)
        {
            cv::Point2f currPoint= _mvPath2f[i];
            if(prevPoint.x == currPoint.x && prevPoint.y == currPoint.y)continue;
            tmpPath.push_back(_mvPath2f[i]);
            prevPoint = currPoint;
        }
        _mvPath2f = tmpPath;
    }
    void createROSMsg(void)
    {
        // ros msg
        _mPath.header.stamp = ros::Time::now();
        geometry_msgs::PoseStamped tmp;
        for(int i = 0; i < _mvPath2f.size(); i++)
        {
            tmp.pose.position.x = _mvPath2f[i].x;
            tmp.pose.position.y = _mvPath2f[i].y;
            _mPath.poses.push_back(tmp);
        }
    }
    void simplifyPath0(void)
    {
        int idx = 0;
        int eNum = 0;
        for(int i = 0; i < _mvPath2f.size() - eNum; i++)
        {
            if(i-2 < 0 || i + 2 >=  _mvPath2f.size() - eNum)continue;
            if(_mvPath2f[i].x < 0 || _mvPath2f[i].y < 0 ||
                _mvPath2f[i-2].x < 0 || _mvPath2f[i-2].y < 0 ||
                _mvPath2f[i+2].x < 0 || _mvPath2f[i+2].y < 0)continue;
            
            if(idx%2 == 0)_mvPath2f.erase(_mvPath2f.begin() + i);
            idx++;
        }
    }
    void simplifyPath(void)
    {
        double THD = 0;
        std::vector<int> vEraseIdx;
        for(int i = 0; i < _mvPath2f.size(); i++)
        {
            if(i-2 < 0 || i + 2 >=  _mvPath2f.size())continue;
            if(_mvPath2f[i].x < 0 || _mvPath2f[i].y < 0 ||
                // _mvPath2f[i-2].x < 0 || _mvPath2f[i-2].y < 0 ||
                // _mvPath2f[i+2].x < 0 || _mvPath2f[i+2].y < 0 ||
                _mvPath2f[i-1].x < 0 || _mvPath2f[i-1].y < 0 ||
                _mvPath2f[i+1].x < 0 || _mvPath2f[i+1].y < 0)continue;
            
            // Get Slope
            double X1 = _mvPath2f[i].x - _mvPath2f[i-1].x;
            double Y1 = _mvPath2f[i].y - _mvPath2f[i-1].y;
            double X2 = _mvPath2f[i+1].x - _mvPath2f[i].x;
            double Y2 = _mvPath2f[i+1].y - _mvPath2f[i].y;

            bool infinity1 = false; 
            bool infinity2 = false;
            double slope1, slope2;
            if(X1 < 0.01) infinity1 = true;
            else slope1 = Y1/X1;

            if(X2 < 0.01) infinity2 = true;
            else slope2 = Y2/X2;

            // Insert Erase Idx
            if(infinity1 && infinity2)vEraseIdx.push_back(i);
            else if(infinity1 || infinity2)
            {
            //     double angle = 90;
            //     if(infinity1)
            //     {
            //         double angle = 90 - fabs(atan(slope2))*(180.0/M_PI);
            //     }
            //     else if(infinity2)
            //     {
            //         double angle = 90 - fabs(atan(slope1))*(180.0/M_PI);
            //     }
            //     else assert(false);
            //     angle = fabs(angle);
            //     if(angle <= THD)vEraseIdx.push_back(i);
            }
            else
            {
            //     double angle = fabs(atan(slope2) - atan(slope1))*(180.0/M_PI);
            //     angle = fmod(angle,360.0);
            //     if(angle > 180) angle = 180 - angle;
            //     angle = fabs(angle);
            //     if(angle <= THD)vEraseIdx.push_back(i);
            }

        }
        // Erase Point
        for(int i = vEraseIdx.size() - 1; i >= 0; i--)
        {
            _mvPath2f.erase(_mvPath2f.begin()+(vEraseIdx[i]));
        }
        
    }
    void simplifyPath2(void)
    {
        double THD = 0;
        std::vector<int> vEraseIdx;
        for(int i = 0; i < _mvPath2f.size(); i++)
        {
            if(i-2 < 0 || i + 2 >=  _mvPath2f.size())continue;
            if(_mvPath2f[i].x < 0 || _mvPath2f[i].y < 0 ||
                _mvPath2f[i-2].x < 0 || _mvPath2f[i-2].y < 0 ||
                _mvPath2f[i+2].x < 0 || _mvPath2f[i+2].y < 0)continue;
            
            // Get Slope
            double X1 = _mvPath2f[i].x - _mvPath2f[i-2].x;
            double Y1 = _mvPath2f[i].y - _mvPath2f[i-2].y;
            double X2 = _mvPath2f[i+2].x - _mvPath2f[i].x;
            double Y2 = _mvPath2f[i+2].y - _mvPath2f[i].y;

            bool infinity1 = false; 
            bool infinity2 = false;
            double slope1, slope2;
            if(X1 < 0.01) infinity1 = true;
            else slope1 = Y1/X1;
            if(X2 < 0.01) infinity2 = true;
            else slope2 = Y2/X2;

            // Insert Erase Idx
            if(infinity1 && infinity2)vEraseIdx.push_back(i);
            else if(infinity1 || infinity2)
            {
                double angle = 90;
                if(infinity1)
                {
                    double angle = 90 - fabs(atan(slope2))*(180.0/M_PI);
                }
                else if(infinity2)
                {
                    double angle = 90 - fabs(atan(slope1))*(180.0/M_PI);
                }
                else assert(false);
                angle = fabs(angle);
                if(angle <= THD)vEraseIdx.push_back(i);
            }
            else
            {
                double angle = fabs(atan(slope2) - atan(slope1))*(180.0/M_PI);
                angle = fmod(angle,360.0);
                if(angle > 180) angle = 180 - angle;
                angle = fabs(angle);
                if(angle <= THD)vEraseIdx.push_back(i);
            }

        }
        // Erase Point
        for(int i = vEraseIdx.size() - 1; i >= 0; i--)
        {
            _mvPath2f.erase(_mvPath2f.begin()+(vEraseIdx[i]));
        }

    }
    std::vector<cv::Point2f> getPathVector(void)
    {
        return _mvPath2f;
    }
    nav_msgs::Path getPathRosMsg(void)
    {
        assert(_mvPath2f.size() != 0);
        return _mPath;
    }
    void clear(void)
    {
        _mvPath2f.clear();
    }
};

void mySigintHandler(int sig){
    ros::shutdown();
}

void label2Path(cv::Mat img, std::vector<cv::Point2f> &rawPath)
{
    cv::Mat tmpImg = img.clone();
    while(1)
    {
        rawPath.push_back(cv::Point(-1,-1));
        cv::Point startPoint = cv::Point(-1,-1);
        // Get Start Point
        for(int i = 0; i < img.rows; i++)
        {
            for(int j = 0; j < img.cols; j++)
            {
                if((int)tmpImg.at<uchar>(i,j) != 0)continue;
                if(startPoint.x < 0 || startPoint.y <0)
                {
                    startPoint.x = j;
                    startPoint.y = i;
                }
                break; // 1. 가장 작은 y값 => 2. 가장 작은 x값
            }
        }
        cv::Mat imgDebug = img.clone();
        cv::cvtColor(imgDebug,imgDebug,CV_GRAY2BGR);

        if(startPoint.x == -1 || startPoint.y == -1)return;
        rawPath.push_back(startPoint);
        rawPath.push_back(cv::Point(-2,-2));
        // Algorithm
        tmpImg.at<uchar>(startPoint.y,startPoint.x) = (uchar)255;
        
        while(1)
        {
            // cv::circle(imgDebug,startPoint,2,cv::Scalar(0,0,255),-1);
            // cv::imshow("debug start point", imgDebug);
            // cv::waitKey(1);
            int tmpCol = startPoint.x;
            int tmpRow = startPoint.y;
            if(false){}
            else if(((int)tmpImg.at<uchar>(tmpRow+1,tmpCol) == 0) &&
                (tmpRow+1 >= 0) && (tmpCol >= 0) && (tmpRow+1 < 480) && (tmpCol < 640)) // 아래
            {
                startPoint.x = tmpCol;
                startPoint.y = tmpRow+1;
                tmpImg.at<uchar>(startPoint.y,startPoint.x) = (uchar)255;
                rawPath.push_back(startPoint);
            }
            else if((int)tmpImg.at<uchar>(tmpRow,tmpCol-1) == 0&&
                tmpRow >= 0 && tmpCol-1 >= 0 && tmpRow < 480 && tmpCol-1 < 640) // 왼쪽
            {
                startPoint.x = tmpCol-1;
                startPoint.y = tmpRow;
                tmpImg.at<uchar>(startPoint.y,startPoint.x) = (uchar)255;
                rawPath.push_back(startPoint);
            }
            else if((int)tmpImg.at<uchar>(tmpRow+1,tmpCol-1) == 0&&
                tmpRow+1 >= 0 && tmpCol-1 >= 0 && tmpRow+1 < 480 && tmpCol-1 < 640) // 왼쪽+아래
            {
                startPoint.x = tmpCol-1;
                startPoint.y = tmpRow+1;
                tmpImg.at<uchar>(startPoint.y,startPoint.x) = (uchar)255;
                rawPath.push_back(startPoint);
            }
            else if((int)tmpImg.at<uchar>(tmpRow,tmpCol+1) == 0&&
                tmpRow >= 0 && tmpCol+1 >= 0 && tmpRow < 480 && tmpCol+1 < 640) // 오른쪽
            {
                startPoint.x = tmpCol+1;
                startPoint.y = tmpRow;
                tmpImg.at<uchar>(startPoint.y,startPoint.x) = (uchar)255;
                rawPath.push_back(startPoint);
            }
            else if((int)tmpImg.at<uchar>(tmpRow+1,tmpCol+1) == 0&&
                tmpRow+1 >= 0 && tmpCol+1 >= 0 && tmpRow+1 < 480 && tmpCol+1 < 640) // 오른쪽+아래
            {
                startPoint.x = tmpCol+1;
                startPoint.y = tmpRow+1;
                tmpImg.at<uchar>(startPoint.y,startPoint.x) = (uchar)255;
                rawPath.push_back(startPoint);
            }
            else if((int)tmpImg.at<uchar>(tmpRow-1,tmpCol) == 0&&
                tmpRow-1 >= 0 && tmpCol >= 0 && tmpRow-1 < 480 && tmpCol < 640) // 위쪽
            {
                startPoint.x = tmpCol;
                startPoint.y = tmpRow-1;
                tmpImg.at<uchar>(startPoint.y,startPoint.x) = (uchar)255;
                rawPath.push_back(startPoint);
            }
            else if((int)tmpImg.at<uchar>(tmpRow-1,tmpCol-1) == 0&&
                tmpRow-1 >= 0 && tmpCol-1 >= 0 && tmpRow-1 < 480 && tmpCol-1 < 640) // 위쪽+왼쪽
            {
                startPoint.x = tmpCol-1;
                startPoint.y = tmpRow-1;
                tmpImg.at<uchar>(startPoint.y,startPoint.x) = (uchar)255;
                rawPath.push_back(startPoint);
            }
            else if((int)tmpImg.at<uchar>(tmpRow-1,tmpCol+1) == 0&&
                tmpRow-1 >= 0 && tmpCol+1 >= 0 && tmpRow-1 < 480 && tmpCol+1 < 640) // 위쪽+오른쪽
            {
                startPoint.x = tmpCol+1;
                startPoint.y = tmpRow-1;
                tmpImg.at<uchar>(startPoint.y,startPoint.x) = (uchar)255;
                rawPath.push_back(startPoint);
            }
            else
            {
                break;
            }
            
        }

    }
    
    
}

#endif UTILITY_H