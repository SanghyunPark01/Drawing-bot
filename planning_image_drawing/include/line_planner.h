#ifndef LINE_PLANNER_H
#define LINE_PLANNER_H

#include "line_planner.h"
#include "utility.h"

#include <opencv2/xfeatures2d.hpp>

static Path _gMousePath;
class Planner
{
private:
    // param
    std::string IMAGE_TOPIC, FLAG_IMAGE_TOPIC, DRAWING_TYPE, START_CALCULATE, START_DRAWING;
    std::string PUB_CAPTURED_IMG, PUB_DRAWING_PATH;
    // ROS
    ros::NodeHandle nh;
    ros::Subscriber _subImage;
    ros::ServiceServer _srvSetImgFlag;
    ros::ServiceServer _srvsSetType;
    ros::ServiceServer _srvCalculateFlag;
    ros::ServiceServer _srvPubPahtFlag;
    ros::Publisher _pubCapturedImg;
    ros::Publisher _pubPath;

    //
    int _mnDrawingType = -1; // 0 네모, 1 원, 2 카메라, 3 직접 그리기?
    bool _mbCalculateFlag = false;
    bool _mbPubFlag = false;
    int SAVE_IMAGE_NUM = 0;

    //
    std::queue<sensor_msgs::ImageConstPtr> _mqImgBuf;
    std::mutex _mtxCallback;

    // image set info
    std::queue<cv::Mat> _mqImgData;
    std::mutex _mtxSetImg;
    // line image
    std::queue<cv::Mat> _mqLineImg;
    std::mutex _mtxLine;
    // path
    std::queue<Path> _mqPath;
    std::mutex _mtxPath;


    //
    bool _mbSetImgFlag = false;
    bool _mbDrawingFlag = false;
    cv::Ptr<cv::FastFeatureDetector> _mFASTdetector = cv::FastFeatureDetector::create(50);

    // ROS callback
    void callbackImage(const sensor_msgs::ImageConstPtr &imgMsg);
    bool setImageFlag(planning_image_drawing::set_img_flagRequest& req, planning_image_drawing::set_img_flagResponse& res);
    bool setDrawingType(planning_image_drawing::drawing_typeRequest& req, planning_image_drawing::drawing_typeResponse& res);
    bool setCalcuateFlag(planning_image_drawing::start_calculate_flagRequest& req, planning_image_drawing::start_calculate_flagResponse& res);
    bool setPubPathFlag(planning_image_drawing::start_drawing_flagRequest& req, planning_image_drawing::start_drawing_flagResponse& res);

    
    // Process
    void getLine(void);
    void drawWithMouse(void);
    static void drawLine(int event, int x, int y, int flags, void* userdata)
    {
        static cv::Point prevPoint(-1,-1);
        if(event == cv::EVENT_LBUTTONDOWN)
        {
            prevPoint = cv::Point(x,y);
            _gMousePath.insertPoint(cv::Point2f(x,y)); // move point
            _gMousePath.insertPoint(cv::Point2f(-2,-2)); // down
        }
        else if(event == cv::EVENT_LBUTTONUP)
        {
            prevPoint = cv::Point(-1,-1);
            _gMousePath.insertPoint(cv::Point2f(-1,-1)); //up
        }
        else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON))
        {
            cv::Point currentPoint(x,y);
            if(prevPoint.x >=0 && prevPoint.y >=0 && 
                currentPoint.x >=0 && currentPoint.x < 640 &&
                currentPoint.y >=0 && currentPoint.y < 480)
            {
                double s = (prevPoint.x - currentPoint.x)*(prevPoint.x - currentPoint.x) + (prevPoint.y - currentPoint.y)*(prevPoint.y - currentPoint.y);
                if( s >= 2)
                {
                    _gMousePath.insertPoint(cv::Point2f(x,y));
                    cv::line(*static_cast<cv::Mat*>(userdata), prevPoint, currentPoint, cv::Scalar(0, 0, 255), 2);
                    prevPoint = currentPoint;
                    cv::imshow("White Board", *static_cast<cv::Mat*>(userdata));
                }
                
            }
            else
            {
                prevPoint = cv::Point(-1,-1);
                _gMousePath.insertPoint(cv::Point2f(-1,-1)); //up
            }
        }
    }
    void getPath(void);
    void publishPath(void);
public:
    Planner(const ros::NodeHandle& nh_);
    // thread
    void calculateLine();
    void planningLine();
    
};

#endif LINE_PLANNER_H