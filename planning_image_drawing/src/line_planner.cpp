#include "line_planner.h"

Planner::Planner(const ros::NodeHandle& nh_):nh(nh_)
{
    nh.param<std::string>("/param/name/image_topic", IMAGE_TOPIC, "/");
    nh.param<std::string>("/param/name/image_flag_topic", FLAG_IMAGE_TOPIC, "/");
    nh.param<std::string>("/param/name/drawing_type_topic", DRAWING_TYPE, "/");
    nh.param<std::string>("/param/name/start_calcuate_topic", START_CALCULATE, "/");
    nh.param<std::string>("/param/name/pub_path_flag", START_DRAWING, "/");
    nh.param<std::string>("/param/name/pub_msg_capture_img", PUB_CAPTURED_IMG, "/");
    nh.param<std::string>("/param/name/pub_msg_drawing_path", PUB_DRAWING_PATH, "/");

    _subImage = nh.subscribe(IMAGE_TOPIC, 1000, &Planner::callbackImage, this);
    _srvSetImgFlag = nh.advertiseService(FLAG_IMAGE_TOPIC, &Planner::setImageFlag, this);
    _srvsSetType = nh.advertiseService(DRAWING_TYPE, &Planner::setDrawingType, this);
    _srvCalculateFlag = nh.advertiseService(START_CALCULATE, &Planner::setCalcuateFlag, this);
    _srvPubPahtFlag = nh.advertiseService(START_DRAWING, &Planner::setPubPathFlag, this);

    _pubCapturedImg = nh.advertise<sensor_msgs::Image>(PUB_CAPTURED_IMG, 10);
    _pubPath = nh.advertise<nav_msgs::Path>(PUB_DRAWING_PATH, 10);
}

/*
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 
@@@@@@@@@@@@@@@@       Callback        @@@@@@@@@@@@@@@@@ 
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 
 */
void Planner::callbackImage(const sensor_msgs::ImageConstPtr &imgMsg)
{
    std::unique_lock<std::mutex> lock(_mtxCallback);
    _mqImgBuf.push(imgMsg);
    // std::cout << "[DEBUG]Get Image" << "\n";
    if(_mqImgBuf.size() > 100)
    {
        _mqImgBuf.pop();
    }
}
bool Planner::setImageFlag(planning_image_drawing::set_img_flagRequest& req, planning_image_drawing::set_img_flagResponse& res)
{
    double _mdSetTime = -3e8;
    // if(_mnDrawingType >= 0 && _mnDrawingType <= 3)
    // {
    //     // Do nothing
    // }
    if(req.set_image_flag_time.toSec() <= 0 || _mnDrawingType == -1 || _mbCalculateFlag)
    {
        res.set_success = false;
        return true;
    }
    if(_mnDrawingType == 0)
    {
        cv::Mat img = cv::imread("/home/psh/kw_ws/src/planning_image_drawing/test_image/rectangle.png", cv::IMREAD_GRAYSCALE);
        cv::Mat pub_img = cv::imread("/home/psh/kw_ws/src/planning_image_drawing/test_image/rectangle.png");
        cv::resize(img,img,cv::Size(640, 480));
        std::unique_lock<std::mutex> lock(_mtxCallback);
        while(!_mqImgBuf.empty())
        {
            _mqImgBuf.pop();
        }

        sensor_msgs::ImagePtr img_msg;
        cv::resize(pub_img,pub_img,cv::Size(640, 480));
        img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img.clone()).toImageMsg();
        _pubCapturedImg.publish(img_msg);

        std::unique_lock<std::mutex>(_mtxSetImg);
        _mqImgData.push(img);
        res.set_success = true;
        return true;
    }
    else if(_mnDrawingType == 1)
    {
        cv::Mat img = cv::imread("/home/psh/kw_ws/src/planning_image_drawing/test_image/circle.jpeg", cv::IMREAD_GRAYSCALE);
        cv::Mat pub_img = cv::imread("/home/psh/kw_ws/src/planning_image_drawing/test_image/circle.jpeg");
        cv::resize(img,img,cv::Size(640, 480));
        std::unique_lock<std::mutex> lock(_mtxCallback);
        while(!_mqImgBuf.empty())
        {
            _mqImgBuf.pop();
        }

        sensor_msgs::ImagePtr img_msg;
        cv::resize(pub_img,pub_img,cv::Size(640, 480));
        img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img.clone()).toImageMsg();
        _pubCapturedImg.publish(img_msg);

        std::unique_lock<std::mutex>(_mtxSetImg);
        _mqImgData.push(img);
        res.set_success = true;
        return true;
    }
    else if(_mnDrawingType == 2)
    {
        if(_mqImgBuf.empty())
        {
            res.set_success = false;
            return true;
        }
    }
    else if(_mnDrawingType == 3)
    {
        _mbSetImgFlag = true;
        res.set_success = true;
        return true;
    }


    _mdSetTime = req.set_image_flag_time.toSec();
    sensor_msgs::ImageConstPtr imgMsg;
    while(_mnDrawingType == 2)
    {
        if(_mqImgBuf.empty())continue;
        std::unique_lock<std::mutex> lock(_mtxCallback);
        imgMsg = _mqImgBuf.front();
        if(std::abs(imgMsg->header.stamp.toSec() - _mdSetTime) < 0.1)
        {
            break;
        }
        else if(imgMsg->header.stamp.toSec() < _mdSetTime)
        {
            _mqImgBuf.pop();
        }
        else
        {
            break;
        }
    }
    {
        std::unique_lock<std::mutex> lock(_mtxCallback);
        while(!_mqImgBuf.empty())
        {
            _mqImgBuf.pop();
        }
    }
    
    std::cout << "[DEBUG]Set Image" << "\n";
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e1)
    {
        res.set_success = false;
        ROS_ERROR("cv_bridge exception: %s", e1.what());
        return false;
    }
    cv::Mat img = cv_ptr->image;
    res.set_success = true;
    {
        std::unique_lock<std::mutex>(_mtxSetImg);
        _mqImgData.push(img);
    }

    /*===============================
        TODO: Pub Set image to UI
    ================================*/
    _pubCapturedImg.publish(*imgMsg);
    SAVE_IMAGE_NUM++;
    std::string name = std::to_string(SAVE_IMAGE_NUM);
    cv::imwrite("/home/psh/kw_ws/src/planning_image_drawing/captured_img/"+name+".png",img);

    return true;
}
bool Planner::setDrawingType(planning_image_drawing::drawing_typeRequest& req, planning_image_drawing::drawing_typeResponse& res)
{
    _mnDrawingType = req.type;
    if(_mnDrawingType < 0 || _mnDrawingType > 3 || _mbCalculateFlag)
    {
        res.set_success = false;
        return true;
    }
    if(_mnDrawingType == 3)
    {
        _mbDrawingFlag = true;
    }
    
    res.set_success = true;
    std::cout << "[DEBUG]Drawing Type: " << _mnDrawingType << "\n";
    return true;
}
bool Planner::setCalcuateFlag(planning_image_drawing::start_calculate_flagRequest& req, planning_image_drawing::start_calculate_flagResponse& res)
{
    // if(_mqLineImg.empty())
    // {
    //     res.set_success = false;
    //     return true;
    // }
    if(req.start_flag)_mbCalculateFlag = true;
    else _mbCalculateFlag =false;
    res.set_success = true;
    return true;
}
bool Planner::setPubPathFlag(planning_image_drawing::start_drawing_flagRequest& req, planning_image_drawing::start_drawing_flagResponse& res)
{
    if(req.start_flag)_mbPubFlag = true;
    else _mbPubFlag = false;

    /*======================
        TODO: Pub Path
    =======================*/






    res.set_success = true;
    return true;
}

/*
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 
@@@@@@@@@@@@@@@@        Thread         @@@@@@@@@@@@@@@@@ 
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 
 */
void Planner::calculateLine()
{
    while (1)
    {
        if(_mbDrawingFlag)
        {
            drawWithMouse();
            _mbDrawingFlag = false;
        }
        else
        {
            getLine();
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}
void Planner::planningLine()
{
    while (1)
    {
        getPath();
        publishPath();
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

/*
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 
@@@@@@@@@@@@@@@       For Thread       @@@@@@@@@@@@@@@@@ 
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 
 */
void Planner::getLine(void)
{
    if(_mqImgData.empty())return;
    cv::Mat igTarget;
    {
        std::unique_lock<std::mutex>(_mtxSetImg);
        if(_mnDrawingType == 2){
            cv::cvtColor(_mqImgData.front(), igTarget, cv::COLOR_BGR2GRAY);
        }
        else
        {
            igTarget = _mqImgData.front();
        }
        _mqImgData.pop();
    }

    // cv::GaussianBlur(igTarget,igTarget, cv::Size(3,3), 0);
    cv::Mat igEdge;
    cv::Canny(igTarget, igEdge, 100, 150, 3);
    cv::dilate(igEdge,igEdge,cv::Mat(),cv::Point(-1,-1),1);
    // cv::dilate(igEdge,igEdge,cv::Mat(),cv::Point(-1,-1),1);
    // cv::erode(igEdge,igEdge,cv::Mat(),cv::Point(-1,-1),1);
    cv::erode(igEdge,igEdge,cv::Mat(),cv::Point(-1,-1),1);

    {
        std::unique_lock<std::mutex>(_mtxLine);
        _mqLineImg.push(igEdge);
    }
}
void Planner::drawWithMouse(void)
{
    _gMousePath.clear();
    cv::Mat imgWhiteBoardPlanner(480,640, CV_8UC3, cv::Scalar(255,255,255));
    cv::imshow("White Board", imgWhiteBoardPlanner);
    cv::setMouseCallback("White Board",drawLine,&imgWhiteBoardPlanner);
    while(1)
    {
        if(_mbSetImgFlag || _mnDrawingType != 3)
        {
            cv::destroyAllWindows();

            if(_mnDrawingType == 3)
            {
                {
                    std::unique_lock<std::mutex> lock(_mtxLine);
                    while(!_mqLineImg.empty())
                    {
                        _mqLineImg.pop();
                    }
                }
                /*===============================
                    TODO: Pub Set image to UI
                ================================*/
                sensor_msgs::ImagePtr img_msg;
                img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgWhiteBoardPlanner.clone()).toImageMsg();
                _pubCapturedImg.publish(img_msg);

                // save path
                {
                    std::lock_guard<std::mutex>lock(_mtxPath);
                    _mqPath.push(_gMousePath);
                }
                
            }
            _mbSetImgFlag = false;
            break;
        }
        cv::waitKey(1);
    }
}

void Planner::getPath(void)
{   
    if(!_mbCalculateFlag)return;
    if(_mqLineImg.empty())return;
    // if(_mbSetImgFlag)return;
    cv::Mat igLine;
    {
        std::unique_lock<std::mutex> lock(_mtxLine);
        // 이미지 마지막 한장만
        while(1)
        {
            if(_mqLineImg.size() == 1)break;
            _mqLineImg.pop();
        }
        igLine = _mqLineImg.front();
        _mqLineImg.pop();
    }
    // cv::imshow("test_line",igLine);
    // cv::waitKey(0);


    // ===========================================================================
    cv::Mat labels ,stats, centroids;
    int nNumOfLabel = cv::connectedComponentsWithStats(igLine, labels, stats, centroids, 8, CV_32S);
    // std::cout << "num of labels: " << nNumOfLabel << "\n"; // 갯수 백그라운드 포함 -> 0 번째가 백그라운드
    // std::cout << "labels row: " << labels.rows << "\n"; 
    // std::cout << "labels col: " << labels.cols << "\n"; 
    // std::cout << "stats row: " << stats.rows << "\n"; // 갯수 백그라운드 포함
    // std::cout << "stats col: " << stats.cols << "\n"; // 좌상단x, 좌상단y, 가로, 세로, 면적
    
    std::vector<cv::Mat> vLabelImg;
    // std::vector<cv::Point> vEndPoint;
    for(int i = 1; i < nNumOfLabel; i++)
    {
        cv::Mat empty = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));
        cv::cvtColor(empty,empty,CV_RGB2GRAY);
        vLabelImg.push_back(empty.clone());
    }
    // genereate each label img
    for(int i = 0; i < labels.rows; i++)
    {
        for(int j = 0; j < labels.cols; j++)
        {
            int nLabelNum = labels.at<int>(i,j);
            if(nLabelNum == 0)continue;
            vLabelImg[nLabelNum-1].at<uchar>(i,j) = (uchar)0;
        }
    }

    // for(int i = 0; i < vLabelImg.size(); i++)
    // {
    //     cv::imshow("test",vLabelImg[i]);
    //     cv::waitKey(0);
    // }
    
    std::vector<cv::Point2f> rawPath;
    for(int i = 0; i < vLabelImg.size(); i++)
    {
        int size = stats.at<int>(i+1,4);
        if(size < 100)continue;
        cv::erode(vLabelImg[i],vLabelImg[i],cv::Mat(),cv::Point(-1,-1),1);
        cv::dilate(vLabelImg[i],vLabelImg[i],cv::Mat(),cv::Point(-1,-1),1);
        label2Path(vLabelImg[i].clone(), rawPath);
        // std::cout << rawPath.size() << "\n";
    }
    // cv::destroyAllWindows();
    // ===========================================================================
    std::cout << "[DEBUG]Raw Path size: " << rawPath.size() << "\n";

    Path path;
    path.insertPath(rawPath);

    // for(int idxRow = 0; idxRow < igLine.rows ; idxRow++)
    // {
    //     path.insertPoint(cv::Point2f(-1,-1));
    //     for(int idxCol = 0; idxCol < igLine.cols; idxCol++)
    //     {
    //         if(idxCol == 0)
    //         {
    //             if((int)igLine.at<uchar>(idxRow,idxCol) == 255)
    //             {
    //                 path.insertPoint(cv::Point2f(idxCol,idxRow));
    //                 path.insertPoint(cv::Point2f(-2,-2));
    //             }
    //             continue;
    //         }
    //         // After idxCol == 1
    //         int nCase = -1;
    //         if((int)igLine.at<uchar>(idxRow,idxCol) == 0)nCase = 0; // up
    //         else if((int)igLine.at<uchar>(idxRow,idxCol) == 255 && (int)igLine.at<uchar>(idxRow,idxCol-1) == 0)nCase = 1; // down
    //         else if((int)igLine.at<uchar>(idxRow,idxCol) == 255 && (int)igLine.at<uchar>(idxRow,idxCol-1) == 255)nCase = 2; // move

    //         if(nCase == 0)path.insertPoint(cv::Point2f(-1,-1));
    //         else if(nCase == 1)
    //         {
    //             path.insertPoint(cv::Point2f(idxCol,idxRow));
    //             path.insertPoint(cv::Point2f(-2,-2));
    //         }
    //         else if(nCase == 2)path.insertPoint(cv::Point2f(idxCol,idxRow));
    //     }
    // }

    {
        std::lock_guard<std::mutex>lock(_mtxPath);
        _mqPath.push(path);
    }
}

void Planner::publishPath(void)
{
    if(!_mbCalculateFlag)return;
    if(_mqPath.empty())return;
    Path curPath;
    {
        std::lock_guard<std::mutex>lock(_mtxPath);
        while(1)
        {
            if(_mqPath.size() == 1)break;
            _mqPath.pop();
        }
        curPath = _mqPath.front();
        _mqPath.pop();
    }
    curPath.createPath();
    // if(false)
    // {
    //     auto path = curPath.getPathVector();
    //     cv::Mat empty = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));
    //     cv::cvtColor(empty,empty,CV_RGB2GRAY);
    //     // std::cout << (int)empty.at<uchar>(120,120) << "\n";
    //     for(int i = 0 ; i < path.size(); i++)
    //     {
    //         auto tmpPoint = path[i];
    //         if(tmpPoint.x < 0 || tmpPoint.y < 0)continue;
    //         cv::circle(empty, tmpPoint, 5, cv::Scalar(0));
    //         cv::imshow("test",empty);

    //         if(path.size() > 1000)cv::waitKey(5);
    //         else cv::waitKey(20);
    //     }
    //     cv::destroyAllWindows();
    // }
    int nRawSize = curPath.getPathVector().size();
    std::cout << "[DEBUG]Path Size: " << nRawSize << "\n";
    int nSimpleSize;

    // auto waypoint optimization
    int nNum = 0;
    // if(_mnDrawingType != 3)curPath.simplifyPath();
    nNum++;
    std::cout << "[DEBUG]Simplify Path Size[" << nNum << "]: " << curPath.getPathVector().size() << "\n";
    nSimpleSize = curPath.getPathVector().size();
    // while (true)
    // {
    //     curPath.simplifyPath();
    //     nNum++;
    //     std::cout << "[DEBUG]Simplify Path Size[" << nNum << "]: " << curPath.getPathVector().size() << "\n";
    //     if(curPath.getPathVector().size() == nSimpleSize)break;
    //     nSimpleSize = curPath.getPathVector().size();
    // }

    // nNum = 0;
    // while (true)
    // {
    //     curPath.simplifyPath2();
    //     nNum++;
    //     std::cout << "[DEBUG]Simplify2 Path Size[" << nNum << "]: " << curPath.getPathVector().size() << "\n";
    //     if(curPath.getPathVector().size() == nSimpleSize)break;
    //     nSimpleSize = curPath.getPathVector().size();
    // }

    
    curPath.createROSMsg();
    /*==========================
        TODO: Pub Path to UI
    ==========================*/
    {
        _pubPath.publish(curPath.getPathRosMsg());
    }
    _mbCalculateFlag = false;
}
