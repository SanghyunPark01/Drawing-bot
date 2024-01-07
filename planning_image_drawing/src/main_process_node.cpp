#include "line_planner.h"

void callbackShutdown(const std_msgs::Bool &msg)
{
    ros::shutdown();
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_image_drawing");
    ros::NodeHandle nh_("~");
    ros::Subscriber subShutdown_ = nh_.subscribe("/shutdown_flag",10,callbackShutdown);
    
    Planner planner(nh_);

    std::thread thread_process0(&Planner::calculateLine, &planner);
    std::thread thread_process1(&Planner::planningLine, &planner);

    ros::AsyncSpinner spinner(6);
    spinner.start();
    // ros::spin();

    signal(SIGINT, mySigintHandler);
    ros::waitForShutdown();

    if(!ros::ok())
    {
        std::cout << "[STATUS]Process shut down!" << "\n";
        system("killall -9 rosmaster && killall -9 rosout"); // all program kill
    }
    return 0;
}