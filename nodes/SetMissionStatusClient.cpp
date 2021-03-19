/*                                                                                             
 * Filename: SetMissionStatusClient.cpp                                               
 * Path: auv_traj_follow_manager\nodes
 * Created Date: Friday, March 19th 2021, 14:05:02 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include "cstdlib"
#include <ros/ros.h>
#include "auv_traj_follow_manager/SetMissionStatus.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_mission_status");
    if(argc != 2)
    {
        ROS_WARN("usage: set_mission_status type: <status tag>");
        return 1;
    }

    ros::NodeHandle nh;
    ros::ServiceClient mission_status_set_cl = nh.serviceClient<auv_traj_follow_manager::SetMissionStatus>("set_mission_status");
    
    auv_traj_follow_manager::SetMissionStatus srv;
    srv.request.IsManage = atoll(argv[1]);

    if(mission_status_set_cl.call(srv))
    {
        ROS_INFO("Server feedback: %s", srv.response.FeedbackMsg.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service set_mission_status!");
        return 1;
    }

    return 0;
}

