/*                                                                                             
 * Filename: AUVTrajFollowManagerROS.cpp                                               
 * Path: auv_traj_follow_manager\nodes
 * Created Date: Friday, March 19th 2021, 14:05:02 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <stdio.h>
#include "auv_traj_follow_manager/AUVTrajFollowManagerROS.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auv_traj_follow_manager_node");

    bool debug = false;

    if(argc == 2)
    {
        if(strcmp(argv[1], "debug") == 0)
        {
            printf("Debug mode\n");
            debug = true;
        }
        else{
            printf("Raw mode\n");
        }
    }

    std::string auv_ns = "armsauv";
    auv_traj_follow_manager::AUVTrajFollowManagerROS auv_traj_follow_manager(auv_ns, debug);

    auv_traj_follow_manager.start();

    ros::spin();

    return 0;
}

