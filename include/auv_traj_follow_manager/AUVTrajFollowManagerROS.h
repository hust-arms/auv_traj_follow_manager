/*
 * Filename: auv_traj_follow_manager/AUVTrajFollowManagerROS.h
 * Path: auv_traj_follow_manager
 * Created Date: Thirsday, March 17th 2021, 10:58:05 pm
 * Author: zhao wang
 *
 * Copyright (c) 2021 hust-arms
 */

#ifndef AUV_TRAJ_FOLLOW_MANAGER_ROS_H_
#define AUV_TRAJ_FOLLOW_MANAGER_ROS_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include "auv_control_msgs/AUVCtrlInfo.h"
#include "auv_controller/ResetCtrlState.h"
#include "auv_traj_follow_manager/SetMissionStatus.h"

/* boost lib */
#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/chrono.hpp>
#include <boost/timer.hpp>

namespace auv_traj_follow_manager
{

/**
 * @brief AUV trajectory following manager
 */
class AUVTrajFollowManagerROS
{
public:
    /**
     * @brief Constructor
     */
    AUVTrajFollowManagerROS(std::string auv_name, bool debug);

    /**
     * @brief Deconstructor
     */
    ~AUVTrajFollowManagerROS();

    /**
     * @brief Start threads
     */
    void start();

    /**
     * @brief Service to set mission status 
     */
    bool setMissionStatus(auv_traj_follow_manager::SetMissionStatus::Request& req,
                          auv_traj_follow_manager::SetMissionStatus::Response& res);

private:
    /**
     * @brief Transform string to geometry_msgs::Point vector
     */
    bool strToGeoPointVector(const XmlRpc::XmlRpcValue& xmlrpc, std::vector<geometry_msgs::Point>& geo_vec);

    /**
     * @brief Parse 2D point from string 
     */
    std::vector<std::vector<double>> parseVVD(const std::string& input, std::string& error_return);

    /**
     * @brief Start trajectory follow manage
     */
    void trajFollowManageThread();

    /**
     * @brief Wake control 
     */
    void wakeManageThread(const ros::TimerEvent& event);

    /**
     * @brief Start desired info publishment
     */
    void publishThread();

    /**
     * @brief Return mission flag
     */
    bool getMissionFlag()
    {
        boost::unique_lock<boost::recursive_mutex> mission_flag_lock(mission_flag_mutex_);
        bool flag = is_start_mission_;
        mission_flag_lock.unlock();
        return flag;
    }

    /**
     * @brief If vehicle accesses the field of ont way point, update the control information
     */
    void updateCtrlInfo();

    /**
     * @brief If vehicle accesses the end point of waypoint list, reset base property info
     */
    bool isAccessEndPoint();

    /**
     * @brief Return x in global frame
     */
    double getGlobalX()
    {
        boost::unique_lock<boost::recursive_mutex> pose_lock(pose_mutex_);
        double x = x_;
        pose_lock.unlock();
        return x;
    }

    /**
     * @brief Return y in global frame
     */
    double getGlobalY()
    {
        boost::unique_lock<boost::recursive_mutex> pose_lock(pose_mutex_);
        double y = y_;
        pose_lock.unlock();
        return y;
    }

    /**
     * @brief Return z in global frame
     */
    double getGlobalZ()
    {
        boost::unique_lock<boost::recursive_mutex> pose_lock(pose_mutex_);
        double z = z_;
        pose_lock.unlock();
        return z;
    }

    /**
     * @brief Return x linear velocity in vehicle frame
     */
    double getLinVelX()
    {
        boost::unique_lock<boost::recursive_mutex> pose_lock(pose_mutex_);
        double u = u_;
        pose_lock.unlock();
        return u;
    }

    /**
     * @brief Get vehicle pose 
     */
    void getPose(double& x, double& y, double& z)
    {
        boost::unique_lock<boost::recursive_mutex> pose_lock(pose_mutex_);
        x = x_; y = y_; z = z_;
        pose_lock.unlock();
    }

    /**
     * @brief Get index of current followed way point 
     */
    int getWayPointIndex()
    {
        boost::unique_lock<boost::recursive_mutex> wp_index_lock(wp_index_mutex_);
        int wp_index = wp_index_;
        wp_index_lock.unlock();
        return wp_index;
    }
  
    /**
     * @brief Position & pose input
     */
    void posegtCb(const nav_msgs::Odometry::ConstPtr& msg){
        boost::unique_lock<boost::recursive_mutex> pose_lock(pose_mutex_);
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        z_ = msg->pose.pose.position.z;
        pose_lock.unlock();
    }

private:
    /* ros components */
    ros::Publisher auv_control_info_pub_;
    ros::ServiceClient ctrl_state_reset_cl_;

    ros::Subscriber pose_sub_; // should be replaced by TCP client

    /* base property */
    double manage_dt_;
    XmlRpc::XmlRpcValue wp_xmlrpc_;

    /* vehicle states */
    double x_, y_, z_;
    double u_;
    int wp_index_;
    double thre_;

    /* desired vehicle states */
    double depth_d_, pitch_d_, yaw_d_;
    double x_d_, y_d_, u_d_;

    /* threads */
    boost::thread* manage_th_;
    boost::thread* pub_th_;
    boost::condition_variable_any manage_cond_;

    /* target waypoints */
    std::vector<geometry_msgs::Point> wp_vec_;

    /* resources mutex */
    boost::recursive_mutex print_mutex_;
    boost::recursive_mutex pose_mutex_;
    boost::recursive_mutex desired_info_mutex_;
    boost::recursive_mutex mission_flag_mutex_;
    boost::recursive_mutex wp_index_mutex_;

    boost::recursive_mutex manage_mutex_;

    /* flags */
    bool is_start_mission_;

    /* debug */
    bool debug_;
}; // class AUVTrajFollowManagerROS

}; // ns

#endif
