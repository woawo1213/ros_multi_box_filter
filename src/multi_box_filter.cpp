/*
 *  Software License Agreement (BSD License)
 *
 *  multi_box_filter.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *  modified: jm <jmshin@wonik.com>
 */
#include <string>
#include <vector>

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"
#include "multi_box_filter/multi_box_filter.h"

#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(laser_filters::LaserScanMultiBoxFilter, filters::FilterBase<sensor_msgs::LaserScan>)


bool laser_filters::LaserScanMultiBoxFilter::configure()
{   
    XmlRpc::XmlRpcValue box_xmlrpc_;
    up_and_running_ = true;

    bool box_set = getParam("box",box_xmlrpc_);
    bool box_frame_set = getParam("box_frame", box_frame_);
    bool invert_set    = getParam("invert", invert_filter);

    parser_.read_param(box_xmlrpc_);
    box_ = parser_.generate_box(box_xmlrpc_);
    marker_pub_.initialize(box_);

    ROS_INFO("Multi Box filter started!");

    if(!box_frame_set)
    {
        ROS_ERROR("box_frame is not set!");
    }
    if(!box_set)
    {
        ROS_ERROR("box is not set!");
    }
    if(!invert_set)
    {
        ROS_INFO("invert filter not set, assuming false");
        invert_filter=false;
    }

    return box_frame_set && box_set;
}

bool laser_filters::LaserScanMultiBoxFilter::update(
    const sensor_msgs::LaserScan& input_scan,
    sensor_msgs::LaserScan &output_scan)
{
    output_scan = input_scan;
    
    sensor_msgs::PointCloud2 laser_cloud;
    
    std::string error_msg;

    bool success = tf_.waitForTransform(
        box_frame_,
        input_scan.header.frame_id,
        input_scan.header.stamp + ros::Duration().fromSec(input_scan.ranges.size()*input_scan.time_increment),
        ros::Duration(1.0),
        ros::Duration(0.01),
        &error_msg
    );
    if(!success)
    {
        ROS_WARN("Could not get transform, irgnoring laser scan! %s", error_msg.c_str());
        return false;
    }
    //polar to cartesian
    try
    {
        projector_.transformLaserScanToPointCloud(box_frame_, input_scan, laser_cloud, tf_);
    }
    catch(tf::TransformException& ex)
    {
        if(up_and_running_)
        {
            ROS_WARN_THROTTLE(1, "Dropping Scan: Tansform unavailable %s", ex.what());
            return true;
        }
        else
        {
            ROS_INFO_THROTTLE(.3, "Ignoring Scan: Waiting for TF");
        }
        return false;
    }
    const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
    const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
    const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
    const int z_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "z");

    if(i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1)
    {
        ROS_INFO_THROTTLE(.3, "x, y, z and index fields are required, skipping scan");
    }

    const int i_idx_offset = laser_cloud.fields[i_idx_c].offset;
    const int x_idx_offset = laser_cloud.fields[x_idx_c].offset;
    const int y_idx_offset = laser_cloud.fields[y_idx_c].offset;
    const int z_idx_offset = laser_cloud.fields[z_idx_c].offset;

    const int pstep = laser_cloud.point_step;
    const long int pcount = laser_cloud.width * laser_cloud.height;
    const long int limit = pstep * pcount;

    int i_idx, x_idx, y_idx, z_idx;  

    for(
        i_idx = i_idx_offset,
        x_idx = x_idx_offset,
        y_idx = y_idx_offset,
        z_idx = z_idx_offset;

        x_idx < limit;

        i_idx += pstep,
        x_idx += pstep,
        y_idx += pstep,
        z_idx += pstep)
    {
    // TODO works only for float data types and with an index field
    // I'm working on it, see https://github.com/ros/common_msgs/pull/78 
        float x = *((float*)(&laser_cloud.data[x_idx]));
        float y = *((float*)(&laser_cloud.data[y_idx]));
        float z = *((float*)(&laser_cloud.data[z_idx]));
        int index = *((int*)(&laser_cloud.data[i_idx]));

        // invert_filter 신경 안씀

        for(int i = 0; i < box_.size(); i++) {
            if(box_[i].is_in(x, y)) {
                // 2022.01.27 : jglee
                // obstacle_layer 에서 cart_leg 를 지워줘야 하기 때문에, inf data 로 clear 함(Nan -> inf).
                output_scan.ranges[index] = std::numeric_limits<float>::infinity();
                break;
            }
        }
    }
    marker_pub_.publish(input_scan.header.stamp);
    up_and_running_ = true;
    return true;
}