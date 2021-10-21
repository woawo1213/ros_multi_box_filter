/*
 *  multi_box_filter.h
 *  modified: JMSHIN <woawo1213@gmail.com>
 */

#include <string>
#include <vector>

#include "multi_box_filter/multi_box_filter.h"
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"
#include "multi_box_filter/multi_box_filter.h"

#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(laser_filters::LaserScanMultiBoxFilter, filters::FilterBase<sensor_msgs::LaserScan>)

double getNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name)
{
    // Make sure that the value we're looking at is either a double or an int.
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt && value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
        std::string &value_string = value;
        ROS_FATAL("Values in the polygon specification (param %s) must be numbers. Found value %s.",
                  full_param_name.c_str(), value_string.c_str());
        throw std::runtime_error("Values in the polygon specification must be numbers");
    }
    return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

std::vector<laser_filters::Box> makeBoxFromXMLRPC(const XmlRpc::XmlRpcValue &box_xmlrpc, const std::string &full_param_name)
{
    laser_filters::Box box_tmp;
    std::vector<laser_filters::Box> box_;

    float f_x, f_y, f_w, f_h; // [X, Y, W, H]

    // Make sure we have an array of at least 1 elements.
    if (box_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray || box_xmlrpc.size() < 1)
    {
        ROS_FATAL("The box (parameter %s) must be specified as nested list on the parameter server with at least "
                  "1 box eg: [[x1, y1, w1, h1]]",
                  full_param_name.c_str());

        throw std::runtime_error("The box must be specified as nested list on the parameter server with at least "
                                 "1 box eg: [[x1, y1, w1, h1]]");
    }

    for (int i = 0; i < box_xmlrpc.size(); ++i)
    {
        // Make sure each element of the list is an array of size 4. (x and y and width and height coordinates)
        XmlRpc::XmlRpcValue point = box_xmlrpc[i];
        if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() != 4)
        {
            ROS_FATAL("The box (parameter %s) must be specified as list of lists on the parameter server eg: "
                      "[[x1, y1, w1, h1], ..., [xn, yn, wn, hn]], but this spec is not of that form.",
                      full_param_name.c_str());
            throw std::runtime_error("The box must be specified as list of lists on the parameter server eg: "
                                     "[[x1, y1, w1, h1], ..., [xn, yn, wn, hn]], but this spec is not of that form");
        }

        f_x = getNumberFromXMLRPC(point[0], full_param_name);
        f_y = getNumberFromXMLRPC(point[1], full_param_name);
        f_w = getNumberFromXMLRPC(point[2], full_param_name);
        f_h = getNumberFromXMLRPC(point[3], full_param_name);

        box_tmp.x = f_x;
        box_tmp.y = f_y;
        box_tmp.w = f_w;
        box_tmp.h = f_h;

        box_.push_back(box_tmp);
    }

    return box_;
}

laser_filters::LaserScanMultiBoxFilter::LaserScanMultiBoxFilter()
{
}

bool laser_filters::LaserScanMultiBoxFilter::configure()
{
    XmlRpc::XmlRpcValue box_xmlrpc;
    tf::Point min_tmp, max_tmp;

    up_and_running_ = true;

    bool box_set = getParam("box", box_xmlrpc);
    bool box_frame_set = getParam("box_frame", box_frame_);
    bool invert_set = getParam("invert", invert_filter);
    box_ = makeBoxFromXMLRPC(box_xmlrpc, "box");

    for (int i = 0; i < box_.size(); i++)
    {
        max_tmp.setX(box_[i].x); //max_.x()
        max_tmp.setY(box_[i].y); //max_.y()
        max_tmp.setZ(1.000);
        max_.push_back(max_tmp);

        min_tmp.setX(box_[i].x - box_[i].w); //min_.x()
        min_tmp.setY(box_[i].y - box_[i].h); //min_.y()
        min_tmp.setZ(-1.000);
        min_.push_back(min_tmp);
    }

    ROS_INFO("Multi Box filter started!");

    if (!box_frame_set)
    {
        ROS_ERROR("box_frame is not set!");
    }
    if (!box_set)
    {
        ROS_ERROR("box is not set!");
    }
    if (!invert_set)
    {
        ROS_INFO("invert filter not set, assuming false");
        invert_filter = false;
    }

    return box_frame_set && box_set;
}

bool laser_filters::LaserScanMultiBoxFilter::update(
    const sensor_msgs::LaserScan &input_scan,
    sensor_msgs::LaserScan &output_scan)
{
    output_scan = input_scan;
    sensor_msgs::PointCloud2 laser_cloud;

    std::string error_msg;

    bool success = tf_.waitForTransform(
        box_frame_,
        input_scan.header.frame_id,
        input_scan.header.stamp + ros::Duration().fromSec(input_scan.ranges.size() * input_scan.time_increment),
        ros::Duration(1.0),
        ros::Duration(0.01),
        &error_msg);
    if (!success)
    {
        ROS_WARN("Could not get transform, irgnoring laser scan! %s", error_msg.c_str());
        return false;
    }

    try
    {
        projector_.transformLaserScanToPointCloud(box_frame_, input_scan, laser_cloud, tf_);
    }
    catch (tf::TransformException &ex)
    {
        if (up_and_running_)
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

    if (i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1)
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

    for (
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
        float x = *((float *)(&laser_cloud.data[x_idx]));
        float y = *((float *)(&laser_cloud.data[y_idx]));
        float z = *((float *)(&laser_cloud.data[z_idx]));
        int index = *((int *)(&laser_cloud.data[i_idx]));
        int box_count = box_.size();

        tf::Point point(x, y, z);

        if (!invert_filter)
        {
            for (int count_ = 0; count_ < box_count; count_++) //remove the boxes of the lits
            {
                if (inBox(point, count_))
                {
                    output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
        else
        {
            for (int count_ = 0; count_ < box_count; count_++)
            {
                if (inBox(point, count_))
                {
                    output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
    }
    up_and_running_ = true;
    return true;
}

bool laser_filters::LaserScanMultiBoxFilter::inBox(tf::Point &point, int count)
{
    return point.x() < max_[count].x() && point.x() > min_[count].x() &&
           point.y() < max_[count].y() && point.y() > min_[count].y() &&
           point.z() < max_[count].z() && point.z() > min_[count].z();
}