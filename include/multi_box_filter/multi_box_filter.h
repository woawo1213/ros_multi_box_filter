/*
 *  multi_box_filter.h
 *  modified: JMSHIN <woawo1213@gmail.com>
 */

#ifndef MULTIBOXFILTER_H
#define MULTIBOXFILTER_H

#include <string>
#include <vector>

#include <filters/filter_base.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Polygon.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace laser_filters
{
    /**
 * @brief This is a filter that removes points in a laser scan inside of multi cartesian box.
 */

    struct Box
    {
        Box(float x, float y, float w, float h) : x(x), y(y), w(w), h(h) {}
        float x, y, w, h;
        Box() { float x = 0, y = 0, w = 0, h = 0; }
    };

    class LaserScanMultiBoxFilter : public filters::FilterBase<sensor_msgs::LaserScan>
    {
    public:
        LaserScanMultiBoxFilter();
        bool configure();

        bool update(
            const sensor_msgs::LaserScan &input_scan,
            sensor_msgs::LaserScan &filtered_scan);

    private:
        ros::Publisher box_pub_;

        bool inBox(tf::Point &point, int count);

        std::string box_frame_;
        std::vector<Box> box_;
        laser_geometry::LaserProjection projector_;

        // tf listener to transform scans into the box_frame
        tf::TransformListener tf_;

        // defines two opposite corners of the box
        std::vector<tf::Point> min_, max_;

        bool invert_filter;
        bool up_and_running_;
    };

}

#endif /* multi_box_filter.h */
