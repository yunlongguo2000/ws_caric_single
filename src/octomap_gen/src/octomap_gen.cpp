#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_ros/conversions.h>

class OctomapNode
{
public:
    OctomapNode()
    {
        // 订阅点云消息
        pointcloud_sub = nh.subscribe("/jurong/cloud_inW", 1, &OctomapNode::pointCloudCallback, this);
        octomap_pub = nh.advertise<octomap_msgs::Octomap>("/jurong/octomap", 1);
        octree = new octomap::OcTree(1); // 设置分辨率 米
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        octomap::Pointcloud octomap_cloud;
        octomap::pointCloud2ToOctomap(*msg, octomap_cloud);

        // 更新octree
        for (auto& point : octomap_cloud)
        {
            octree->updateNode(octomap::point3d(point.x(), point.y(), point.z()), true);
        }

        octree->updateInnerOccupancy();

        // 发布octomap
        octomap_msgs::Octomap octomap_msg;
        octomap_msg.header.frame_id = "world";
        octomap_msg.header.stamp = ros::Time::now();
        if (octomap_msgs::fullMapToMsg(*octree, octomap_msg))
        {
            octomap_pub.publish(octomap_msg);
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber pointcloud_sub;
    ros::Publisher octomap_pub;
    octomap::OcTree* octree;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_node");
    OctomapNode node;
    ros::spin();
    return 0;
}
