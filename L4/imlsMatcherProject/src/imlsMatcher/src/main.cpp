#include "imls_icp.h"
#include <csm/csm_all.h>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "champion_nav_msgs/ChampionNavLaserScan.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "CsmMatcher.h"
#include "sophus/se3.h"
#include "sophus/so3.h"
//pcl::visualization::CloudViewer g_cloudViewer("cloud_viewer");
//此处bag包的地址需要自行修改
std::string bagfile = "/home/wang/CLionProjects/lidar_slam/L4/imlsMatcherProject/src/bag/imls_icp.bag";

class imlsDebug
{
public:
    imlsDebug()
    {
        m_imlsPathPub = m_node.advertise<nav_msgs::Path>("imls_path_pub_",1,true);
        m_imlsPath.header.stamp = ros::Time::now();
        m_imlsPath.header.frame_id = "odom";
        m_odomPathPub = m_node.advertise<nav_msgs::Path>("odom_path_pub_",1,true);
        m_odomPath.header.stamp = ros::Time::now();
        m_odomPath.header.frame_id = "odom";

        m_isFirstFrame = true;

        match_method = "csm";

        rosbag::Bag bag;
        bag.open(bagfile, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(std::string("/sick_scan"));
        topics.push_back(std::string("/odom"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        //按顺序读取bag内激光的消息和里程计的消息
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            champion_nav_msgs::ChampionNavLaserScanConstPtr scan = m.instantiate<champion_nav_msgs::ChampionNavLaserScan>();
            if(scan != NULL)
            {
                if(match_method == "imls")
                    championLaserScanCallback(scan);
                else
                    championLaserScanCallbackCsmMatch(scan);
            }


            nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();
            if(odom != NULL)
                odomCallback(odom);

            ros::spinOnce();
            if(!ros::ok())
                break;
        }
        // m_laserscanSub = m_nh.subscribe("sick_scan",5,&imlsDebug::championLaserScanCallback,this);
    }

    //将激光消息转换为激光坐标系下的二维点云
    void ConvertChampionLaserScanToEigenPointCloud(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg,
                                                   std::vector<Eigen::Vector2d>& eigen_pts)
    {
        eigen_pts.clear();
        for(int i = 0; i < msg->ranges.size(); ++i)
        {
            if(msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;

            double lx = msg->ranges[i] * std::cos(msg->angles[i]);
            double ly = msg->ranges[i] * std::sin(msg->angles[i]);

            if(std::isnan(lx) || std::isinf(ly) ||
               std::isnan(ly) || std::isinf(ly))
                continue;

            eigen_pts.push_back(Eigen::Vector2d(lx,ly));
        }
    }
    //把激光雷达数据 转换为PI-ICP需要的数据
    void LaserScanToLDP(const champion_nav_msgs::ChampionNavLaserScanConstPtr& pScan,
                               LDP& ldp)
    {

        int nPts = pScan->ranges.size();
        ldp = ld_alloc_new(nPts);

        for(int i = 0;i < nPts;i++)
        {
            double dist = pScan->ranges[i];
            if(dist > pScan->range_min && dist < pScan->range_max)
            {
                ldp->valid[i] = 1;
                ldp->readings[i] = dist;
            }
            else
            {
                ldp->valid[i] = 0;
                ldp->readings[i] = -1;
            }
//            ldp->theta[i] = pScan->angle_min+pScan->angle_increment*i;
            ldp->theta[i] = pScan->angles[i];
        }
        ldp->min_theta = ldp->theta[0];
//        std::cout<<"pScan->angle_0 = "<<ldp->theta[0]<<"  pScan->angle_min = "<<pScan->angle_min<<std::endl;
        ldp->max_theta = ldp->theta[nPts-1];
//        std::cout<<"pScan->angle_n = "<<ldp->theta[nPts-1]<<"  pScan->angle_max = "<<pScan->angle_max<<std::endl;

        ldp->odometry[0] = 0.0;
        ldp->odometry[1] = 0.0;
        ldp->odometry[2] = 0.0;

        ldp->true_pose[0] = 0.0;
        ldp->true_pose[1] = 0.0;
        ldp->true_pose[2] = 0.0;
    }

    void championLaserScanCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg)
    {
        if(m_isFirstFrame == true)
        {
            std::cout <<"First Frame"<<std::endl;
            m_isFirstFrame = false;
            m_prevLaserPose = Eigen::Vector3d(0, 0, 0);
            pubPath(m_prevLaserPose, m_imlsPath, m_imlsPathPub);
            ConvertChampionLaserScanToEigenPointCloud(msg, m_prevPointCloud);
            return ;
        }
        std::vector<Eigen::Vector2d> nowPts;
        ConvertChampionLaserScanToEigenPointCloud(msg, nowPts);
        //调用imls进行icp匹配，并输出结果．
        m_imlsMatcher.setSourcePointCloud(nowPts);
        m_imlsMatcher.setTargetPointCloud(m_prevPointCloud);

        Eigen::Matrix3d rPose,rCovariance;
        if(m_imlsMatcher.Match(rPose,rCovariance))
        {
            std::cout <<"IMLS Match Successful:"<<rPose(0,2)<<","<<rPose(1,2)<<","<<atan2(rPose(1,0),rPose(0,0))*57.295<<std::endl;
            Eigen::Matrix3d lastPose;
            lastPose << cos(m_prevLaserPose(2)), -sin(m_prevLaserPose(2)), m_prevLaserPose(0),
                    sin(m_prevLaserPose(2)),  cos(m_prevLaserPose(2)), m_prevLaserPose(1),
                    0, 0, 1;
            Eigen::Matrix3d nowPose = lastPose * rPose;
            m_prevLaserPose << nowPose(0, 2), nowPose(1, 2), atan2(nowPose(1,0), nowPose(0,0));
            pubPath(m_prevLaserPose, m_imlsPath, m_imlsPathPub);
        }
        else
        {
            std::cout <<"IMLS Match Failed!!!!"<<std::endl;
        }

        m_prevPointCloud = nowPts;
    }
    void championLaserScanCallbackCsmMatch(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg)
    {
//        std::cout<<"scan callback,  "<<"time stamp = "<<msg->header.stamp<<std::endl;

        if(m_isFirstFrame == true)
        {
            std::cout <<"First Frame"<<std::endl;
            m_isFirstFrame = false;
            m_prevLaserPose = Eigen::Vector3d(0, 0, 0);
            pubPath(m_prevLaserPose, m_imlsPath, m_imlsPathPub);
            LaserScanToLDP(msg, m_csmMatcher.m_prevLDP);
            return ;
        }
        LDP nowLDP;
        LaserScanToLDP(msg, nowLDP);

        //调用csm进行icp匹配，并输出结果．
//        m_imlsMatcher.setSourcePointCloud(nowPts);
//        m_imlsMatcher.setTargetPointCloud(m_prevPointCloud);

        Eigen::Matrix3d rPose,rCovariance;
        Eigen::Vector3d rPoseVec;
        if(m_odomPath.poses.size()<2)
            rPoseVec = Eigen::Vector3d::Zero();
        else
            rPoseVec = cal_delta_distance();
//        std::cout<<"before match, rpose = "<<rPoseVec.transpose()<<std::endl;
        rPoseVec = m_csmMatcher.PIICPBetweenTwoFrames(nowLDP,rPoseVec);
//        std::cout<<"after match, rpose = "<<rPoseVec.transpose()<<std::endl;

        rPose << cos(rPoseVec[2]), -sin(rPoseVec[2]), rPoseVec[0],
                 sin(rPoseVec[2]), cos(rPoseVec[2]),  rPoseVec[1],
                 0               , 0               ,  1;
//        std::cout <<"csm Match Successful:"<<rPose(0,2)<<","<<rPose(1,2)<<","<<atan2(rPose(1,0),rPose(0,0))*57.295<<std::endl;
        Eigen::Matrix3d lastPose;
        lastPose << cos(m_prevLaserPose(2)), -sin(m_prevLaserPose(2)), m_prevLaserPose(0),
                sin(m_prevLaserPose(2)),  cos(m_prevLaserPose(2)), m_prevLaserPose(1),
                0, 0, 1;
        Eigen::Matrix3d nowPose = lastPose * rPose;
        m_prevLaserPose << nowPose(0, 2), nowPose(1, 2), atan2(nowPose(1,0), nowPose(0,0));
        pubPath(m_prevLaserPose, m_imlsPath, m_imlsPathPub);

    }

    //求解得到两帧odom数据之间的位姿差
    //即求解当前位姿　在　上一时刻　坐标系中的坐标
    Eigen::Vector3d  cal_delta_distance()
    {

        Eigen::Vector3d d_pos;  //return value
        int size = m_odomPath.poses.size();
        Eigen::Quaterniond now_Q;
        now_Q.x() = m_odomPath.poses[size-1].pose.orientation.x;
        now_Q.y() = m_odomPath.poses[size-1].pose.orientation.y;
        now_Q.z() = m_odomPath.poses[size-1].pose.orientation.z;
        now_Q.w() = m_odomPath.poses[size-1].pose.orientation.w;
        Eigen::Vector3d now_t;
        now_t(0) =  m_odomPath.poses[size-1].pose.position.x;
        now_t(1) =  m_odomPath.poses[size-1].pose.position.y;
        now_t(2) =  m_odomPath.poses[size-1].pose.position.z;

        Eigen::Quaterniond last_Q;
        last_Q.x() = m_odomPath.poses[size-2].pose.orientation.x;
        last_Q.y() = m_odomPath.poses[size-2].pose.orientation.y;
        last_Q.z() = m_odomPath.poses[size-2].pose.orientation.z;
        last_Q.w() = m_odomPath.poses[size-2].pose.orientation.w;
        Eigen::Vector3d last_t;
        last_t(0) =  m_odomPath.poses[size-2].pose.position.x;
        last_t(1) =  m_odomPath.poses[size-2].pose.position.y;
        last_t(2) =  m_odomPath.poses[size-2].pose.position.z;

        Sophus::SE3 now_T(now_Q, now_t);
        Sophus::SE3 last_T(last_Q, last_t);
        Sophus::SE3 last_T_now = last_T.inverse() * now_T;
        Eigen::Vector3d last_t_now = last_T_now.translation();
        assert(std::abs(last_t_now[2] - 0) < 1e-4 );
        Eigen::Quaterniond last_Q_now = last_T_now.unit_quaternion();

        tf::Quaternion last_Q_now_tf(last_Q_now.x(), last_Q_now.y(), last_Q_now.z(), last_Q_now.w());
        d_pos[2] = tf::getYaw(last_Q_now_tf);
        d_pos[0] = last_t_now[0];
        d_pos[1] = last_t_now[1];
        //degree regular
        if(d_pos[2]>M_PI)
            d_pos[2] = -M_PI + d_pos[2]-floor(d_pos[2]/M_PI)*M_PI;
        if(d_pos[2]<-M_PI)
            d_pos[2] = M_PI + d_pos[2]+floor(d_pos[2]/(-M_PI))*M_PI;

        assert(d_pos[2] <= M_PI && d_pos[2] >= -M_PI);
        return d_pos;
    }
    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        if(m_isFirstFrame == true)
            return;
//        std::cout<<"odom callback,  "<<"time stamp = "<<msg->header.stamp<<std::endl;
        pubPath(msg, m_odomPath, m_odomPathPub);
    }

    //发布路径消息
    void pubPath(Eigen::Vector3d& pose, nav_msgs::Path &path, ros::Publisher &mcu_path_pub_)
    {
        ros::Time current_time = ros::Time::now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = pose(0);
        this_pose_stamped.pose.position.y = pose(1);

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        mcu_path_pub_.publish(path);
    }

    void pubPath(const nav_msgs::OdometryConstPtr& msg, nav_msgs::Path &path, ros::Publisher &mcu_path_pub_)
    {
        ros::Time current_time = ros::Time::now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
        this_pose_stamped.pose.position.y = msg->pose.pose.position.y;

        this_pose_stamped.pose.orientation.x = msg->pose.pose.orientation.x;
        this_pose_stamped.pose.orientation.y = msg->pose.pose.orientation.y;
        this_pose_stamped.pose.orientation.z = msg->pose.pose.orientation.z;
        this_pose_stamped.pose.orientation.w = msg->pose.pose.orientation.w;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        mcu_path_pub_.publish(path);
    }

    bool m_isFirstFrame;
    ros::NodeHandle m_nh;
    IMLSICPMatcher m_imlsMatcher;
    CsmMatcher m_csmMatcher;
    Eigen::Vector3d m_prevLaserPose;
    std::vector<Eigen::Vector2d> m_prevPointCloud;
    nav_msgs::Path m_imlsPath;
    nav_msgs::Path m_odomPath;

    tf::TransformListener m_tfListener;
    ros::NodeHandle m_node;

    ros::Subscriber m_laserscanSub;
    ros::Publisher m_imlsPathPub;
    ros::Publisher m_odomPathPub;

    std::string match_method;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imls_debug");

    imlsDebug imls_debug;

    ros::spin();

    return (0);
}

