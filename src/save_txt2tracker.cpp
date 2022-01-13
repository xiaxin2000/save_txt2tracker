/*This rosnode is for generating the point cloud map given the pose (position&attitude) from RT3000. 
With this accurate pose, the current lidar point cloud will be transformed into the map coordinates. 
Then, the point cloud is stacked to generate the map.
If you have any questions, please feel free to contact Xin Xia (xiaxin2000@gmail.com)*/
#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <gps_common/GPSFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <boost/shared_ptr.hpp>


using namespace std;
using std::atan2;
using std::cos;
using std::sin;

using std::cout;
using std::endl;
using std::vector;
using std::setprecision;

std::string PathName_of_txt="/home/carma/";//needing to be changed to customized path
std::ofstream outFile;
int txt_start_=0;
double deg_rad=M_PI/180,rad_deg=180/M_PI;


/*Transform quaternion to euler angle*/
Eigen::Vector3f Quaternion2EulerAngle(Eigen::Quaternionf Quaternion_ToBeTransformed) //XX 四元数转化成欧拉角
{

	Eigen::Vector3f EulerAngle_FromQuaternion;
	Eigen::Vector4f q_coeffs = Quaternion_ToBeTransformed.coeffs();
	float yaw_from_Quaternion,pitch_from_Quaternion,roll_from_Quaternion,qw,qx,qy,qz;

	qw=q_coeffs[3];
	qx=q_coeffs[0];
	qy=q_coeffs[1];
	qz=q_coeffs[2];

	yaw_from_Quaternion=std::atan2(2*(qx*qy+qw*qz),(1-2*(qy*qy+qz*qz)));
	pitch_from_Quaternion=std::asin(2*(qw*qy-qz*qx));
	roll_from_Quaternion=std::atan2(2*(qw*qx+qy*qz),(1-2*(qx*qx+qy*qy)));

	EulerAngle_FromQuaternion[0]=yaw_from_Quaternion;
	EulerAngle_FromQuaternion[1]=pitch_from_Quaternion;
	EulerAngle_FromQuaternion[2]=roll_from_Quaternion;

	return EulerAngle_FromQuaternion;

}


/*Transform euler angle to quaternion*/
Eigen::Quaternionf EulerAngle2Quaternion(Eigen::Vector3f EulerAngle_ToBeTransformed)
{

	Eigen::Quaternionf Quaternion_FromEulerAngle;

	float yaw=EulerAngle_ToBeTransformed[0], pitch=EulerAngle_ToBeTransformed[1], roll=EulerAngle_ToBeTransformed[2];

	float cy = std::cos(yaw*0.5), sy = std::sin(yaw*0.5);
	float cp = std::cos(pitch*0.5), sp = std::sin(pitch*0.5);
	float cr = std::cos(roll*0.5), sr = std::sin(roll*0.5);

	Eigen::Vector3f v;
	v[0] = sr * cp * cy - cr * sp * sy;
	v[1] = cr * sp * cy + sr * cp * sy;
	v[2] = cr * cp * sy - sr * sp * cy;

	Quaternion_FromEulerAngle.w()=cr * cp * cy + sr * sp * sy;
	Quaternion_FromEulerAngle.vec() = v; 

	return Quaternion_FromEulerAngle;

}

/*Rotation matrix generation function*/
Eigen::Matrix3f Creat_Rt(Eigen::Vector3f euler_angle) 
{

	Eigen::Matrix3f Rz= Eigen::Matrix3f::Identity();
	Eigen::Matrix3f Ry= Eigen::Matrix3f::Identity();
	Eigen::Matrix3f Rx= Eigen::Matrix3f::Identity();

	float alpha = euler_angle(0), beta = euler_angle(1), gamma = euler_angle(2);
	Rz(0,0)=cos(alpha);
	Rz(0,1)=-sin(alpha);
	Rz(1,0)=sin(alpha);
	Rz(1,1)=cos(alpha);

	Ry(0,0)=cos(beta);
	Ry(0,2)=sin(beta);
	Ry(2,0)=-sin(beta);
	Ry(2,2)=cos(beta);

	Rx(1,1)=cos(gamma);
	Rx(1,2)=-sin(gamma);
	Rx(2,1)=sin(gamma);
	Rx(2,2)=cos(gamma);

	Eigen::Matrix3f Rt=Rz*Ry*Rx;
	return Rt;
}


void detect_obj_sub_callback(autoware_msgs::DetectedObjectArray input)
{
	
	txt_start_++;

	std::string type="Car";
	double truncated=0;
	double occluded=0;
	double alpha=0;
	double bbox_1=0;
	double bbox_2=0;
	double bbox_3=0;
	double bbox_4=0;
	double height=0;
	double width=0;
	double length=0;
	double location_x=0;
	double location_y=0;
	double location_z=0;
	double rotation_y=0;
	double score=0;

    std::stringstream ss;
    std::cout << "txt_start_ = " << txt_start_ << std::endl;
    ss << txt_start_;
    std::string tmp_str = ss.str()+ ".txt";
    PathName_of_txt = PathName_of_txt + tmp_str;

	outFile.open(PathName_of_txt,std::ios::out);

	for (auto object: input.objects)
	{

		Eigen::Quaternionf q_;
		q_.x()=object.pose.orientation.x;
		q_.y()=object.pose.orientation.y;
		q_.z()=object.pose.orientation.z;
		q_.w()=object.pose.orientation.w;
		Eigen::Vector3f euler_angle=Quaternion2EulerAngle(q_);
		rotation_y=-euler_angle(0);
		// std::cout<<"yaw av = "<<yaw<<std::endl;
		length=object.dimensions.x;
		width=object.dimensions.y;
		height=object.dimensions.z;
		location_x=object.pose.position.x;
		location_y=object.pose.position.x;
		location_z=object.pose.position.x;

		score=object.score;

   		outFile<<type<<' '
   				<<truncated<<' '
				<<occluded<<' ' 
				<<alpha<<' ' 
				<<bbox_1<<' ' 
				<<bbox_2<<' ' 
				<<bbox_3<<' ' 
				<<bbox_4<<' ' 
				<<height<<' ' 
				<<width<<' ' 
				<<length<<' ' 
				<<location_x<<' ' 
				<<location_y<<' ' 
				<<location_z<<' ' 
				<<rotation_y<<' ' 
				<<score<<' ' 
				<<endl;

	}
	outFile.close();
	std::cout<<"test saving txt data = "<<txt_start_<<std::endl;
	// outFile.close();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_txt2tracker");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    PathName_of_txt = private_nh.param<string>("PathName_of_txt", "/home/carma/save_txt2tracker/");

	ros::Subscriber detect_obj_in_map_sub = nh.subscribe("/detection/lidar_detector/objects", 100,detect_obj_sub_callback);

    ros::spin();

    return 0;
}
