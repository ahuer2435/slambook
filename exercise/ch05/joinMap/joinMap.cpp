#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>
#include <boost/format.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
  vector<cv::Mat> colorImages,depthImages;	//vector存储多个图片，每个图片使用cv::Mat格式存储。
  vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>> poses;		//相机位姿存储结构
  
  //读取位姿配置文件。
  ifstream fin("./pose.txt");
  if(!fin){
    cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
    return 1;
  }
  
  //设置colorImages，depthImages和poses。
  for(int i=0;i<5;i++){
    boost::format fmt("./%s/%d.%s");
    colorImages.push_back(cv::imread((fmt%"color"%(i+1)%"png").str()));
    depthImages.push_back(cv::imread((fmt%"depth"%(i+1)%"pgm").str(),-1));
    
    //利用位姿设置外参，即变换矩阵T。
    double data[7] = {0};
    for(auto& d:data){
      fin >> d;
    }
    Eigen::Quaterniond q(data[6],data[3],data[4],data[5]);
    Eigen::Isometry3d T(q);
    T.pretranslate(Eigen::Vector3d(data[0],data[1],data[2]));
    poses.push_back(T);
  }
  
  //定义内参。cx和cy是像素数；fx,fy,depthScale 单位都是：像素数/米
  double cx = 325.5;	
  double cy = 253.5;
  double fx = 518.0;
  double fy = 519.0;
  double depthScale = 1000.0;	//
  
  cout<<"正在将图片转为点云..."<<endl;
  
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloud;
  
  //创建一个点云结构及指向其的指针。
  PointCloud::Ptr pointCloudPtr(new PointCloud);
  
  //填充点云数据结构
  for(int i = 0;i < 5;i++){
    cout<<"转换图像中: "<<i+1<<endl;
    cv::Mat color = colorImages[i];
    cv::Mat depth = depthImages[i];
    Eigen::Isometry3d T = poses[i];
    for(int v=0;v<color.rows;v++){
      for(int u=0;u<color.cols;u++){
	unsigned int d = depth.ptr<unsigned short>(v)[u];
	if(d ==0 ){
	  continue;
	}
	Eigen::Vector3d point;		//相机坐标系下的点
	//将图片坐标系下的点转化为相机坐标系下的点坐标point[3]。
	point[2] = double(d)/depthScale;	//z轴
	point[0] = (u-cx)*point[2]/fx;		//x轴
	point[1] = (v-cy)*point[2]/fy;		//y轴
	
	//将相机坐标系下的点坐标转化为世界坐标系下的坐标：pointWorld[3]
	Eigen::Vector3d pointWorld = T*point;
	
	//将世界坐标系下的点坐标转化为点云数据p。
	PointT p;
	p.x = pointWorld[0];
	p.y = pointWorld[1];
	p.z = pointWorld[2];
	p.b = color.data[v*color.step+u*color.channels()];
	p.g = color.data[v*color.step+u*color.channels()+1];
	p.r = color.data[v*color.step+u*color.channels()+2];
	pointCloudPtr->points.push_back(p);	//组合点云数据pointCloudPtr
      }
    }
  }
  

  pointCloudPtr->is_dense = false;	//不知什么作用。
  cout<<"点云共有"<<pointCloudPtr->size()<<"个云。"<<endl;
  pcl::io::savePCDFileBinary("map.pcd",*pointCloudPtr);    //将点云数据保存成图片。
  
  return 0;
}