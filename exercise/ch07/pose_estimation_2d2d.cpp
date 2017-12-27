#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "extra.h"

#define OPENCV2 1
using namespace std;
using namespace cv;

void find_feature_matches(const Mat& img_1,const Mat& img_2,
			  std::vector<KeyPoint>& keypoints_1,
			  std::vector<KeyPoint>& keypoints_2,
			  std::vector<DMatch>& matches);

void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
			  std::vector<KeyPoint> keypoints_2,
			  std::vector<DMatch> matches,
			  Mat& R,Mat& t);

Point2d pixel2cam(const Point2d& p,const Mat& K);

void find_feature_matches(const Mat& img_1,const Mat& img_2,
			  std::vector<KeyPoint>& keypoints_1,
			  std::vector<KeyPoint>& keypoints_2,
			  std::vector<DMatch>& matches)
{
  Mat description_1,description_2;
#if OPENCV2
  Ptr<FeatureDetector> detector = FeatureDetector::create("ORB");
  Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create("ORB");
#else
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
#endif
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  
  detector->detect(img_1,keypoints_1);
  detector->detect(img_2,keypoints_2);
  
  descriptor->compute(img_1,keypoints_1,description_1);
  descriptor->compute(img_2,keypoints_2,description_2);
  
  vector<DMatch> src_match;
  matcher->match(description_1,description_2,src_match);
  
  double min_dist=10000,max_dist=0;
  for(int i = 0; i<description_1.rows;i++)
  {
    double dist = src_match[i].distance;
    if(dist < min_dist) min_dist = dist;
    if(dist > max_dist) max_dist = dist;
  }
  
  printf("max_dist = %lf\n",max_dist);
  printf("min_dist = %lf\n",min_dist);
  
  for(int i=0;i<description_1.rows;i++)
  {
    if(src_match[i].distance <= max(2*min_dist,30.0))
    {
      matches.push_back(src_match[i]);
    }
  }
}

//对照内参矩阵看下意义
Point2d pixel2cam(const Point2d& p, const Mat& K)
{
  return Point2d((p.x - K.at<double>(0,2))/K.at<double>(0,0),
		 (p.y - K.at<double>(1,2))/K.at<double>(1,1)
  );
}

void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
			  std::vector<KeyPoint> keypoints_2,
			  std::vector<DMatch> matches,
			  Mat& R,Mat& t)
{
  //内参矩阵与第五章的形式不一致？可以参考http://blog.csdn.net/liulina603/article/details/52953414
  Mat K = (Mat_<double>(3,3) << 520.9,325.1,
				0,521.0,249.7,
				0,0,1);
  vector<Point2f> points1;
  vector<Point2f> points2;
  //matches不能直接使用吗？
  for(int i=0;i<(int) matches.size();i++)
  {
    points1.push_back(keypoints_1[matches[i].queryIdx].pt);
    points2.push_back(keypoints_2[matches[i].queryIdx].pt);
  }
  
  Mat fundamental_matrix;
  //八点法计算基础矩阵
  fundamental_matrix = findFundamentalMat(points1,points2,CV_FM_8POINT);
  cout<<"fundamental_matrix is "<<endl<<fundamental_matrix<<endl;
  
  Point2d principal_point(325.1,249.7);		//相机光心，单位：pixel
  double focal_length = 521;			//相机焦距，单位：pixel
  Mat essential_matrix;
  essential_matrix = findEssentialMat(points1,points2,focal_length,principal_point);
  cout<<"esstial_matrix is "<<endl<<essential_matrix<<endl;
  
  Mat homography_matrix;
  homography_matrix = findHomography(points1,points2,RANSAC,3);
  cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;
  
  recoverPose(essential_matrix,points1,points2,R,t,focal_length,principal_point);
  recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
  cout<<"R is "<<endl<<R<<endl;
  cout<<"t is "<<endl<<t<<endl;
    
}

/*
 * 由两幅照片，估计相机的运动。
 */
int main(int argc,char **argv)
{
  if(argc !=3 )
  {
    cout << "usage:pose_estimation_2d2d img1 img2"<<endl;
    return 1;
  }
  Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread(argv[2],CV_LOAD_IMAGE_COLOR);
  
  vector<KeyPoint> keypoints_1,keypoints_2;
  vector<DMatch> matches;
  find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);
  cout<<"matches.size = "<<matches.size()<<endl;
  
  Mat R,t;
  pose_estimation_2d2d(keypoints_1,keypoints_2,matches,R,t);
 
//以下没看明白。  
  Mat t_x = (Mat_<double>(3,3) << 0,-t.at<double>(2,0),t.at<double>(1,0),
				  t.at<double>(2,0),0,-t.at<double>(0,0),
				  -t.at<double>(1,0),t.at<double>(0,0),0);
  cout << "t^R="<<endl<<t_x*R<<endl;
  
  Mat K = (Mat_<double>(3,3)<<520.9,0,325.1,
			      0,521.0,249.7,
			      0,0,1 );
  for(DMatch m: matches)
  {
    Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt,K);
    Mat y1 = (Mat_<double>(3,1) << pt1.x,pt1.y,1);
    Point2d pt2 = pixel2cam(keypoints_2[m.queryIdx].pt,K);
    Mat y2 = (Mat_<double>(3,1) << pt2.x,pt2.y,1);
    Mat d = y2.t() * t_x * R * y1;
    cout << "epipolar constraint = " << d << endl;
  }
  return 0;
}