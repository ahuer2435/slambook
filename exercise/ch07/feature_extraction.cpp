#include <iostream>
#include <boost/iterator/iterator_concepts.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#define OPENCV2 1
using namespace std;
using namespace cv;

int main(int argc,char **argv)
{
  if(argc != 3){
    cout << "usage: feature_extraction img1 img2"<<endl;
    return 1;
  }
  
  //读取图片，存储类型是Mat。
  Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread(argv[2],CV_LOAD_IMAGE_COLOR);
  
  //图片特征点使用std::vector<KeyPoint>来存储。
  std::vector<KeyPoint> keypoint_1,keypoint_2;
  //特征点的描述子使用Mat来存储，Mat存储矩阵形式。
  Mat descriptors_1,descriptors_2;
  
#if OPENCV2
  Ptr<FeatureDetector> detector = FeatureDetector::create("ORB");
  Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create("ORB");
#else
  
  //定义特征点检测器。
  Ptr<FeatureDetector> detector = ORB::create(1000);
  //定义特征点描述子提取器。
  Ptr<DescriptorExtractor> descriptor= ORB::create();
#endif
  //定义描述子匹配器。
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

  //调用特征点检测器设置特征点数据。
  detector->detect(img_1,keypoint_1);
  detector->detect(img_2,keypoint_2);
  cout << "keypoint_1: " <<keypoint_1.data()<<endl;
  
  //调用描述子提取器计算描述子。
  descriptor->compute(img_1,keypoint_1,descriptors_1);
  descriptor->compute(img_2,keypoint_2,descriptors_2);
  
  
  //以图片的形式显示特征点。使用drawKeypoints函数。
  Mat outimg1;
  drawKeypoints(img_1,keypoint_1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
  imshow("ORB特征点",outimg1);
  cv::waitKey(0);
  
  //定义描述子匹配数据使用vector<DMatch>存储。
  vector<DMatch> matches;
  matcher->match(descriptors_1,descriptors_2,matches);
  
  //计算匹配距离的最大值和最小值
  //为啥选用描述子的行数计算最短距离。
  double min_dist=10000;
  double max_dist=0;
  for(int i=0;i<descriptors_1.rows;i++){
    double dist = matches[i].distance;
    if(dist < min_dist){
      min_dist = dist;
    }
    if(dist > max_dist){
      max_dist = dist;
    }
  }
  
  //过滤匹配点。
  std::vector<DMatch> good_matches;
  for(int i=0;i<descriptors_1.rows;i++){
    if(matches[i].distance <= max(2*min_dist,30.0)){
      good_matches.push_back(matches[i]);
    }
  }
  
  //分别使用过滤前后的匹配数据画出图形。
  Mat img_match;
  Mat img_goodmatch;
  drawMatches(img_1,keypoint_1,img_2,keypoint_2,matches,img_match);
  drawMatches(img_1,keypoint_1,img_2,keypoint_2,good_matches,img_goodmatch);
  imshow("test1",img_match);
  imshow("test2",img_goodmatch);
  cv::waitKey(0); 
  
  return 0;
}