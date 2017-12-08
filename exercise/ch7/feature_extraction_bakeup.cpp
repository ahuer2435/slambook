#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

#define ORB 1

int main(int argc,char **argv)
{
  if(argc != 3){
    cout << "usage: feature_extraction img1 img2"<<endl;
    return 1;
  }
  
  Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread(argv[2],CV_LOAD_IMAGE_COLOR);
  
  std::vector<KeyPoint> keypoint_1,keypoint_2;
  Mat descriptors_1,descriptors_2;

#if ORB
//Ptr<ORB> orb = ORB::create(500,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31,20);
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> = ORB::create();
#else
  Ptr<FeatureDetector> detector = FeatureDetector::create("Feature-Detector");
  Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create("Descriptor-Extractor");
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
#endif

#if ORB
  orb->detect(img_1,keypoint_1);
  orb->detect(img_2,keypoint_2);
#else 
  detector->detect(img_1,keypoint_1);
  detector->detect(img_2,keypoint_2);
#endif

  #if ORB
  orb->compute(img_1,keypoint_1,descriptors_1);
  orb->compute(img_2,keypoint_2,descriptors_2);
#else
  descriptor->compute(img_1,keypoint_1,descriptors_1);
  descriptor->compute(img_2,keypoint_2,descriptors_2);
#endif
  
  Mat outimg1;
  drawKeypoints(img_1,keypoint_1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
  imshow("ORB特征点",outimg1);
  cv::waitKey(0); 
  
  return 0;
}