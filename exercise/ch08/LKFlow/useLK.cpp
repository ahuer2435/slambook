#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
using namespace std; 

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

int main( int argc, char** argv )
{
  if ( argc != 2 )
  {
      cout<<"usage: useLK path_to_dataset"<<endl;
      return 1;
  }
  
  string path_to_dataset = argv[1];
  string associate_file = path_to_dataset + "/associate.txt";
  
  //读取深度图与彩色图的对应关系。
  ifstream fin( associate_file );
  if ( !fin ) 
  {
      cerr<<"I cann't find associate.txt!"<<endl;
      return 1;
  }
  
  string rgb_file, depth_file, time_rgb, time_depth;
  list< cv::Point2f > keypoints;      // 因为要删除跟踪失败的点，使用list
  cv::Mat color, depth, last_color;
  
  for ( int index=0; index<100; index++ )
  {
    fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
    //读取彩色图。
    color = cv::imread( path_to_dataset+"/"+rgb_file );
    //读取深度图。
    depth = cv::imread( path_to_dataset+"/"+depth_file, -1 );
    
     if (index ==0 )
     {
      vector<cv::KeyPoint> kps;
      //cout<<"1"<<endl; 调试FastFeatureDetector出现的错误。
      //获取第一幅图的角点。
      cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create("ORB");
      //cout<<"2"<<endl;
      detector->detect( color, kps );
      //cout<<"3"<<endl;
      //将角点转化为cv::Point2f格式，并保存在keypoints中。
      for ( auto kp:kps )
          keypoints.push_back( kp.pt );
      //cout<<"4"<<endl;
      last_color = color;
      continue;
     }
     
    vector<cv::Point2f> next_keypoints; 
    vector<cv::Point2f> prev_keypoints;
    //使用第一幅图设置prev_keypoints。
    for ( auto kp:keypoints )
        prev_keypoints.push_back(kp);
    vector<unsigned char> status;
    vector<float> error; 
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    //根据LK光流法计算出next_keypoints。
    cv::calcOpticalFlowPyrLK( last_color, color, prev_keypoints, next_keypoints, status, error );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"LK Flow use time："<<time_used.count()<<" seconds."<<endl;
    int i=0; 
    //遍历前一副图角点，根据status判断LK跟踪是否丢失，删除丢失点，将未丢失点保存到keypoints中。
    for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
    {
	if ( status[i] == 0 )
        {
            iter = keypoints.erase(iter);
            continue;
        }
        *iter = next_keypoints[i];
        iter++;
    }
    cout<<"tracked keypoints: "<<keypoints.size()<<endl;
    if (keypoints.size() == 0)
    {
        cout<<"all keypoints are lost."<<endl;
        break; 
    }
    // 画出 keypoints
    cv::Mat img_show = color.clone();
    for ( auto kp:keypoints )
        cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
    cv::imshow("corners", img_show);
    cv::waitKey(0);
    last_color = color;        
  }
  return 0;
}