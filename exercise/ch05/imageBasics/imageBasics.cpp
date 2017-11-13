#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char ** argv)
{
  cv::Mat image;
  image = cv::imread(argv[1]);
  
  if(image.data == nullptr){
    cerr << "文件" << argv[1] << "不存在" <<endl;
    return 0;
  }
  
  cout << "图像宽度： "<<image.cols<<"，高为：　" << image.rows<<",通道数为："<< image.channels()<<endl;
  cv::imshow("image_sample",image);
  cv::waitKey(0);	//手动关闭图片才会继续执行。
  
  //cout << "123" << endl;
  if(image.type() != CV_8UC1 && image.type() != CV_8UC3){
    cout << "请输入一张符合彩色图或者灰度图。"<<endl;
    return 0;
  }
  //cout << "456"<<endl;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for(size_t y = 0; y < image.rows; y++){
    //printf("y = %ld\n",y);
    unsigned char * row_ptr = image.ptr<unsigned char >(y);
    for(size_t x = 0; x < image.cols;x++){
      //printf("x = %ld\n",x);
      unsigned char* data_ptr = &row_ptr[x*image.channels()];
      for(int c = 0; c!= image.channels();c++){
	//printf("c = %d\n",c);
	unsigned char data = data_ptr[c];
      }
    }
  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "遍历图片用时："<<time_used.count()<<"秒。"<<endl;
  
  //直接复制，相当于指针操作，同步修改。
  cv::Mat image_another = image;
  image_another(cv::Rect(0,0,100,100)).setTo(0);
  cv::imshow("image2",image);
  cv::waitKey(0);
  cv::imshow("image_another",image_another);
  cv::waitKey(0);
  
  //使用clone相当于复制一份。
  cv::Mat image_clone = image.clone();
  image_clone(cv::Rect(0,0,50,50)).setTo(255);
  cv::imshow("image3",image);
  cv::waitKey(0);  
  cv::imshow("image_clone",image_clone);
  cv::waitKey(0);
  
  cv::destroyAllWindows();	//不太明白此函数的意义。
  
  
  return 0;
}
