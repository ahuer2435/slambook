#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

//仿函数类的定义。
struct CURVE_FITTING_COST
{
  CURVE_FITTING_COST(double x,double y): _x(x),_y(y){}
  
  template <typename T>
  bool operator()(const T* const abc, T* residual) const
  {
    residual[0] = T(_y) - ceres::exp(abc[0]*T(_x)*T(_x) + abc[1]*T(_x) + abc[2]);
    return true;
  }
  const double _x,_y;
};



int main(int argc, char **argv) {
    double a=1.0, b=2.0,c = 3.0;		//真实参数值
    int N=100;					//样本容量。
    double w_sigma=1.0;				//高斯分布函数参数，代表噪声Sigma值。
    cv::RNG rng;				//openCV随机数生成器
    double abc[3] = {0,0,0};			//参数估计值。
    
    //定义x_data,y_data容器。
    vector<double> x_data,y_data;
    cout << "generating data: "<<endl;
    
    //设置x_data,y_data容器，存放观察值。
    for(int i=0; i<N; i++)
    {
      double x=i/100.0;
      double gaussian_value = rng.gaussian(w_sigma);
      x_data.push_back(x);
      y_data.push_back(exp(a*x*x + b*x +c) + gaussian_value);
      cout<<x_data[i] <<" "<<y_data[i]<<" "<<gaussian_value<<endl;
    }
    
    //构建最小二成问题。
    //配置问题结构，主要是添加误差项，和求导方式；误差项由仿函数获得。
    ceres::Problem problem;
    for(int i = 0; i < N; i++)
    {
      problem.AddResidualBlock(				//向问题添加误差项。
	new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,1,3>(	//自动求导函数，在这里会调用仿函数实现，模板参数：仿函数类，输出维度（误差），输入维度（待估计值）
	  new CURVE_FITTING_COST(x_data[i],y_data[i])		//定义一个仿函数类的对象，调用构造函数，初始化的私有数据_x和_y.
	),			
	nullptr,						//核函数，不用，为nullptr。
	abc);							//待估计参数
    }
    
    //配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;	//增量方程解决方式。
    options.minimizer_progress_to_stdout = true;	//输出到stdout.
    
    ceres::Solver::Summary summary;		//优化信息。
    chrono::steady_clock::time_point t1=chrono::steady_clock::now();
    //优化器Solve需要传入三个参数，求解器，问题和记录
    //记录其实感觉只是一种调试作用。
    //开始优化
    //
    ceres::Solve(options,&problem,&summary);	//开始优化。
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"solve time cost = "<<time_used.count()<<" seconds."<<endl;
    
    cout<<summary.BriefReport()<<endl;		//打印优化信息。
    cout<<"estimated a,b,c = ";			//输出估计结果。
    for(auto a:abc){
      cout<<a<<" ";
    };
    cout<<endl;
    return 0;
}

