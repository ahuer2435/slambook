#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"

int main(int argc, char ** argv)
{
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI,Eigen::Vector3d(0,0,1)).toRotationMatrix();
  Eigen::Quaterniond q(R);
  
  //使用旋转矩阵和四元数构造李群。
  Sophus::SO3 SO3_R(R);
  Sophus::SO3 SO3_V(0,0,M_PI);
  Sophus::SO3 SO3_Q(q);
  
  cout << "R: \n" << R << endl << endl;
  
  //输出SO(3)时，是以so(3)的形式输出。但是SO(3)应该时李群，是一个旋转矩阵。
  //根据输出可以看到李代数个旋转向量是一致的。
  cout << "SO3_R = " << SO3_R << endl;
  cout << "SO3_V = " << SO3_V << endl;
  cout << "SO3_Q = " << SO3_Q << endl;
  
  //对李群使用对数映射，获取对应的李代数so3。
  Eigen:: Vector3d so3 = SO3_R.log();
  cout << "so3 = " << so3.transpose() << endl;
  cout << "so3 hat = \n" << Sophus::SO3::hat(so3) << endl;				//hat和vee是SO3类的static函数。
  cout << "so3 vee = \n" << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose() << endl;
  
  //这部分是扰动模型，没有搞明白其意义
  Eigen::Vector3d update_so3(1e-4, 0 ,0);
  Sophus::SO3 SO3_update = Sophus::SO3::exp(update_so3)*SO3_R;
  cout << "SO3 update = " << SO3_update << endl; 
  
  /***************************下面是对SE(3)操作******************************/
  cout<<"*******SE(3)操作************"<<endl;
  Eigen::Vector3d t(1,0,0);
  Sophus::SE3 SE3_Rt(R,t);
  Sophus::SE3 SE3_qt(q,t);
  cout << "SE3 from R,t = "<<endl<<SE3_Rt<<endl;	//对于SE(3),前三个量是so(3)，后３个量是位移。
  cout << "SE3 from R,q = "<<endl<<SE3_qt<<endl;
  
  typedef Eigen::Matrix<double,6,1> Vector6d;
  Vector6d se3 = SE3_Rt.log();				//指数映射运算
  cout << "se3 = "<< se3.transpose()<<endl;		//顺序和SE(3)相反，位移在前，so(3)在后。
  cout << "se3 hat = \n" << Sophus::SE3::hat(se3) << endl;				//hat和vee是SO3类的static函数。
  cout << "se3 vee = \n" << Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose() << endl;
  
  //SE(3)的扰动模型，不明白有啥意义。
  Vector6d update_se3;
  update_se3.setZero();
  update_se3(0)=e-4;
  Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3)*SE3_Rt;
  cout<<"SE3 update = "<< SE3_updated << endl;
  
  
  
  return 0;
}