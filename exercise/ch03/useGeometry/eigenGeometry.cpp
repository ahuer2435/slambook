#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc,char **argv)
{
  //旋转向量
  Eigen::AngleAxisd rotation_vector(M_PI/2,Eigen::Vector3d(0,1,1));	//旋转向量为（0,0，M_PI/2）
  cout.precision(3);
  
  //旋转向量转化为旋转矩阵
  Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();			//和rotation_vector.matrix()相同
  cout << "rotation_matrix: \n" <<  rotation_matrix << endl;
  
  //旋转向量转化为欧拉角
  Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);		//ZYX顺序
  cout << "yaw pitch roll = " << euler_angles.transpose() << endl;
  
  //分别通过旋转向量和旋转矩阵生成四元数。
  Eigen::Quaterniond quad = Eigen::Quaterniond(rotation_vector);
  cout <<"quad = " << quad.coeffs().transpose() << endl;		//0 0 0.707 0.707(i,j,k,s),实部在最后，但并不总是如此。
  quad = Eigen::Quaterniond(rotation_matrix);
  cout <<"quad = " << quad.coeffs().transpose() << endl;		//0 0 0.707 0.707(i,j,k,s),实部在最后，但并不总是如此。
  
  //旋转向量设置欧式变换矩阵
  Eigen::Isometry3d tran_matrix = Eigen::Isometry3d::Identity();		//初始化为单位空间矩阵，然后在旋转平移。
  tran_matrix.rotate(rotation_vector);
  tran_matrix.pretranslate(Eigen::Vector3d(1,3,4));
  cout<<"tran_matrix = \n" << tran_matrix.matrix() <<endl;
    
  //通过旋转向量旋转点v
  Eigen::Vector3d v(1,0,0);
  Eigen::Vector3d v_rotated;
  v_rotated = rotation_vector*v;
  cout<<"(1,0,0) after rotation by rotation_vector = "<<v_rotated.transpose()<< endl;
  
  //通过旋转矩阵旋转点v
  v_rotated = rotation_matrix*v; 
  cout<<"(1,0,0) after rotation by rotation_matrix = "<<v_rotated.transpose()<< endl;
  
  //通过四元数旋转点v
  v_rotated = quad*v;
  cout<<"(1,0,0) after rotation by quad = "<<v_rotated.transpose()<< endl;
  
  //通过变换矩阵旋转点v
  Eigen::Vector3d v_transformed;
  v_transformed = tran_matrix*v;
  cout<<"(1,0,0) after transformed by tran_matrix = "<<v_transformed.transpose()<< endl;
  
  
  return 0;
}