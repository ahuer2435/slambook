#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>
#include <boost/concept_check.hpp>
using namespace std;

class CurveFittingVertex: public g2o::BaseVertex<3,Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl()
    {
      _estimate << 0,0,0;
    }
    
    virtual void oplusImpl(const double * update)
    {
      _estimate += Eigen::Vector3d(update);
    }
    virtual bool read(istream &in){}
    virtual bool write(ostream &out) const {}  //若少了const，则词类不可以实例化。
};

class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x):BaseUnaryEdge(), _x(x){}
    void computeError()
    {
      const CurveFittingVertex * v = static_cast<const CurveFittingVertex*>(_vertices[0]);
      const Eigen::Vector3d abc=v->estimate();
      _error(0,0) = _measurement - std::exp(abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0));
    }
    virtual bool read(istream & in){}
    virtual bool write(ostream & out) const {}	//若少了const，则词类不可以实例化。
  public:
    double _x;  
};

int main(int argc,char** argv)
{
  double a = 1.0, b = 2.0,c = 3.0;
  int N = 100;
  double w_sigma = 1.0;
  cv::RNG rng;			//opencv 随机数产生器
  double abc[3] = {0,0,0};
  
  vector<double> x_data,y_data;
  
  //生成观察值x_data和y_data
  cout<<"generating data: "<<endl;
  for(int i=0; i < N; i++)
  {
    double x = i/100.0;
    x_data.push_back(x);
    y_data.push_back(exp(a*x*x + b*x + c) + rng.gaussian(w_sigma));
    cout<<x_data[i]<<" "<<y_data[i]<<endl;
  }
  
  //构建图优化
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> Block;
  //构建线性方程求解器linearSolver
  Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
  
  //由线性方程求解器构建矩阵块求解器solver_ptr
  Block *solver_ptr = new Block(linearSolver);
  
  //由矩阵块求解器构建梯度下降器solver，梯度下降器有GN, LM, DogLeg三种类型。
  g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  //g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  //g2o::OptimizationAlgorithmDogleg *solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);
 
  //创建图模型。
  g2o::SparseOptimizer optimizer;
  //使用梯度下降器设置图模型的求解器
  optimizer.setAlgorithm(solver);
  //打开调试输出
  optimizer.setVerbose(true);
  
  //向图模型添加顶点。
  CurveFittingVertex* v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(0,0,0));
  v->setId(0);
  optimizer.addVertex(v);
  
  //向图模型添加边
  for(int i=0;i<N;i++)
  {
    CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
    edge->setId(i);
    edge->setVertex(0,v);	//设置链接的顶点
    edge->setMeasurement(y_data[i]);	//观测值。
    edge->setInformation(Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma)); //信息矩阵，协方差矩阵之逆。
    optimizer.addEdge(edge);
  }
  
  cout<<"start optimization."<<endl;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();	//计时
  //初始化优化器
  optimizer.initializeOptimization();
  //启动优化器，参数100是什么？
  optimizer.optimize(100);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
  cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;   //统计时间
  
  //输出优化后的值
  Eigen::Vector3d abc_estimate = v->estimate();
  cout<<"estimate model: "<<abc_estimate.transpose()<<endl;
  
  return 0;
}
