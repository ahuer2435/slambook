#include <iostream>
#include <ctime>
using namespace std;
#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50
int main(int argc,char *argv[])
{
	Eigen::Matrix<float,2,3> matrix_23;
	Eigen::Vector3d v_3d;		//默认double型，且是列向量。
	Eigen::Matrix<float,3,1> vd_3d;
	Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
	
	Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> matrix_dynamic;
	Eigen::MatrixXd matrix_x;
	
	matrix_23 << 1,2,3,4,5,6;
	cout << matrix_23 << endl;

	int i,j;
	for(i < 0; i < 2; i++){
		for(j = 0; j < 3; j++){
			cout <<matrix_23(i,j) << "\t";
		}
		cout << endl;
	}

	v_3d << 3,2,1;
	vd_3d << 4,5,6;
	cout << "v_3d: " << endl;
	cout << v_3d << endl << endl; 			//确认是列向量。
	//Eigen::Matrix<double,2,1> result_wrong_type = matrix_23 * v_3d;
	Eigen::Matrix<double,2,1> result = matrix_23.cast<double>() * v_3d;
	cout << "result:" << endl;
	cout << result << endl << endl;

	Eigen::Matrix<float,2,1> result2 = matrix_23 * vd_3d;
	cout << "result2:" << endl;
	cout <<result2 << endl << endl;

	//Eigen::Matrix<double,2,3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

	matrix_33 = Eigen::Matrix3d::Random();
	cout << "matrix_33:" << endl;
	cout << matrix_33 <<endl<<endl;

	cout << "matrix_33.transpose(): " << endl;
	cout << matrix_33.transpose() << endl << endl;
	cout << "matrix_33.sum(): " << endl;
	cout << matrix_33.sum() << endl << endl;
	cout << "matrix_33.trace(): " << endl;     //矩阵的迹即为矩阵的主对角线元素之和，记为tr(matrix_33)，也是矩阵的特征值之和。
	cout << matrix_33.trace() << endl << endl;
	cout << "10*matrix_33: " << endl;
	cout << 10*matrix_33 << endl << endl;
	cout << "matrix_33.inverse():" << endl;
	cout << matrix_33.inverse() << endl << endl;
	cout << "matrix_33.determinant()" << endl;
	cout << matrix_33.determinant() << endl << endl;


	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver (matrix_33.transpose()*matrix_33);
	//cout << "eigen_solver = \n" <<  eigen_solver << endl;
	cout << "Eigen values = \n"<<eigen_solver.eigenvalues() << endl;
	cout << "Eigen vectors = \n"<<eigen_solver.eigenvectors() <<endl;

	Eigen::Matrix<double,MATRIX_SIZE,MATRIX_SIZE> matrix_NN;
	matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE,MATRIX_SIZE);
	Eigen::Matrix<double,MATRIX_SIZE,1> v_Nd;
	v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE,1);

	clock_t time_stt = clock();
	Eigen::Matrix<double,MATRIX_SIZE,1> x = matrix_NN.inverse()*v_Nd;
	cout << "time use in normal inverse is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms" <<endl;
	
	time_stt = clock();
	x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
	cout << "time use in normal inverse is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms" <<endl;
	return 0;
	
}