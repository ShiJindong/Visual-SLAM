#include <iostream>
using namespace std;
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include <Eigen/Cholesky>

#define MATRIX_SIZE 100
/****************************
* 编程实现第二节课习题2.5: 当A为100*100随机矩阵时，用QR和Cholesky分解求x
****************************/

int main(int argc, char **argv) {
    // 由于Eigen固定大小矩阵最大支持到50，故使用动态大小的矩阵
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > matrix_A;
    matrix_A = Eigen::MatrixXd::Random( MATRIX_SIZE, MATRIX_SIZE );
    
    // Cholesky分解求解要求A必须为对称正定矩阵，故我们对矩阵A做对称化处理   
    // 对称化
    matrix_A = matrix_A.transpose()*matrix_A;
    // 在实数范围内，对称性保证了矩阵的半正定性
    cout << "matrix_A = " << endl
         << matrix_A << endl;
         
    //计算特征值，来检验矩阵A的正定性
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver (matrix_A);
    cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
    
    
    // 定义向量b
    Eigen::Matrix< double, MATRIX_SIZE,  1> vector_b;
    vector_b = Eigen::MatrixXd::Random( MATRIX_SIZE,1 );
    
    // 利用QR分解求解
    Eigen::Matrix<double,MATRIX_SIZE,1> x_qr = matrix_A.colPivHouseholderQr().solve(vector_b);
    cout << "利用QR分解求解" << endl
         << "x_qr = " << endl
         << x_qr << endl;
    
    // 利用Cholesky分解求解      
    // Eigen::Matrix<double,MATRIX_SIZE,1> x_cholesky = matrix_A.llt().solve(vector_b);
    // llt()适用于正定矩阵，ldlt()适用于半正定矩阵/负半定矩阵，由于这里A为半正定矩阵，故用ldlt()更合适
    Eigen::Matrix<double,MATRIX_SIZE,1> x_cholesky = matrix_A.ldlt().solve(vector_b);
    cout << "利用Cholesky分解求解" << endl
         << "x_cholesky = " << endl
         << x_cholesky << endl;
         
    // 通过计算矩阵 e = x_qr - x_cholesky的长度来验证两种方法求得的解一致
    Eigen::Matrix<double,MATRIX_SIZE,1> e = x_qr - x_cholesky;
    double sum_e = 0;
    for (int i=0; i< MATRIX_SIZE; i++){
        sum_e += e(i)*e(i);
    }
    cout << "e_length = " << sqrt(sum_e) << endl;
    
    return 0;
}
