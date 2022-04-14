//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.h>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    // 对于square matric 与 thin matric的区别: Thin unitaries are only available if your matrix type has a Dynamic number of columns (for example MatrixXf)
    JacobiSVD<Matrix3d> svd(E, ComputeFullU | ComputeFullV);
    Vector3d sigular_values = svd.singularValues();
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    // 将奇异值转化为sigma矩阵
    Matrix3d sigma;
    sigma << (sigular_values(0,0) + sigular_values(1,0))/2, 0, 0,
              0, (sigular_values(0,0) + sigular_values(1,0))/2, 0,
              0, 0, 0;
    cout<<"sigma = " << endl << sigma << endl;
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d Rz = AngleAxisd(M_PI/2, Vector3d(0,0,1)).toRotationMatrix();
    Matrix3d Rz_minus = AngleAxisd(-M_PI/2, Vector3d(0,0,1)).toRotationMatrix();
    
    Matrix3d t_wedge1 = U * Rz * sigma * U.transpose();
    Matrix3d t_wedge2 = U * Rz_minus * sigma * U.transpose();;

    Matrix3d R1 = U * Rz.transpose() * V.transpose();
    Matrix3d R2 = U * Rz_minus.transpose() * V.transpose();;
    // END YOUR CODE HERE

    cout << "R1 = " << endl << R1 << endl;
    cout << "R2 = " << endl << R2 << endl;
    cout << "t1 = " << endl << Sophus::SO3::vee(t_wedge1) << endl;
    cout << "t2 = " << endl << Sophus::SO3::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << endl << tR << endl;
    cout << "E =  " << endl << E << endl;

    return 0;
}
