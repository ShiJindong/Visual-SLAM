//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

int main(int argc, char **argv) {
    
    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    double x, y, z;
    
    ifstream fin_p3d(p3d_file);
    if (!fin_p3d)
    {
        cerr<<"请在有p3d.txt的目录下运行此程序"<<endl;
        return 1;
    }
    while(!fin_p3d.eof()){
        fin_p3d>> x >> y >> z;         
        p3d.push_back(Vector3d(x, y, z));
    }
    
    ifstream fin_p2d(p2d_file);
    if (!fin_p2d)
    {
        cerr<<"请在有p2d.txt的目录下运行此程序"<<endl;
        return 1;
    }
    while(!fin_p2d.eof()){
        fin_p2d >> x >> y;         
        p2d.push_back(Vector2d(x, y));
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;
    
    Sophus::SE3 T_esti = Sophus::SE3(Matrix3d::Identity(), Vector3d(1,0,0));       // 初始化估计的位姿

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();
        cost = 0;
        
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 
            Vector4d p3d_homog = Vector4d(p3d[i](0), p3d[i](1), p3d[i](2), 1);       // 将p3d[i]转换为齐次坐标
            Vector4d p3d_rot = T_esti.matrix() * p3d_homog;                          // p3d_rot为利用估计位姿T_esti进行旋转后的3D点的齐次坐标
            double Xr = p3d_rot(0);                             // Xr, Yr, Zr为旋转后的3D点的坐标
            double Yr = p3d_rot(1);
            double Zr = p3d_rot(2);
            Vector3d u =  K * Vector3d(Xr, Yr, Zr);             // 计算重投影位置u的坐标
            u = Vector3d(u(0)/u(2), u(1)/u(2), 1);              // 将其转化为齐次坐标
            Vector2d error = p2d[i] - Vector2d(u(0), u(1));     // 计算第i个数据点的重投影误差
            // END YOUR CODE HERE

            // compute jacobian
            // START YOUR CODE HERE 
            Matrix<double, 2, 6> J;
            J << fx/Zr, 0, -fx*Xr/(Zr*Zr), -fx*Xr*Yr/(Zr*Zr), fx+fx*Xr*Xr/(Zr*Zr), -fy*Yr/Zr,
                  0, fy/Zr, -fy*Yr/(Zr*Zr), -fy-fy*Yr*Yr/(Zr*Zr), fy*Xr*Yr/(Zr*Zr), fy*Xr/Zr;
            J = -J;                                             // 雅可比矩阵取负号，因为观测值-估计值
	       // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * error;
            cost += error.squaredNorm()/2;
        }

	// solve dx 
        // START YOUR CODE HERE 
        Vector6d dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti = Sophus::SE3::exp(dx) * T_esti;      // 更新估计的位姿
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
    
    
}
