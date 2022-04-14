#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>

/****************************
* 本程序完成第2次作业第4题: 几何运算练习
****************************/

int main(int argc, char **argv) {
    Eigen::Matrix<double,3,1> vC;           // 相机传感器系下的点 vC = [0.3, 0.2, 1.2]
    vC << 0.3, 0.2, 1.2;
    
    // 机器人系R 到 世界系W
    Eigen::Quaterniond qWR(0.55, 0.3, 0.2, 0.2);       // 注意Eigen中四元数赋值的顺序，实数w在首；但是实际上它的内部存储顺序是[x y z w]。实际上利用q.coeffs()输出系数的时候也是按内部存储顺序输出
    qWR = qWR.normalized();                // 要注意的是，只有单位四元数才表示旋转矩阵，所以要先对四元数做单位化
    Eigen::Matrix<double,3,1> tWR;
    tWR<<0.1, 0.2, 0.3;
    
    // 本体系B 到 机器人系R
    Eigen::Quaterniond qRB(0.99, 0, 0, 0.01);
    qRB = qRB.normalized();
    Eigen::Matrix<double,3,1> tRB;
    tRB<<0.05, 0, 0.5;
    
    // 激光传感器系L 到 本体系B
    Eigen::Quaterniond qBL(0.3, 0.5, 0, 20.1);
    qBL = qBL.normalized();
    Eigen::Matrix<double,3,1> tBL;
    tBL<<0.4, 0, 0.5;
    
    // 相机传感器系C 到 本体系B
    Eigen::Quaterniond qBC(0.8, 0.2, 0.1, 0.1);
    qBC = qBC.normalized();
    Eigen::Matrix<double,3,1> tBC;
    tBC<<0.5, 0.1, 0.5;
    
    // 使用四元数进行旋转(相当于旋转矩阵)，使用平移向量进行平移
    cout << "use quaternion: " << endl;
    Eigen::Matrix<double,3,1> vB = Eigen::Matrix<double,3,1>::Zero();         // 求Body系下的点 vB
    vB = qBC*vC + tBC;
    cout << "vB = " << vB.transpose()<< endl;
    Eigen::Matrix<double,3,1> vL = Eigen::Matrix<double,3,1>::Zero();         // 求激光系下的点 vL
    vL = qBL.inverse()*(vB - tBL);
    cout << "vL = " << vL.transpose()<< endl;
    Eigen::Matrix<double,3,1> vR = Eigen::Matrix<double,3,1>::Zero();         // 求机器人系下的点 vR
    vR = qRB*vB + tRB;
    cout << "vR = " << vR.transpose()<< endl;
    Eigen::Matrix<double,3,1> vW = Eigen::Matrix<double,3,1>::Zero();         // 求世界系下的点 vW
    vW = qWR*vR + tWR;
    cout << "vW = " << vW.transpose()<< endl << endl;
    
    // 使用欧式转换对上述利用四元数的坐标轴转换结果进行验证
    cout << "use Isometry: " << endl;
    Eigen::Isometry3d TBC = Eigen::Isometry3d::Identity();                    // 求Body系下的点 vB
    TBC.rotate(qBC.toRotationMatrix());
    TBC.pretranslate(tBC);
    vB = TBC*vC;
    cout << "vB = " << vB.transpose()<< endl;
    Eigen::Isometry3d TBL = Eigen::Isometry3d::Identity();                    // 求激光系下的点 vL
    TBL.rotate(qBL.toRotationMatrix());
    TBL.pretranslate(tBL);
    vL = TBL.inverse()*vB;
    cout << "vL = " << vL.transpose()<< endl;
    Eigen::Isometry3d TRB = Eigen::Isometry3d::Identity();                    // 求机器人系下的点 vR
    TRB.rotate(qRB.toRotationMatrix());
    TRB.pretranslate(tRB);
    vR = TRB*vB;
    cout << "vR = " << vR.transpose()<< endl;
    Eigen::Isometry3d TWR = Eigen::Isometry3d::Identity();                    // 求世界系下的点 vW
    TWR.rotate(qWR.toRotationMatrix());
    TWR.pretranslate(tWR);
    vW = TWR*vR;
    cout << "vW = " << vW.transpose()<< endl;
    
    return 0;
}
