#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <string>
#include <stdint.h>
#include <opencv2/opencv.hpp>

#include <unordered_set>
#include <memory>
#include <vector>
#include <stdlib.h> 

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/sparse_optimizer.h"

#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"

#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/stuff/sampler.h"

#include <sophus/se3.h>

using namespace Eigen;
using namespace std;

typedef Eigen::Matrix< double, 6, 1 > Vector6d;
typedef Eigen::Matrix< double, 9, 1 > Vector9d;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;
typedef g2o::BlockSolver<g2o::BlockSolverTraits<9,3> > BalBlockSolver;

void World2Camera(const Vector9d cam_vec, const Eigen::Vector3d Pw, Eigen::Vector3d& Pc);
void SolveBALProblem(const string filename);
inline void CamProjectionWithDistortion(const Vector9d cam_vec, const Vector3d point, Vector2d& u);

//定义相机顶点，含9个参数: R, t, f, k1, k2
class VertexCameraBAL : public g2o::BaseVertex<9, Vector9d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL() {}
    virtual bool read ( std::istream& in ) { return false;}
    virtual bool write ( std::ostream& out ) const {return false;}

    virtual void setToOriginImpl() {}

    virtual void oplusImpl ( const double* update )
    {
	// VectorXd::ConstMapType v ( update, VertexCameraBAL::Dimension );
        Vector9d::ConstMapType v ( update );
	//注意: g2o是旋转在前，平移在后，而se3是平移在前，旋转在后     **********
	Vector6d update_se3;
    update_se3 << v[3], v[4], v[5], v[0], v[1], v[2];           //从相机位姿顶点获取变换矩阵的李代数的更新量
	//获取当前变换矩阵的李代数的更新量
	Vector6d estimate_se3;
    estimate_se3 <<_estimate[3], _estimate[4], _estimate[5], _estimate[0], _estimate[1], _estimate[2];  
	//通过左乘更新量计算更新后的变换矩阵的SE3
	Sophus::SE3 after_update_SE3 = Sophus::SE3::exp(update_se3) * Sophus::SE3::exp(estimate_se3);
	//转化为李代数
	Vector6d after_update_se3 = after_update_SE3.log();
	// 更新估计
	Vector9d camera_update;
	camera_update << after_update_se3[3], after_update_se3[4],  after_update_se3[5], 
		             after_update_se3[0], after_update_se3[1],  after_update_se3[2], 
		             _estimate[6] + v[6], _estimate[7] + v[7],  _estimate[8] + v[8];
	_estimate = camera_update;
    }
};

//定义3D路标点顶点，含3个参数: x, y, z
class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL() {}

    virtual bool read ( std::istream& in ) { return false;}
    virtual bool write ( std::ostream& out ) const {return false;}

    virtual void setToOriginImpl() {}

    virtual void oplusImpl ( const double* update )
    {
        Eigen::Vector3d::ConstMapType v ( update );
        _estimate += v;
    }
};

// 定义边, 观测值为两维
class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, Eigen::Vector2d,VertexCameraBAL, VertexPointBAL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeObservationBAL() {}

    virtual bool read ( std::istream& in ) {return false;}
    virtual bool write ( std::ostream& out ) const {return false;}

    //计算误差
    virtual void computeError() override
    {
        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> (vertex(0));
        const VertexPointBAL* point = static_cast<const VertexPointBAL*> (vertex(1));

        Eigen::Vector2d predictions;

        CamProjectionWithDistortion(cam->estimate(), point->estimate(), predictions);

	// 注意定义的error为测量值减去估计值
        _error[0] = _measurement[0] - predictions[0];
        _error[1] = _measurement[1] - predictions[1];
    }

    // 自定义雅可比
    virtual void linearizeOplus() override
    {
        
        
        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> (vertex(0));
        const VertexPointBAL* point = static_cast<const VertexPointBAL*> (vertex(1));

        Eigen::Vector3d Pw;
        Eigen::Vector3d Pc;
        Vector9d camera_vec;

        Pw = point->estimate();

        camera_vec = cam->estimate();
        World2Camera(camera_vec,Pw,Pc);

        double x =  Pc[0];
        double y =  Pc[1];
        double z =  Pc[2];
        double f = camera_vec[6];
        double k1 = camera_vec[7];
        double k2 = camera_vec[8];

        double x_2 = x * x;
        double y_2 = y * y;
        double z_2 = z * z;
        double z_3 = z * z * z;
        double n_2 = (x * x + y * y) / (z * z);
        double n_4 = n_2 * n_2;
        double q = 1 + k1 * n_2 + k2 * n_2 * n_2;
        double w = k1 + 2 * k2 * n_2;

        // 计算雅可比(由3项合成)
        Eigen:: Matrix<double, 2, 3> J_ep;
        J_ep(0, 0) = -f * q / z - 2 * f * x_2 * w / z_3;
        J_ep(0, 1) = -f * x / z_3 * 2 * y * w;
        J_ep(0, 2) = f * x / z_2 * (q + 2 * n_2 * w);
        J_ep(1, 0) = -f * x / z_3 * y * w;
        J_ep(1, 1) = -f * q / z - 2 * f * y_2 * w / z_3;
        J_ep(1, 2) = f * y / z_2 * (q + 2 * n_2 * w);
        J_ep = (-1) * J_ep;
        
        Eigen::Matrix3d P_hat;
        P_hat << 0, (-1) * z, y, z, 0, (-1) * x, (-1) * y, x, 0;
        Eigen::Matrix<double, 3, 6> J_p_psi;
        
        // 注意: g2o定义的se3旋转在前，平移在后 !!!                                           
        J_p_psi << (-1) * P_hat, Eigen::Matrix3d::Identity();


        Eigen::Matrix<double, 2, 6> J_e_psi;
        J_e_psi = J_ep * J_p_psi;
        
        Eigen:: Matrix<double, 2, 1> J_ef;
        J_ef(0, 0) = x / z * q;
        J_ef(1, 0) = y / z * q;
        
        Eigen:: Matrix<double, 2, 1> J_ek1;
        J_ek1(0, 0) = f * x * n_2 / z;
        J_ek1(1, 0) = f * y * n_2 / z;
        
        Eigen:: Matrix<double, 2, 1> J_ek2;
        J_ek2(0, 0) = f * x * n_4 / z;
        J_ek2(1, 0) = f * y * n_4 / z;

	    _jacobianOplusXi.block<2,6>(0,0) = J_e_psi;
        _jacobianOplusXi.block<2,1>(0,6) = J_ef;
        _jacobianOplusXi.block<2,1>(0,7) = J_ek1;
        _jacobianOplusXi.block<2,1>(0,8) = J_ek2;

        _jacobianOplusXj = J_ep * Sophus::SO3::exp(Vector3d(camera_vec[0], camera_vec[1], camera_vec[2])).matrix();
    }
};


//加载BAL的文本文件
class LoadBALProblem
{
public:
    LoadBALProblem( string filename ):filename_(filename) {}
    ~LoadBALProblem()
    {
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] observations_cameras;
        delete[] observations_points;
    }
    
    int num_cameras()        const{ return num_cameras_;     }
    int num_points()         const{ return num_points_;      }
    int num_observations()   const{ return num_observations_;}

    double num_observations_cameras(int index)   { return observations_cameras[index];}
    double num_observations_points(int index)   { return observations_points[index];}
    double num_observations_uv(int index)   { return observations_[index];}

    int point_index(int index)   { return point_index_[index];}
    int camera_index(int index)   { return camera_index_[index];}

   
    void ReadData();
    void WriteToPLYFile(const std::string& filename);
    double* ReWritePoses(){return observations_cameras;}
    double* ReWritePoints(){return observations_points;}

    void Camera2World(const Eigen::Vector3d angleAxis, const Eigen::Vector3d Pc, Eigen::Vector3d& Pw);

private:
    int num_cameras_;
    int num_points_;
    int num_observations_;
    int observations_cameras_;
    int observations_points_;
    string filename_;

    int* point_index_;
    int* camera_index_;
    double* observations_;
    double* observations_cameras;
    double* observations_points;
};



void SolveBALProblem(const string filename)
{
    LoadBALProblem loadBALProblem(filename);
    loadBALProblem.ReadData();

    loadBALProblem.WriteToPLYFile("../initial.ply");

    //创建稀疏对象
    g2o::SparseOptimizer optimizer;

    //稀疏求解器
    g2o::LinearSolver<BalBlockSolver::PoseMatrixType>* linearSolver =
            new g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>();
    dynamic_cast<g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>* >(linearSolver)->setBlockOrdering(true);

    //矩阵块求解器
    BalBlockSolver* solver_ptr = new BalBlockSolver(linearSolver);

    //LM算法
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    //增加相机顶点
    const int num_cam = loadBALProblem.num_cameras();
    for(int i = 0; i < num_cam; i++)
    {
        Vector9d temVecCamera;
        for (int j = 0; j < 9; j++)
        {
            temVecCamera[j] = loadBALProblem.num_observations_cameras(9*i+j);
        }
        VertexCameraBAL* pCamera = new VertexCameraBAL();
        pCamera->setEstimate(temVecCamera);
        pCamera->setId(i);
        optimizer.addVertex(pCamera);

    }

    //增加路标顶点
    const int point_num = loadBALProblem.num_points();
    for(int i = 0; i < point_num; i++)
    {
        Vector3d temVecPoint;
        for (int j = 0; j < 3; j++)
        {
            temVecPoint[j] = loadBALProblem.num_observations_points(3*i+j);
        }
        VertexPointBAL* pPoint = new VertexPointBAL();
        pPoint->setEstimate(temVecPoint);

        //这里加上pose的数量,避免跟pose的ID重复
        pPoint->setId(i + num_cam);
        pPoint->setMarginalized(true);
        optimizer.addVertex(pPoint);
    }
    
    //增加边
    const int  num_observations =loadBALProblem.num_observations();
    for(int i = 0; i < num_observations; ++i)
    {
        EdgeObservationBAL* bal_edge = new EdgeObservationBAL();
        
        const int camera_id = loadBALProblem.camera_index(i);
        const int point_id = loadBALProblem.point_index(i) + num_cam;

        //使用鲁棒核函数Huber
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(1.0);
        bal_edge->setRobustKernel(rk);


        bal_edge->setVertex(0,dynamic_cast<VertexCameraBAL*>(optimizer.vertex(camera_id)));
        bal_edge->setVertex(1,dynamic_cast<VertexPointBAL*>(optimizer.vertex(point_id)));
        bal_edge->setInformation(Eigen::Matrix2d::Identity());
        bal_edge->setMeasurement(Eigen::Vector2d(loadBALProblem.num_observations_uv(2*i + 0),
                                                 loadBALProblem.num_observations_uv(2*i + 1)));

        optimizer.addEdge(bal_edge);
    }

    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(20);

    double* cameras = loadBALProblem.ReWritePoses();
    for(int i = 0; i < num_cam; i++)
    {
        VertexCameraBAL* pCamera = dynamic_cast<VertexCameraBAL*>(optimizer.vertex(i));
        Vector9d NewCameraVec = pCamera->estimate();
        memcpy(cameras + i * 9, NewCameraVec.data(), sizeof(double) * 9);
    }

    double* points = loadBALProblem.ReWritePoints();
    for(int j = 0; j < point_num; j++)
    {
        VertexPointBAL* pPoint = dynamic_cast<VertexPointBAL*>(optimizer.vertex(j + num_cam));
        Eigen::Vector3d NewPointVec = pPoint->estimate();
        memcpy(points + j * 3, NewPointVec.data(), sizeof(double) * 3);
    }

    loadBALProblem.WriteToPLYFile("../final.ply");
    cout<<"The Optimization of BA in Large has been finished!" <<endl;
}



void World2Camera(const Vector9d camera, const Eigen::Vector3d P_w, Eigen::Vector3d& P_c)
{
    Vector4d Pw(P_w[0],P_w[1],P_w[2],1.0);
    Sophus::Vector6d se3_RT;
    se3_RT << camera[3],camera[4], camera[5],camera[0],camera[1], camera[2];

    Vector4d P = Sophus::SE3::exp(se3_RT).matrix() * Pw;
    P_c[0] = P[0];
    P_c[1] = P[1];
    P_c[2] = P[2];
}


inline void CamProjectionWithDistortion(const Vector9d camera, const Vector3d point, Vector2d& u)
{

    Eigen::Vector3d p;
    World2Camera(camera, point, p);

    // Compute the center fo distortion
    double xp = -p[0]/p[2];
    double yp = -p[1]/p[2];

    // Apply second and fourth order radial distortion
    const double k1 = camera[7];
    const double k2 = camera[8];

    double r2 = xp*xp + yp*yp;
    double distortion = 1.0 + k1*r2 + k2*r2*r2 ;

    const double f = camera[6];
    u[0] = f * distortion * xp;
    u[1] = f * distortion * yp;

}

void LoadBALProblem::Camera2World(const Eigen::Vector3d angleAxis, const Eigen::Vector3d P_c, Eigen::Vector3d& P_w)
{
    cv::Mat Rcw;
    Sophus::Vector6d Tcw;
    Tcw <<P_c[0], P_c[1], P_c[2], angleAxis[0],angleAxis[1],angleAxis[2];
    Eigen::Matrix4d Twc;
    //Twc = Tcw^-1
    Twc = Sophus::SE3::exp(Tcw).matrix().inverse();
    P_w[0] = Twc(0,3);
    P_w[1] = Twc(1,3);
    P_w[2] = Twc(2,3);
}


void LoadBALProblem::WriteToPLYFile(const std::string& filename)
{
    std::ofstream of(filename.c_str());

    of<< "ply"
      << '\n' << "format ascii 1.0"
      << '\n' << "element vertex " << num_cameras_ + num_points_
      << '\n' << "property float x"
      << '\n' << "property float y"
      << '\n' << "property float z"
      << '\n' << "property uchar red"
      << '\n' << "property uchar green"
      << '\n' << "property uchar blue"
      << '\n' << "end_header" << std::endl;

    for(int i = 0; i < num_cameras(); ++i)
    {


        Eigen::Vector3d P_c;
        Eigen::Vector3d P_w;

        Eigen::Vector3d angleAxis(observations_cameras[9*i+0],
                                  observations_cameras[9*i+1],
                                  observations_cameras[9*i+2]);

        P_c[0] = observations_cameras[9*i+3];
        P_c[1] = observations_cameras[9*i+4];
        P_c[2] = observations_cameras[9*i+5];

        Camera2World(angleAxis,P_c, P_w);
        of << P_w[0] << ' ' << P_w[1] << ' ' << P_w[2]<< ' '
           << "0 255 0" << '\n';
    }

    // Export the structure (i.e. 3D Points) as white points.
    for(int i = 0; i < num_points(); ++i){
        for(int j = 0; j < 3; ++j)
        {
            of << observations_points[3*i+j] << ' ';
        }
        //加上颜色
        of << "255 255 255\n";
    }

    of.close();
}


void LoadBALProblem::ReadData()
{
    ifstream fin(filename_);
    if(!fin)
    {
        cout<< "Error opening .txt file"<< endl;
        return;
    }
    fin>>num_cameras_;
    fin>>num_points_;
    fin>>num_observations_;

    std::cout << "pose number: " << num_cameras_ <<endl;
    std::cout << "point number: " << num_points_  <<endl;
    std::cout << "observations number: " << num_observations_ <<endl;

    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];

    for (int i = 0; i < num_observations_; ++i)
    {
        fin>>camera_index_[i];
        fin>>point_index_[i];
        fin>>observations_[2*i];
        fin>>observations_[2*i+1];
    }

    observations_cameras_ = 9*num_cameras_;
    observations_points_ = 3*num_points_;
    observations_cameras = new double[observations_cameras_];
    observations_points = new double[observations_points_];

    for (int i = 0; i < observations_cameras_; ++i)
    {
        fin>>observations_cameras[i];
    }

    for (int i = 0; i < observations_points_; ++i)
    {
        fin>>observations_points[i];
    }
}



int main(int argc, char** argv)
{

    const string filename = "../data/problem-52-64053-pre.txt";
    SolveBALProblem(filename);

    return 0;
}















