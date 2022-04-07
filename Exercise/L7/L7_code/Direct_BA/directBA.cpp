//
// Created by xiang on 1/4/18.
// this program shows how to perform direct bundle adjustment
//
#include <iostream>

using namespace std;

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <Eigen/Core>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>
#include <boost/format.hpp>


typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> VecSE3;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3d;



// global variables
string pose_file = "../data/poses.txt";
string points_file = "../data/points.txt";

// intrinsics
float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}


// g2o vertex that use sophus::SE3 as pose, 自定义路标3D坐标的结点VertexSophus
class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSophus() {}

    ~VertexSophus() {}

    bool read(std::istream &is) {return false;}

    bool write(std::ostream &os) const {return false;}

    virtual void setToOriginImpl() {
        _estimate = Sophus::SE3();                                          //将估计量设成SE3，方便后续利用Sophus进行计算
    }

    virtual void oplusImpl(const double *update_) {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);      // 将数组update_转化为Eigen的6x1矩阵update
        // 相当于Eigen::VectorXd::ConstMapType update(update_, VertexSophus::Dimension);
        setEstimate(Sophus::SE3::exp(update) * estimate());                 // 左乘更新的SE3
    }
};

// TODO edge of projection error, implement it
// 16x1 error, which is the errors in patch
typedef Eigen::Matrix<double,16,1> Vector16d;   // 特征点空间坐标结点：g2o::VertexSBAPointXYZ，来自 g2o/types/sba
class EdgeDirectProjection : public g2o::BaseBinaryEdge<16, Vector16d, g2o::VertexSBAPointXYZ, VertexSophus> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeDirectProjection(float *color, cv::Mat &target) {
        this->origColor = color;
        this->targetImg = target;
    }

    ~EdgeDirectProjection() {}

    virtual void computeError() override {
        // TODO START YOUR CODE HERE
        // compute projection error ..
        const g2o::VertexSBAPointXYZ* pPoint = static_cast<const g2o::VertexSBAPointXYZ*>(vertex(0));   //将vertex(0)转换为VertexSBAPointXYZ类的指针
        const VertexSophus* pCamera = static_cast<const VertexSophus*>(vertex(1));
        
        // Eigen::Vector3d q_ij = pCamera->estimate() * pPoint->estimate();    //经估计位姿变换后的3D坐标     // ***
        Eigen::Vector3d q_ij = pCamera->estimate().rotation_matrix() * pPoint->estimate() + pCamera->estimate().translation();
        Eigen::Matrix3d K;
        K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        Eigen::Vector3d u_ij = K * q_ij;
        float u = u_ij(0)/u_ij(2);
        float v = u_ij(1)/u_ij(2);
        
        if(u - 2 < 0 || u + 1 >= targetImg.cols || v -2 < 0 || v + 1 >= targetImg.rows){
            // 边界处理
            this->setLevel(1);   //将setLevel置为1
            for(int n = 0; n < EdgeDirectProjection::Dimension; n++)                                     // ***
                _error[n] = 0;
        }
        else{
            for(int x = -2; x <= 1; x++ )       //列
                for(int y = -2; y <= 1; y++){   //行
                    int index = (x + 2) * 4 + (y + 2);    //第y行第x列, 注意index的顺序是逐列读取的
                    _error[index] = origColor[index] - GetPixelValue(targetImg, u + x, v + y);           // ***
                }
        }
    } 
    
    //雅可比
    virtual void linearizeOplus() override {
        if(level() == 1){      //边界处理 
            _jacobianOplusXi = Eigen::Matrix<double, 16, 3>::Zero();        
            _jacobianOplusXj = Eigen::Matrix<double, 16, 6>::Zero();
            return;                                                                                 
        }
            
        const g2o::VertexSBAPointXYZ* pPoint = static_cast<const g2o::VertexSBAPointXYZ*>(vertex(0));  
        const VertexSophus* pCamera = static_cast<const VertexSophus*>(vertex(1));
        // Eigen::Vector3d q_ij = pCamera->estimate() * pPoint->estimate();    //经估计位姿变换后的3D坐标
        Eigen::Vector3d q_ij = pCamera->estimate().rotation_matrix() * pPoint->estimate() + pCamera->estimate().translation();
        Eigen::Matrix3d K;
        K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        Eigen::Vector3d u_ij = K * q_ij;
        float u = u_ij(0)/u_ij(2);
        float v = u_ij(1)/u_ij(2);
            
        float x = q_ij[0];
        float y = q_ij[1];
        float z = q_ij[2];
        float z2 = z * z;
            
        Eigen::Matrix<double, 2, 3> J_uq;
        J_uq(0, 0) = fx / z;
        J_uq(0, 1) = 0;
        J_uq(0, 2) = -fx * x / z2;
        J_uq(1, 0) = 0;
        J_uq(1, 1) = fy / z;
        J_uq(1, 2) = -fy * y / z2;
            
        Eigen::Matrix<double, 3, 6> J_qkesi;
        
        J_qkesi <<  1, 0, 0, 0,  z, -y, 
                    0, 1, 0, -z, 0,  x, 
                    0, 0, 1, y, -x,  0;                                   // ***
        
        
                        
        Eigen::Matrix<double, 1, 2> J_Iu;
        for(int x = -2; x <= 1; x++ )       //列
            for(int y = -2; y <= 1; y++){   //行
                int index = (x + 2) * 4 + (y + 2);    //第y行第x列
                J_Iu(0, 0) = (GetPixelValue(targetImg, u + x + 1, v + y) - GetPixelValue(targetImg, u + x - 1, v + y)) / 2;
                J_Iu(0, 1) = (GetPixelValue(targetImg, u + x, v + y + 1) - GetPixelValue(targetImg, u + x, v + y - 1)) / 2;
                // 注意: 雅可比总共16行，对应16个误差项，将雅可比表示成block的形式，第index个误差项位于(index, 0)处
                _jacobianOplusXi.block<1,3>(index, 0) = -J_Iu * J_uq * pCamera->estimate().rotation_matrix();
                _jacobianOplusXj.block<1,6>(index, 0) = -J_Iu * J_uq * J_qkesi;
            }
    }
    // Let g2o compute jacobian for you

    virtual bool read(istream &in) {return false;}

    virtual bool write(ostream &out) const {return false;}

private:
    cv::Mat targetImg;  // the target image
    float *origColor = nullptr;   // 16 floats, the color of this point
};

// plot the poses and points for you, need pangolin
void Draw(const VecSE3 &poses, const VecVec3d &points);



int main(int argc, char **argv) {

    
    // read poses and points
    VecSE3 poses;
    VecVec3d points;
    ifstream fin(pose_file);

    while (!fin.eof()) {
        double timestamp = 0;
        fin >> timestamp;
        if (timestamp == 0) break;
        double data[7];
        for (auto &d: data) fin >> d;
        poses.push_back(Sophus::SE3(
                Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                Eigen::Vector3d(data[0], data[1], data[2])
        ));
        if (!fin.good()) break;
    }
    fin.close();


    vector<float *> color;
    fin.open(points_file);
    while (!fin.eof()) {
        double xyz[3] = {0};
        for (int i = 0; i < 3; i++) fin >> xyz[i];
        if (xyz[0] == 0) break;
        points.push_back(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
        float *c = new float[16];
        for (int i = 0; i < 16; i++) fin >> c[i];
        color.push_back(c);

        if (fin.good() == false) break;
    }
    fin.close();

    cout << "poses: " << poses.size() << ", points: " << points.size() << endl;

    // read images
    vector<cv::Mat> images;
    boost::format fmt("../data/%d.png");
    for (int i = 0; i < 7; i++) {
        images.push_back(cv::imread((fmt % i).str(), 0));
    }

    // build optimization problem
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> DirectBlock;  // 求解的向量是6＊1的
    DirectBlock::LinearSolverType *linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
    DirectBlock *solver_ptr = new DirectBlock(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // TODO add vertices, edges into the graph optimizer
    // START YOUR CODE HERE
    // 添加路标结点
    for(int i = 0; i < points.size(); i++){                   // ***
        g2o::VertexSBAPointXYZ* pPoint = new g2o::VertexSBAPointXYZ();     // 注意VertexSBAPointXYZ是g2o自定义的结点类型
        pPoint->setId(i);
        pPoint->setEstimate(points[i]);
        pPoint->setMarginalized(true);     //由于路标点较相机位姿点更多，所以选择将路标点边缘化
        optimizer.addVertex(pPoint);
    }
    
    // 添加相机位姿结点
    for(int j = 0; j < poses.size(); j++){
        VertexSophus* pCamera = new VertexSophus();                                                       // ***
        pCamera->setId(j + points.size());        // 加上路标点的数量，使得相机位姿结点的Id与路标结点的Id相区别
        pCamera->setEstimate(poses[j]);
        optimizer.addVertex(pCamera);
    }
    
    //添加边(这里每个相机位姿结点和路标结点之间都有一条边存在)
    for(int j = 0; j < poses.size(); j++)
        for(int i = 0; i < points.size(); i++){
            EdgeDirectProjection* edge_PC = new EdgeDirectProjection(color[i], images[j]);
            edge_PC->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i)));              //为边添加对应的路标结点
            edge_PC->setVertex(1, dynamic_cast<VertexSophus*>(optimizer.vertex(j + points.size())));        //为边添加对应的相机位姿结点
            // 设置信息矩阵，由于我们的error为16维，所以信息矩阵的维度为16x16 !!!
            edge_PC->setInformation(Eigen::Matrix<double, 16, 16>::Identity());
            
            // 为每个边对应的误差项设置鲁棒核函数Huber
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            rk->setDelta(1.0);
            edge_PC->setRobustKernel(rk);
            optimizer.addEdge(edge_PC);
        }

    // END YOUR CODE HERE

    // perform optimization
    optimizer.initializeOptimization(0);
    optimizer.optimize(200);

    // TODO fetch data from the optimizer
    // START YOUR CODE HERE
    for(int j = 0; j < poses.size(); j++)
        for(int i = 0; i < points.size(); i++){
            Eigen::Vector3d pPoint_est = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i))->estimate();
            points[i] = pPoint_est;
            Sophus::SE3 pCamera_est = dynamic_cast<VertexSophus*>(optimizer.vertex(j + points.size()))->estimate();
            poses[j] = pCamera_est;
        }
    
    // END YOUR CODE HERE


    // plot the optimized points and poses
    Draw(poses, points);

    // delete color data
    for (auto &c: color) delete[] c;



    return 0;
}



void Draw(const VecSE3 &poses, const VecVec3d &points) {
    if (poses.empty() || points.empty()) {
        cerr << "parameter is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        for (auto &Tcw: poses) {
            glPushMatrix();
            Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
            glMultMatrixf((GLfloat *) m.data());
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glEnd();
            glPopMatrix();
        }

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); i++) {
            glColor3f(0.0, points[i][2]/4, 1.0-points[i][2]/4);
            glVertex3d(points[i][0], points[i][1], points[i][2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}


