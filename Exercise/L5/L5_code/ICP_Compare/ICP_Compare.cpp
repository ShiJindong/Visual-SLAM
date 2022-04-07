#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include "unistd.h"

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
// Eigen 部分
#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>

// 第5次作业第5题: 用ICP实现轨迹对齐

using namespace std;

// path to trajectory file
string compare_file = "compare.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
void DrawDoubleTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &, 
                          vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &);
void transformation_estimation(const vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &,
                               const vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &,
                               Sophus::SE3 &);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_estimated;          //存储估计位姿
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_goundtruth;         //存储真实位姿
    
    /// implement pose reading code
   
    double sampletime,q_x,q_y,q_z,q_w,t_x,t_y,t_z;      
    //读取compare.txt内的估计轨迹数据和真实轨迹数据
    ifstream fin_trajectory(compare_file);
    
	while(!fin_trajectory.eof()){
        //注意: 给定的数据排列为: [sampletime, t_x, t_y, t_z, q_x, q_y, q_z, q_w], 但是Eigen的四元数虚部在前，实部在后
        fin_trajectory>>sampletime >> t_x >> t_y >> t_z >> q_x >> q_y >> q_z >> q_w;         
        poses_estimated.push_back(Sophus::SE3(Eigen::Quaterniond(q_w, q_x,q_y,q_z),Eigen::Vector3d(t_x,t_y,t_z)));     // 注意: 四元数赋值时实部在前，虚部在后
        fin_trajectory>>sampletime >> t_x >> t_y >> t_z >> q_x >> q_y >> q_z >> q_w;  
        poses_goundtruth.push_back(Sophus::SE3(Eigen::Quaterniond(q_w, q_x,q_y,q_z),Eigen::Vector3d(t_x,t_y,t_z))); 
	}
    
    Sophus::SE3 Tge;
    transformation_estimation(poses_goundtruth, poses_estimated, Tge);
    
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_transformed;          //变换估计位姿，使其与真实对齐
    for(auto& pose : poses_estimated){
       poses_transformed.push_back(Tge * pose); 
    }
    
    // draw trajectory in pangolin
    // DrawDoubleTrajectory(poses_goundtruth, poses_estimated);
    DrawDoubleTrajectory(poses_goundtruth, poses_transformed);
    // end your code here
    
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
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
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}


/****************************************************************************************************************/
void DrawDoubleTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> & poses_goundtruth, 
                          vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> & poses_estimated) {
    if (poses_goundtruth.empty() || poses_estimated.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Comparision", 1024, 768);
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
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        
        for (size_t i = 0; i < poses_goundtruth.size() - 1; i++) {
            glColor3f(1 - (float) i / poses_goundtruth.size(), 0.0f, (float) i / poses_goundtruth.size());
            glBegin(GL_LINES);
            auto p1 = poses_goundtruth[i], p2 = poses_goundtruth[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        
         for (size_t i = 0; i < poses_estimated.size() - 1; i++) {
            glColor3f(0.0f, 1.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = poses_estimated[i], p2 = poses_estimated[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}


/****************************************************************************************************************/
void transformation_estimation(const vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> & poses_goundtruth,
                               const vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> & poses_estimated,
                               Sophus::SE3 & Tge){
    if (poses_goundtruth.empty() || poses_estimated.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }
    
    Eigen::Vector3d p_goundtruth, p_estimated;        // center of mass
    int N = poses_goundtruth.size();
    
    for(int i = 0; i < N; i++){
        p_goundtruth += poses_goundtruth[i].translation();
        p_estimated += poses_estimated[i].translation();
    }
    p_goundtruth /= N; p_estimated /= N;
    
    vector<Eigen::Vector3d> q_goundtruth(N), q_estimated(N);     // remove the center
    for(int i = 0; i < N; i++){
        q_goundtruth[i] = poses_goundtruth[i].translation() - p_goundtruth;
        q_estimated[i] = poses_estimated[i].translation() - p_estimated;
    }
    
    // compute q_goundtruth * q_estimated
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(int i = 0; i < N; i++){
        W += q_goundtruth[i] * q_estimated[i].transpose();
    }
    // cout << "W = " << endl << W << endl;
    
    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    // cout << "U = " << endl << U << endl;
    // cout << "V = " << endl << V << endl;
    
    Eigen::Matrix3d Rge = U * (V.transpose());
    Eigen::Vector3d tge = p_goundtruth - Rge * p_estimated;
    cout << "Rge = " << endl << Rge << endl
         << "tge = " << endl << tge << endl;
    
    // convert to SE3 
    Tge = Sophus::SE3(Rge, tge);

}
