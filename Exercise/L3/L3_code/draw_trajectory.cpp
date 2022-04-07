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


using namespace std;

// path to trajectory file
string trajectory_file = "trajectory.txt";
string groundtruth_file = "groundtruth.txt";
string estimated_file = "estimated.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
void DrawDoubleTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &, 
                          vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_trajectory;
    
    /// implement pose reading code
    // start your code here (5~10 lines)
    // 第3次作业第7题: 轨迹的描绘.
    double sampletime,q_x,q_y,q_z,q_w,t_x,t_y,t_z;      
    //读取数据
    ifstream fin_trajectory(trajectory_file);
    
	while(!fin_trajectory.eof()){
        //注意: 给定的数据排列为: [sampletime, t_x, t_y, t_z, q_x, q_y, q_z, q_w], 四元数虚部在前，实部在后
        fin_trajectory>>sampletime >> t_x >> t_y >> t_z >> q_x >> q_y >> q_z >> q_w;         
        poses_trajectory.push_back(Sophus::SE3(Eigen::Quaterniond(q_w, q_x,q_y,q_z),Eigen::Vector3d(t_x,t_y,t_z)));     // 注意: 四元数赋值时实部在前，虚部在后
	}
    
    
    
    // 第3次作业第8题: 轨迹的误差
    
    // 读取groundtruth.txt内的真实轨迹数据
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_goundtruth;
    ifstream fin_groundtruth(groundtruth_file);
    while(!fin_groundtruth.eof()){
        fin_groundtruth>>sampletime >> t_x >> t_y >> t_z >> q_x >> q_y >> q_z >> q_w;
        poses_goundtruth.push_back(Sophus::SE3(Eigen::Quaterniond(q_w,q_x,q_y,q_z),Eigen::Vector3d(t_x,t_y,t_z)));
    }
    
    // 读取estimated.txt内的估计轨迹数据
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_estimated;
    ifstream fin_estimated(estimated_file);
    while(!fin_estimated.eof()){
        fin_estimated >> sampletime >> t_x >> t_y >> t_z >> q_x >> q_y >> q_z >> q_w;
        poses_estimated.push_back(Sophus::SE3(Eigen::Quaterniond(q_w,q_x,q_y,q_z),Eigen::Vector3d(t_x,t_y,t_z)));
    }
    
    //定义轨迹误差的李代数
    Eigen::Matrix<double,6,1> se3_Terror;  
    //定义轨迹误差的均方根
    double RMSE = 0.0;
    
    for(unsigned int i=0; i<poses_goundtruth.size(); i++){
        se3_Terror = (poses_goundtruth[i].inverse()*poses_estimated[i]).log();
        //函数squaredNorm()可以用来求取一个向量的二范数(Euclidean,欧氏距离),数学上为向量自己对自己的点积， 即向量各个元素的平方和
        RMSE += se3_Terror.squaredNorm();
    }
    // 求取最终两条轨迹误差的均方根
    RMSE = sqrt(RMSE/(double)poses_goundtruth.size());
    cout << "RMSE between groundtruth and estimated trajectory: " << RMSE << endl;         
    
    
    // draw trajectory in pangolin
    DrawTrajectory(poses_trajectory);
    DrawDoubleTrajectory(poses_goundtruth, poses_estimated);
    
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
