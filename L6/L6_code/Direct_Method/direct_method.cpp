#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <vector>
#include <string>
#include <boost/format.hpp>
#include <execution>
#include <chrono>
#include "mutex"

using namespace std;

// 互索器，防止多线程对一个容器进行操作，导致代码崩溃
std::mutex m1, m2, m3, m4, m5;

// 通过宏定义来决定是否多线程
# define MT 0

typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

// Camera intrinsics
// 内参
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// 基线
double baseline = 0.573;
// paths
string left_file = "./left.png";
string disparity_file = "./disparity.png";
boost::format fmt_others("./%06d.png");    // other files

// useful typedefs
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// TODO implement this function
/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
);

// TODO implement this function
/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
);

void DirectPoseEstimationSingleLayerMT(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
);

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

inline int findPosVector(VecVector2d myVector , Eigen::Vector2d value)
{
    VecVector2d::iterator iter=std::find(myVector.begin(),myVector.end(),value);//返回的是一个迭代器指针
    if(iter == myVector.end())
    {
        return -1;
    } else{
        return std::distance(myVector.begin(),iter);
    }
}


// 打印函数执行时间
template <typename FuncT>
void evaluate_and_call(FuncT func, const std::string &func_name = "",
                       int times = 10) {
  double total_time = 0;
  for (int i = 0; i < times; ++i) {
    auto t1 = std::chrono::steady_clock::now();
    func();
    auto t2 = std::chrono::steady_clock::now();
    total_time +=
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count() *
        1000;
  }

  std::cout << "方法 " << func_name
            << " 平均调用时间/次数: " << total_time / times << "/" << times
            << " 毫秒." << std::endl;
}


int main(int argc, char **argv) {

    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng;
    int nPoints = 1000;
    int boarder = 40;
    VecVector2d pixels_ref;
    vector<double> depth_ref;

    // generate pixels in ref and load depth data
    for (int i = 0; i < nPoints; i++) {
        int x = rng.uniform(boarder, left_img.cols - boarder);  // don't pick pixels close to boarder
        int y = rng.uniform(boarder, left_img.rows - boarder);  // don't pick pixels close to boarder
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity; // you know this is disparity to depth
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }

    // estimates 01~05.png's pose using this information
    Sophus::SE3 T_cur_ref;
    T_cur_ref = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());       // 初始化估计的位姿,令R = I， t = 0;

    for (int i = 1; i < 6; i++) {  // 1~10
        cv::Mat img = cv::imread((fmt_others % i).str(), 0);
        
        /*
        if(MT == 1)
            // 多线程
            evaluate_and_call([&]() { DirectPoseEstimationSingleLayerMT(left_img, img, pixels_ref, depth_ref, T_cur_ref); },  
                    "DirectPoseEstimationSingleLayerMT", 1);
        else
            //单线程
            evaluate_and_call([&]() { DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref); },
                    "DirectPoseEstimationSingleLayer", 1);
        */        
        
        DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
        
        
    }
}


void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 100;

    double cost = 0, lastCost = 0;
    int nGood = 0;  // good projections
    VecVector2d goodProjection;
    
    for (int iter = 0; iter < iterations; iter++) {
        nGood = 0;
        goodProjection.clear();

        // Define Hessian and bias
        Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
        Vector6d b = Vector6d::Zero();  // 6x1 bias
        
        for (size_t i = 0; i < px_ref.size(); i++) {

            // compute the projection in the second image
            // TODO START YOUR CODE HERE
            nGood++;                                               // 存入至少一个投影点，防止 nGood = 0
            goodProjection.push_back(Eigen::Vector2d(0, 0));      
            
            Eigen::Vector2d px = px_ref[i];
            double X = (px(0) - cx) * depth_ref[i] / fx;            
            double Y = (px(1) - cy) * depth_ref[i] / fy;
            double Z = depth_ref[i];
            Eigen::Vector4d P_homog = Eigen::Vector4d(X, Y, Z, 1); // 将px_ref[i]转化为3D齐次座标
            Eigen::Vector4d P_rot = T21.matrix() * P_homog;        // P_rot为经估计的变换矩阵变换后得到的3D坐标
            double X_rot = P_rot(0);
            double Y_rot = P_rot(1);
            double Z_rot = P_rot(2);
            Eigen::Matrix3d K;
            K << fx, 0, cx, 0, fy, cy, 0, 0, 1;                    // 内参矩阵
            // px_rot 为选取的像素点经变换矩阵变换后进行投影的像素坐标
            Eigen::Vector3d px_rot = K * Eigen::Vector3d(X_rot, Y_rot, Z_rot);
            // 对 px_rot 齐次化
            px_rot = Eigen::Vector3d(px_rot(0)/px_rot(2), px_rot(1)/px_rot(2), 1);  
                    
            // 判断 px_rot的坐标值是否跑出图像外
            if(px_rot(0) < half_patch_size || px_rot(0) > img2.cols - half_patch_size 
                || px_rot(1) < half_patch_size || px_rot(1) > img2.rows - half_patch_size){  
                continue;
            }
            else{
                nGood++;
                goodProjection.push_back(Eigen::Vector2d(px_rot(0), px_rot(1))); 
            }
    
            // and compute error and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {
                    // 计算数据点的光度误差
                    double error = GetPixelValue(img1, px(0) + x, px(1) + y) - GetPixelValue(img2, px_rot(0) + x, px_rot(1) + y);        
                    
                    Matrix26d J_pixel_xi;   // pixel to \xi in Lie algebra
                    J_pixel_xi(0,0) = fx / Z_rot;
                    J_pixel_xi(0,1) = 0;
                    J_pixel_xi(0,2) = -fx * X_rot / Z_rot / Z_rot;
                    J_pixel_xi(0,3) = -fx * X_rot * Y_rot / Z_rot / Z_rot;
                    J_pixel_xi(0,4) = fx + fx * X_rot * X_rot / Z_rot / Z_rot;
                    J_pixel_xi(0,5) = -fy * Y_rot / Z_rot;
                    J_pixel_xi(1,0) = 0;
                    J_pixel_xi(1,1) = fy / Z_rot;
                    J_pixel_xi(1,2) = -fy * Y_rot / Z_rot / Z_rot;
                    J_pixel_xi(1,3) = -fy - fy * Y_rot * Y_rot / Z_rot / Z_rot;
                    J_pixel_xi(1,4) = fy * X_rot * Y_rot / Z_rot / Z_rot; 
                    J_pixel_xi(1,5) = fy * X_rot / Z_rot;
                    
                    Eigen::Vector2d J_img_pixel;    // image gradients
                    J_img_pixel(0) = (GetPixelValue(img2, px_rot(0) + x + 1, px_rot(1) + y) - GetPixelValue(img2, px_rot(0) + x - 1, px_rot(1) + y)) / 2;
                    J_img_pixel(1) = (GetPixelValue(img2, px_rot(0) + x, px_rot(1) + y + 1) - GetPixelValue(img2, px_rot(0) + x, px_rot(1) + y - 1)) / 2;
                    
                    // total jacobian
                    Vector6d J = - J_pixel_xi.transpose() * J_img_pixel;

                    H += J * J.transpose();
                    b += -error * J;
                    cost += error * error;
                }
            // END YOUR CODE HERE
        }

        // solve update and put it into estimation
        // TODO START YOUR CODE HERE
        Vector6d update;
        update = H.ldlt().solve(b);
        T21 = Sophus::SE3::exp(update) * T21;
        // END YOUR CODE HERE

        cost /= nGood;

        if (isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            // cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost) {
            // cout << "cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        lastCost = cost;
        // cout << "cost = " << cost << ", good = " << nGood << endl;
    }
    // cout << "good projection: " << nGood << endl;
    cout << "T21 = \n" << T21.matrix() << endl;

    // in order to help you debug, we plot the projected pixels here
    cv::Mat img1_show, img2_show;
    cv::cvtColor(img1, img1_show, CV_GRAY2BGR);
    cv::cvtColor(img2, img2_show, CV_GRAY2BGR);

    for (auto &px: px_ref) {
        cv::rectangle(img1_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, 250, 0));
    }
    
    for (auto &px: goodProjection) {
        cv::rectangle(img2_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, 250, 0));
    }

    
    cv::imshow("reference", img1_show);
    cv::imshow("current", img2_show);
    cv::waitKey();
}




void DirectPoseEstimationSingleLayerMT(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 100;

    double cost = 0, lastCost = 0;
    int nGood = 0;  // good projections
    VecVector2d goodProjection;
    
    for (int iter = 0; iter < iterations; iter++) {
        nGood = 0;
        goodProjection.clear();

        // Define Hessian and bias
        Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
        Vector6d b = Vector6d::Zero();  // 6x1 bias
        int i = 0;
        
        std::for_each(std::execution::par_unseq, px_ref.begin(), px_ref.end(), 
                  [&half_patch_size, &img1, &img2, &depth_ref, &T21, &i, 
                     &H, &b, &cost, &nGood, &goodProjection](auto &px) {

            // compute the projection in the second image
            // TODO START YOUR CODE HERE
            nGood++;                                               // 存入至少一个投影点，防止 nGood = 0
            goodProjection.push_back(Eigen::Vector2d(0, 0));      
            //int i = findPosVector(px_ref , px);                    // 如何获取元素 px 在容器 px_ref 中的位置?
            
            
            double X = (px(0) - cx) * depth_ref[i] / fx;            
            double Y = (px(1) - cy) * depth_ref[i] / fy;
            double Z = depth_ref[i];
            Eigen::Vector4d P_homog = Eigen::Vector4d(X, Y, Z, 1); // 将px_ref[i]转化为3D齐次座标
            Eigen::Vector4d P_rot = T21.matrix() * P_homog;        // P_rot为经估计的变换矩阵变换后得到的3D坐标
            double X_rot = P_rot(0);
            double Y_rot = P_rot(1);
            double Z_rot = P_rot(2);
            Eigen::Matrix3d K;
            K << fx, 0, cx, 0, fy, cy, 0, 0, 1;                    // 内参矩阵
            // px_rot 为选取的像素点经变换矩阵变换后进行投影的像素坐标
            Eigen::Vector3d px_rot = K * Eigen::Vector3d(X_rot, Y_rot, Z_rot);
            // 对 px_rot 齐次化
            px_rot = Eigen::Vector3d(px_rot(0)/px_rot(2), px_rot(1)/px_rot(2), 1);  
            
            /*
            // 判断 px_rot的坐标值是否跑出图像外
            if(px_rot(0) < half_patch_size || px_rot(0) > img2.cols - half_patch_size 
                || px_rot(1) < half_patch_size || px_rot(1) > img2.rows - half_patch_size){  
                continue;
            }
            else{
                nGood++;
                std::lock_guard<std::mutex> guard(m1);    
                //代替m.lock(); m.unlock();某一个线程访问容器success的时候，就会上锁，其他线程无法访问该容器，该线程访问结束，会解锁，其他线程可以访问
                goodProjection.push_back(Eigen::Vector2d(px_rot(0), px_rot(1))); 
            }
            */
           
            if(px_rot(0) >= half_patch_size && px_rot(0) <= img2.cols - half_patch_size 
                && px_rot(1) >= half_patch_size && px_rot(1) <= img2.rows - half_patch_size){
                // and compute error and jacobian
                for (int x = -half_patch_size; x < half_patch_size; x++)
                    for (int y = -half_patch_size; y < half_patch_size; y++) {
                        // 计算数据点的光度误差
                        double error = GetPixelValue(img1, px(0) + x, px(1) + y) - GetPixelValue(img2, px_rot(0) + x, px_rot(1) + y);        
                        
                        Matrix26d J_pixel_xi;   // pixel to \xi in Lie algebra
                        J_pixel_xi(0,0) = fx / Z_rot;
                        J_pixel_xi(0,1) = 0;
                        J_pixel_xi(0,2) = -fx * X_rot / Z_rot / Z_rot;
                        J_pixel_xi(0,3) = -fx * X_rot * Y_rot / Z_rot / Z_rot;
                        J_pixel_xi(0,4) = fx + fx * X_rot * X_rot / Z_rot / Z_rot;
                        J_pixel_xi(0,5) = -fy * Y_rot / Z_rot;
                        J_pixel_xi(1,0) = 0;
                        J_pixel_xi(1,1) = fy / Z_rot;
                        J_pixel_xi(1,2) = -fy * Y_rot / Z_rot / Z_rot;
                        J_pixel_xi(1,3) = -fy - fy * Y_rot * Y_rot / Z_rot / Z_rot;
                        J_pixel_xi(1,4) = fy * X_rot * Y_rot / Z_rot / Z_rot; 
                        J_pixel_xi(1,5) = fy * X_rot / Z_rot;
                    
                        Eigen::Vector2d J_img_pixel;    // image gradients
                        J_img_pixel(0) = (GetPixelValue(img2, px_rot(0) + x + 1, px_rot(1) + y) - GetPixelValue(img2, px_rot(0) + x - 1, px_rot(1) + y)) / 2;
                        J_img_pixel(1) = (GetPixelValue(img2, px_rot(0) + x, px_rot(1) + y + 1) - GetPixelValue(img2, px_rot(0) + x, px_rot(1) + y - 1)) / 2;
                    
                        // total jacobian
                        Vector6d J = - J_pixel_xi.transpose() * J_img_pixel;

                        
                        std::lock_guard<std::mutex> lock{m3};
                        H += J * J.transpose();
                        b += -error * J;
                        cost += error * error;
                    }
                nGood++;
                std::lock_guard<std::mutex> guard(m1);    
                //代替m.lock(); m.unlock();某一个线程访问容器success的时候，就会上锁，其他线程无法访问该容器，该线程访问结束，会解锁，其他线程可以访问
                goodProjection.push_back(Eigen::Vector2d(px_rot(0), px_rot(1)));
            }
            std::lock_guard<std::mutex> lock{m2};
            i++;
            
            // END YOUR CODE HERE
        });

        // solve update and put it into estimation
        // TODO START YOUR CODE HERE
        Vector6d update;
        update = H.ldlt().solve(b);
        T21 = Sophus::SE3::exp(update) * T21;
        // END YOUR CODE HERE

        cost /= nGood;

        if (isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            // cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost) {
            // cout << "cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        lastCost = cost;
        // cout << "cost = " << cost << ", good = " << nGood << endl;
    }
    // cout << "good projection: " << nGood << endl;
    cout << "T21 = \n" << T21.matrix() << endl;

    // in order to help you debug, we plot the projected pixels here
    cv::Mat img1_show, img2_show;
    cv::cvtColor(img1, img1_show, CV_GRAY2BGR);
    cv::cvtColor(img2, img2_show, CV_GRAY2BGR);

    for (auto &px: px_ref) {
        cv::rectangle(img1_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, 250, 0));
    }
    
    for (auto &px: goodProjection) {
        cv::rectangle(img2_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, 250, 0));
    }

    
    cv::imshow("reference", img1_show);
    cv::imshow("current", img2_show);
    cv::waitKey();
}





void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<cv::Mat> pyr1, pyr2; // image pyramids
    vector<VecVector2d> px_ref_pyr_vect;               // px_ref_pyr_vect用于从上到下存储所有金字塔内 imag1 的角点位置
    // TODO START YOUR CODE HERE

    // END YOUR CODE HERE
    // 步骤1. 从上到下存储金字塔图像 和 金字塔内 img1 的角点位置
     double fxG = fx, fyG = fy, cxG = cx, cyG = cy;  // backup the old values
     
     for (int level = pyramids - 1; level >= 0; level--) {
        cv::Mat img1_pyr, img2_pyr;                   // 储存各层金字塔对应尺寸的图像
        // 使用函数 cv::resize(Mat, result, cv::Size(width,height));构建不同尺寸的图像
        resize(img1, img1_pyr, cv::Size(img1.size().width * scales[level], img1.size().height * scales[level]));
        resize(img2, img2_pyr, cv::Size(img2.size().width * scales[level], img2.size().height * scales[level]));
        pyr1.push_back(img1_pyr);
        pyr2.push_back(img2_pyr); 
        VecVector2d px_ref_pyr;                      // set the keypoints in this pyramid level
        for (auto px: px_ref) {
            px_ref_pyr.push_back(scales[level] * px);
        }
        px_ref_pyr_vect.push_back(px_ref_pyr);
     }
    
    //步骤2. 通过 存储的金字塔图像 和 每一层金字塔内img1的角点位置 计算每一层的img2对应的角点位置  (coarse-to-fine LK tracking in pyramids)
    T21 = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());       // 初始化估计的位姿,令R = I， t = 0;
    for (int level = pyramids - 1; level >= 0; level--) {          // 注意: 对金字塔进行迭代计算的顺序为从上到下
            // scale fx, fy, cx, cy in different pyramid levels
            fx = fxG * scales[level];
            fy = fyG * scales[level];
            cx = cxG * scales[level];
            cy = cyG * scales[level];
            // 对每层金字塔使用单层直接法计算出粗糙解T21
            
            // 调用单层直接法
            DirectPoseEstimationSingleLayer(pyr1[pyramids - 1 - level], pyr2[pyramids - 1 - level], 
                                                px_ref_pyr_vect[pyramids - 1 - level], depth_ref, T21);
            //将上一层求得的粗糙解作为下层金字塔迭代的初始值
    }
    // 最后恢复fx, fy, cx, cy
    fx = fxG;
    fy = fyG;
    cx = cxG;
    cy = cyG;
    
}


