#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <execution>
#include <chrono>
#include "mutex"

using namespace std;
using namespace cv;

// this program shows how to use optical flow

string file_1 = "./1.png";  // first image
string file_2 = "./2.png";  // second image

// 互索器，防止多线程对一个容器进行操作，导致代码崩溃
std::mutex m1, m2;

// 通过宏定义来决定是否多线程
# define MT 0


// TODO implement this funciton
/**
 * single level optical flow
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

// Multi-threaded compute optical flow
void OpticalFlowSingleLevelMT(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

// TODO implement this funciton
/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

/**
 * get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return
 */
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);                //floor(x)返回的是小于或等于x的最大整数
    float yy = y - floor(y);
    return float(                                                      // 返回 双线性插值
            (1 - xx) * (1 - yy) * data[0] +  
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
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
    
    // images, note they are CV_8UC1, not CV_8UC3
    Mat img1 = imread(file_1, 0);     //读取灰度图像
    Mat img2 = imread(file_2, 0);

    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1, kp1);

    // now lets track these key points in the second image
    // first use single level LK in the validation picture
    vector<KeyPoint> kp2_single;
    vector<bool> success_single;
    OpticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single, true);      // false: forwards    true: inverse

    // then test multi-level LK
    vector<KeyPoint> kp2_multi;
    vector<bool> success_multi;
    // OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi, success_multi, true);
    evaluate_and_call([&]() { OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi, success_multi, true); },
                    "OpticalFlowMultiLevel", 1);

    // use opencv's flow for validation
    vector<Point2f> pt1, pt2;
    for (auto &kp: kp1) pt1.push_back(kp.pt);
    vector<uchar> status;
    vector<float> error;
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error, cv::Size(8, 8));

    // plot the differences of those functions
    Mat img2_single;
    cv::cvtColor(img2, img2_single, CV_GRAY2BGR);
    for (int i = 0; i < kp2_single.size(); i++) {
        if (success_single[i]) {
            cv::circle(img2_single, kp2_single[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 250, 0));
        }
    }

    
    Mat img2_multi;
    cv::cvtColor(img2, img2_multi, CV_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++) {
        if (success_multi[i]) {
            cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 250, 0));
        }
    }
    
    
    Mat img2_CV;
    cv::cvtColor(img2, img2_CV, CV_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }

    cv::imshow("tracked single level", img2_single);
    cv::imshow("tracked multi level", img2_multi);
    cv::imshow("tracked by opencv", img2_CV);
    cv::waitKey(0);

    return 0;
}



void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 10;
    bool have_initial = !kp2.empty();

    for (size_t i = 0; i < kp1.size(); i++) {
        auto kp = kp1[i];
        double dx = 0, dy = 0; // dx,dy need to be estimated
        if (have_initial) {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations
        for (int iter = 0; iter < iterations; iter++) {
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            cost = 0;

            if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                kp.pt.y + dy <= half_patch_size || kp.pt.y + dy >= img1.rows - half_patch_size) {   // go outside
                succ = false;
                break;
            }

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    // TODO START YOUR CODE HERE (~8 lines)
                    double error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) - GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy);
                    Eigen::Vector2d J;  // Jacobian
                    if (inverse == false) {
                        // Forward Jacobian
                        // 注意: x,y为相对角点的距离坐标，为取得像素在图像内的绝对坐标需要加上角点的绝对坐标(kp.pt.x, kp.pt.y)
                        // 注意: 雅可比前面的负号
                        J(0) = (GetPixelValue(img2, kp.pt.x + x + dx + 1, kp.pt.y + y + dy) - GetPixelValue(img2, kp.pt.x + x + dx - 1, kp.pt.y + y + dy)) / (-2);    
                        J(1) = (GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy + 1) - GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy - 1)) / (-2);    
                    } else {
                        // Inverse Jacobian
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                        // 用I1(x,y)处的梯度替换I2(x+dx,y+dy)处的梯度，使得计算得到的梯度总有意义
                        J(0) = (GetPixelValue(img1, kp.pt.x + x + 1, kp.pt.y + y) - GetPixelValue(img1, kp.pt.x + x - 1, kp.pt.y + y)) / (-2);     // u方向梯度
                        J(1) = (GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y + 1) - GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y - 1)) / (-2);     // v方向梯度
                    }
                    // compute H, b and set cost;
                    H += J * J.transpose();
                    b += -J * error;
                    cost += error * error / 2;
                    // TODO END YOUR CODE HERE
                }

            // compute update
            // TODO START YOUR CODE HERE (~1 lines)
            Eigen::Vector2d update;
            update = H.ldlt().solve(b);
            // TODO END YOUR CODE HERE

            if (isnan(update[0])) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }
            if (iter > 0 && cost > lastCost) {
                cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;
        }

        success.push_back(succ);

        // set kp2
        if (have_initial) {
            kp2[i].pt = kp.pt + Point2f(dx, dy);
        } else {
            KeyPoint tracked = kp;
            tracked.pt += cv::Point2f(dx, dy);
            kp2.push_back(tracked);
        }
    }
}



void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse) {
    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    typedef vector<KeyPoint> kpVecType;           // type of keypoint vector
    vector<Mat> pyr1, pyr2;                       // image pyramids
    vector<kpVecType> kp1_pyr_vect;               // kp1_pyr_vect用于从上到下存储所有金字塔内 imag1 的角点位置
    // 步骤1. 从上到下存储金字塔图像 和 金字塔内 imag1 的角点位置
    for (int i = 0; i < pyramids; i++) {         
        Mat img1_pyr, img2_pyr;                   // 储存各层金字塔对应尺寸的图像
        // 使用函数 cv::resize(Mat, result, cv::Size(width,height));构建不同尺寸的图像
        resize(img1, img1_pyr, Size(img1.size().width * scales[pyramids - 1 - i], img1.size().height * scales[pyramids - 1 - i]));
        resize(img2, img2_pyr, Size(img2.size().width * scales[pyramids - 1 - i], img2.size().height * scales[pyramids - 1 - i]));
        pyr1.push_back(img1_pyr);
        pyr2.push_back(img2_pyr);
        
        kpVecType kp1_pyr;                        // kp1_pyr存储imag1内的角点对应在单层金字塔上的位置       
        for(auto kp : kp1){                       // 注意: 这里对vector内成员的值进行操作，不会改变vector内的成员变量     
            kp.pt *= scales[pyramids - 1 - i];     
            kp1_pyr.push_back(kp);    
        }
        kp1_pyr_vect.push_back(kp1_pyr);
    }
    
    //步骤2. 通过 存储的金字塔图像 和 每一层金字塔内imag1的角点位置 计算每一层的imag2对应的角点位置  (coarse-to-fine LK tracking in pyramids)
    kpVecType kp2_pyr;                            // kp2_pyr存储估计的imag2对应的角点 对应在各层金字塔上的位置, 这个变量会在下面的for循环内不断迭代 
    for (int i = 0; i < pyramids; i++) {          // 注意: 对金字塔进行迭代计算的顺序为从上到下
        if(MT == 1){
            OpticalFlowSingleLevelMT(pyr1[i], pyr2[i], kp1_pyr_vect[i], kp2_pyr, success, inverse);  // 对每层金字塔使用多线程多层光流计算出粗糙解kp2_pyr
        }
        else if(MT == 0){
            OpticalFlowSingleLevel(pyr1[i], pyr2[i], kp1_pyr_vect[i], kp2_pyr, success, inverse);    // 对每层金字塔使用多线程单层光流计算出粗糙解kp2_pyr
        }
        else{}
        
        //OpticalFlowSingleLevelMT(pyr1[i], pyr2[i], kp1_pyr_vect[i], kp2_pyr, success, inverse);
        //将上一层求得的粗糙解 映射到下一层金字塔，作为下层金字塔迭代的初始值
        if(i < pyramids-1){
            for(auto &kp : kp2_pyr)               //注意: 这里必须对vector内成员的引用进行操作，否则无法改变vector内的成员变量
                kp.pt *= 2;
        }
    }
    // don't forget to set the results into kp2
    for (auto &kp: kp2_pyr)
        kp2.push_back(kp);
    
}




/*
void OpticalFlowSingleLevelMT(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 10;
    bool have_initial = !kp2.empty();

    for (size_t i = 0; i < kp1.size(); i++) {
        auto kp = kp1[i];
        double dx = 0, dy = 0; // dx,dy need to be estimated
        if (have_initial) {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations
        for (int iter = 0; iter < iterations; iter++) {
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            cost = 0;

            if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                kp.pt.y + dy <= half_patch_size || kp.pt.y + dy >= img1.rows - half_patch_size) {   // go outside
                succ = false;
                break;
            }

            // compute cost and jacobian
            vector<Eigen::Vector2i> index_xy;
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {
                    index_xy.push_back(Eigen::Vector2i(x, y));
                    
                }
                
            std::for_each(std::execution::par_unseq, index_xy.begin(), index_xy.end(), 
                  [&img1, &img2, &dx, &dy, &H, &b, &cost, &kp, &inverse](auto &index) {
                    int x = index(0);
                    int y = index(1);
                    // TODO START YOUR CODE HERE (~8 lines)
                    double error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) - GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy);
                    Eigen::Vector2d J;  // Jacobian
                    if (inverse == false) {
                        // Forward Jacobian
                        // 注意: x,y为相对角点的距离坐标，为取得像素在图像内的绝对坐标需要加上角点的绝对坐标(kp.pt.x, kp.pt.y)
                        // 注意: 雅可比前面的负号
                        J(0) = (GetPixelValue(img2, kp.pt.x + x + dx + 1, kp.pt.y + y + dy) - GetPixelValue(img2, kp.pt.x + x + dx - 1, kp.pt.y + y + dy)) / (-2);    
                        J(1) = (GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy + 1) - GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy - 1)) / (-2);    
                    } else {
                        // Inverse Jacobian
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                        // 用I1(x,y)处的梯度替换I2(x+dx,y+dy)处的梯度，使得计算得到的梯度总有意义
                        J(0) = (GetPixelValue(img1, kp.pt.x + x + 1, kp.pt.y + y) - GetPixelValue(img1, kp.pt.x + x - 1, kp.pt.y + y)) / (-2);     // u方向梯度
                        J(1) = (GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y + 1) - GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y - 1)) / (-2);     // v方向梯度
                    }
                    // compute H, b and set cost;
                    H += J * J.transpose();
                    b += -J * error;
                    cost += error * error / 2;
                    // TODO END YOUR CODE HERE     
                      
            });
            
            // compute update
            // TODO START YOUR CODE HERE (~1 lines)
            Eigen::Vector2d update;
            update = H.ldlt().solve(b);
            // TODO END YOUR CODE HERE

            if (isnan(update[0])) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }
            if (iter > 0 && cost > lastCost) {
                cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;
        }

        success.push_back(succ);

        // set kp2
        if (have_initial) {
            kp2[i].pt = kp.pt + Point2f(dx, dy);
        } else {
            KeyPoint tracked = kp;
            tracked.pt += cv::Point2f(dx, dy);
            kp2.push_back(tracked);
        }
    }
}

*/



/*
void OpticalFlowSingleLevelMT(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse) {

    // parameters
    int half_patch_size = 4;
    int iterations = 10;
    bool have_initial = !kp2.empty();
    
    std::for_each(std::execution::par_unseq, kp1.begin(), kp1.end(), 
                  [&half_patch_size, &iterations, &have_initial, &img1, &img2, &kp2, &success, &inverse](auto &kp) {
        double dx = 0, dy = 0; // dx,dy need to be estimated
        
        int i = ?;
        
        if (have_initial) {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations
        for (int iter = 0; iter < iterations; iter++) {
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            cost = 0;

            if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                kp.pt.y + dy <= half_patch_size || kp.pt.y + dy >= img1.rows - half_patch_size) {   // go outside
                succ = false;
                break;
            }

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    // TODO START YOUR CODE HERE (~8 lines)
                    double error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) - GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy);
                    Eigen::Vector2d J;  // Jacobian
                    if (inverse == false) {
                        // Forward Jacobian
                        // 注意: x,y为相对角点的距离坐标，为取得像素在图像内的绝对坐标需要加上角点的绝对坐标(kp.pt.x, kp.pt.y)
                        // 注意: 雅可比前面的负号
                        J(0) = (GetPixelValue(img2, kp.pt.x + x + dx + 1, kp.pt.y + y + dy) - GetPixelValue(img2, kp.pt.x + x + dx - 1, kp.pt.y + y + dy)) / (-2);    
                        J(1) = (GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy + 1) - GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy - 1)) / (-2);    
                    } else {
                        // Inverse Jacobian
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                        // 用I1(x,y)处的梯度替换I2(x+dx,y+dy)处的梯度，使得计算得到的梯度总有意义
                        J(0) = (GetPixelValue(img1, kp.pt.x + x + 1, kp.pt.y + y) - GetPixelValue(img1, kp.pt.x + x - 1, kp.pt.y + y)) / (-2);     // u方向梯度
                        J(1) = (GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y + 1) - GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y - 1)) / (-2);     // v方向梯度
                    }
                    // compute H, b and set cost;
                    H += J * J.transpose();
                    b += -J * error;
                    cost += error * error / 2;
                    // TODO END YOUR CODE HERE
                }

            // compute update
            // TODO START YOUR CODE HERE (~1 lines)
            Eigen::Vector2d update;
            update = H.ldlt().solve(b);
            // TODO END YOUR CODE HERE

            if (isnan(update[0])) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }
            if (iter > 0 && cost > lastCost) {
                cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;
        }

        std::lock_guard<std::mutex> guard(m1);    //代替m.lock(); m.unlock();某一个线程访问容器success的时候，就会上锁，其他线程无法访问该容器，该线程访问结束，会解锁，其他线程可以访问
        success.push_back(succ);
        
        
        // set kp2
        if (have_initial) {
            kp2[i].pt = kp.pt + Point2f(dx, dy);
        } else {
            KeyPoint tracked = kp;
            tracked.pt += cv::Point2f(dx, dy);
            std::lock_guard<std::mutex> guard(m2);
            kp2.push_back(tracked);
            
        }
    });
    
}

*/



