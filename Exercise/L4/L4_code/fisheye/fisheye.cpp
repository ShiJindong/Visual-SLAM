//
// Created by xiang on 2021/9/9.
//

#include <opencv2/opencv.hpp>
#include <cmath>

// 文件路径，如果不对，请调整
std::string input_file = "./fisheye.jpg";

int main(int argc, char **argv) {
  // 本程序实现鱼眼的等距投影去畸变模型
  // 畸变参数（本例设为零）
  double k1 = 0, k2 = 0, k3 = 0, k4 = 0;

  // 内参
  double fx = 689.21, fy = 690.48, cx = 1295.56, cy = 942.17;

  cv::Mat image = cv::imread(input_file);
  int rows = image.rows, cols = image.cols;
  cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC3); // 去畸变以后的图

  // 计算去畸变后图像的内容
  for (int v = 0; v < rows; v++)
    for (int u = 0; u < cols; u++) {

      double u_distorted = 0, v_distorted = 0;
      // TODO 按照公式，计算点(u,v)对应到畸变图像中的坐标(u_distorted,
      // v_distorted) (~6 lines)
      // start your code here
      double x = (u - cx)/fx;                          // x,y表示去畸变图像(u,v)点对应的相机归一化三维成像平面上的坐标(深度Z=1)
      double y = (v - cy)/fy;
      double r = sqrt(x * x + y * y);                  // r为鱼眼半球体截面半径
      double theta = atan(r);                          // theta为鱼眼半球体上一点与光心的连线和光轴的夹角
      // 利用畸变模型求出发生畸变后theta对应的夹角theta_distorted
      double theta_distorted = theta * (1 + k1 * theta * theta + k2 * pow(theta,4) + k3 * pow(theta,6) + k4 * pow(theta,8));
      // 利用带畸变的夹角theta_distorted求出点(x, y, 1)对应的带畸变的相机归一化三维成像平面上的坐标(x_distorted, y_distorted, 1)
      double x_distorted = (theta_distorted/r) * x;
      double y_distorted = (theta_distorted/r) * y;
      // 利用带畸变的夹角theta_distorted，将带畸变的相机归一化三维成像平面上的坐标(x_distorted, y_distorted, 1)重新投影到二维图像平面上得到(u_distorted, v_distorted)
      // u_distorted = fx * (x_distorted + alpha * y_distorted) + cx; 
      u_distorted = fx * x_distorted + cx;       // 令输入参数alpha = 0
      v_distorted = fy * y_distorted + cy;
      // end your code here

      // 赋值 (最近邻插值)
      if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols &&
          v_distorted < rows) {
        image_undistort.at<cv::Vec3b>(v, u) =
            image.at<cv::Vec3b>((int)v_distorted, (int)u_distorted);
      } else {
        image_undistort.at<cv::Vec3b>(v, u) = 0;
      }
    }

  // 画图去畸变后图像
  cv::imshow("image undistorted", image_undistort);
  cv::imwrite("fisheye_undist.jpg", image_undistort);
  cv::waitKey();

  return 0;
}
