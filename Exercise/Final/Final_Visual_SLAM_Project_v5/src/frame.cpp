/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "myslam/frame.h"

namespace myslam
{
Frame::Frame()
: id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
{

}

Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat gray_left, Mat gray_right )
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), gray_left_(gray_left), gray_right_(gray_right), is_key_frame_(false)
{

}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    return Frame::Ptr( new Frame(factory_id++) );
}

/*
double Frame::findDepth ( const cv::KeyPoint& kp )
{
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x];
    if ( d!=0 )
    {
        return double(d)/camera_->depth_scale_;
    }
    else 
    {
        // check the nearby points 
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/camera_->depth_scale_;
            }
        }
    }
    return -1.0;
}
*/

double Frame::findDepth ( const cv::KeyPoint keypoint )
{
    vector<cv::Point2f> pt_left, pt_right;
    pt_left.push_back(keypoint.pt);
    vector<uchar> status;
    vector<float> error;
    cv::calcOpticalFlowPyrLK(gray_left_, gray_right_, pt_left, pt_right,
                             status, error, cv::Size(8, 8));


    /*
    Mat img2_CV;
    cv::cvtColor(gray_right_, img2_CV, CV_GRAY2BGR);
    for (int i = 0; i < pt_right.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pt_right[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt_left[i], pt_right[i], cv::Scalar(0, 250, 0));
        }
    }
    cv::imshow("tracked by opencv", img2_CV);
    cv::waitKey(0);
    */

    // erase the keypoints, which are lost in optical flow
    // under a good stereo camera alignment, the matched keypoints in the left and right image have very similar y-coordinate
    if ( status[0] == 0 || abs(pt_right[0].y - pt_left[0].y) > 5 ||  (pt_left[0].x <= pt_right[0].x)
                 || pt_right[0].y < 0 || pt_right[0].y > gray_right_.rows-1
                 || pt_right[0].x < 0 || pt_right[0].x > gray_right_.cols-1 )
    {
        return -1.0;     // return a negative depth
    }
    else
    {
        return camera_->fx_ * camera_->b_ / ( pt_left[0].x - pt_right[0].x );
    }
}

void Frame::setPose ( const SE3& T_c_w )
{
    T_c_w_ = T_c_w;
}


Vector3d Frame::getCamCenter() const
{
    return T_c_w_.inverse().translation();
}

bool Frame::isInFrame ( const Vector3d& pt_world )
{
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    // cout<<"P_cam = "<<p_cam.transpose()<<endl;
    if ( p_cam(2,0)<0 ) return false;
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
    return pixel(0,0)>0 && pixel(1,0)>0 
        && pixel(0,0)<gray_left_.cols
        && pixel(1,0)<gray_left_.rows;
}

}
