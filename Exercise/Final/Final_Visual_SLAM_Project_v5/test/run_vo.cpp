// -------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <boost/format.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pangolin/pangolin.h>
#include "unistd.h"

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    myslam::Config::setParameterFile ( argv[1] );
    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );

    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;

    // read frame time stamps
    vector<double> frame_times;
    ifstream fin_times ( dataset_dir+"/times.txt");
    // ifstream fin_times ( "/home/jindong/SLAM/04/times.txt");
    while ( !fin_times.eof() )
    {
        string times;
        fin_times >> times;
        frame_times.push_back( atof( times.c_str() ) );


        if( fin_times.good() == false )
            break;

    }
    cout << "The number of frames: " << frame_times.size() << endl;


    // read images
    // vector<cv::Mat> gray_left_images, gray_right_images;

    // boost::format fmt_left("/home/jindong/SLAM/00/image_0/%06d.png");
    // boost::format fmt_left("dataset_dir/image_0/%d.png");
    // string left_file = "/home/jindong/SLAM/test_00/image_0/1.png";
    /*
    for ( int i=0; i<frame_times.size(); i++ )
    {
        gray_left_images.push_back(cv::imread((fmt_left % i).str(), 0));
        // gray_left_images.push_back(cv::imread(left_file, 0));

    }

    boost::format fmt_right("/home/jindong/SLAM/00/image_1/%06d.png");
    // boost::format fmt_right("dataset_dir/image_1/%d.png");
    for ( int i=0; i<frame_times.size(); i++ )
        gray_right_images.push_back( cv::imread((fmt_right % i).str(), 0) );

    */


    // ***********************************************************************
    myslam::Camera::Ptr camera ( new myslam::Camera );

    // visualization
    cv::viz::Viz3d vis ( "Visual Odometry" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose ( cam_pose );

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );



    cout<<"read total "<<frame_times.size() <<" entries"<<endl;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_trajectory;
    boost::format fmt_left(dataset_dir+"/image_0/%06d.png");
    boost::format fmt_right(dataset_dir+"/image_1/%06d.png");
    // boost::format fmt_left("/home/jindong/SLAM/04/image_0/%06d.png");
    // boost::format fmt_right("/home/jindong/SLAM/04/image_1/%06d.png");

    // create a loop to load the frames into visual odometry
    for ( int i=0; i<frame_times.size(); i++ )
    {
        cout<<"****** loop "<<i<<" ******"<<endl;
        Mat gray_left_image = cv::imread((fmt_left % i).str(), 0);
        Mat gray_right_image = cv::imread((fmt_right % i).str(), 0);

	    if( gray_left_image.data == nullptr || gray_right_image.data == nullptr )
            break;
	    //***********************
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;

	    pFrame->gray_left_ = gray_left_image;
        pFrame->gray_right_ = gray_right_image;
        pFrame->time_stamp_ = frame_times[i];

	    // ****************************
        boost::timer timer;
        vo->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        if ( vo->state_ == myslam::VisualOdometry::LOST )
            break;
        SE3 Twc = pFrame->T_c_w_.inverse();
        poses_trajectory.push_back(Twc);

        // show the map and the camera pose
        cv::Affine3d M (
            cv::Affine3d::Mat3 (
                Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
                Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
                Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
            ),
            cv::Affine3d::Vec3 (
                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
            )
        );

        // Mat img_show = color.clone();
	    Mat img_show = gray_left_image.clone();
        // ***********************************************
        for ( auto& pt:vo->map_->map_points_ )
        {
            myslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
        }

        cv::imshow ( "image", img_show );
        cv::waitKey ( 1 );
        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1, false );
        cout<<endl;
    }

    // draw trajectory in pangolin
    DrawTrajectory(poses_trajectory);
    return 0;
}




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


