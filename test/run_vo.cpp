//
// Created by dhp on 5/05/19.
//

#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 

#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include <myslam/orbextractor.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "usage: run_vo parameter_file" << endl;
        return 1;
    }
    myslam::Config::setParameterFile ( argv[1] );

    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );
    
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[2]), vstrImageLeft, vstrImageRight, vTimestamps);    
    const int nImages = vstrImageLeft.size();  
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);    
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   
    
    myslam::Camera::Ptr camera ( new myslam::Camera );

    cv::Mat imLeft, imRight;

    ofstream fout;
    fout.open("result.txt", ios_base::out);

    // viz模块显示运动
    
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    cv::Point3d cam_pos( 0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose( cam_pose );

    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    vis.showWidget( "World", world_coor );
    vis.showWidget( "Camera", camera_coor );

    //设置两帧VO的起始点与终点
    cv::Point3d point_begin(0.0, 0.0, 0.0);
    cv::Point3d point_end;

    //画图用
    int count = 0;


    cout<<"Vo start"<<endl;
    for(int ni=0; ni<nImages; ni++)
    {
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);


	if ( imLeft.data==nullptr || imRight.data==nullptr ) {
        cerr<<" image left or right is empty!"<<endl;
        break;
    }
    cout<<"image read.."<<endl;
	myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
    cout<<"frame create done"<<endl;

	pFrame->camera_ = camera;
	pFrame->image_left_ = imLeft;
	pFrame->image_right_= imRight;
	pFrame->time_stamp_ = vTimestamps[ni];
	boost::timer timer;
        cout<<"add frame ing"<<endl;

	vo->addFrame ( pFrame );

        cout<<"VO costs time: "<<timer.elapsed()<<endl;
	cout << "*******************************" << endl;
	if ( vo->state_ == myslam::VisualOdometry::LOST )
        break;
	SE3 Tcw = pFrame->T_c_w_.inverse();

        //按照KITTI的格式，将结果写入txt文件中
    fout<<Tcw.rotation_matrix()(0,0)<<" "
        <<Tcw.rotation_matrix()(0,1)<<" "
        <<Tcw.rotation_matrix()(0,2)<<" "
        <<Tcw.translation()(0,0)<<" "
        <<Tcw.rotation_matrix()(1,0)<<" "
        <<Tcw.rotation_matrix()(1,1)<<" "
        <<Tcw.rotation_matrix()(1,2)<<" "
        <<Tcw.translation()(1,0)<<" "
        <<Tcw.rotation_matrix()(2,0)<<" "
        <<Tcw.rotation_matrix()(2,1)<<" "
        <<Tcw.rotation_matrix()(2,2)<<" "
        <<Tcw.translation()(2,0)<<endl;

	//可视化
        cv::Affine3d M(
                cv::Affine3d::Mat3(
                        Tcw.rotation_matrix()(0,0), Tcw.rotation_matrix()(0,1), Tcw.rotation_matrix()(0,2),
                        Tcw.rotation_matrix()(1,0), Tcw.rotation_matrix()(1,1), Tcw.rotation_matrix()(1,2),
                        Tcw.rotation_matrix()(2,0), Tcw.rotation_matrix()(2,1), Tcw.rotation_matrix()(2,2)
                ),
                cv::Affine3d::Vec3(
                        Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0)
                )
        );

	
	vector<cv::viz::WLine> lines;
	//更新两帧VO的终点，起点是上次两帧VO的终点
	point_end = cv::Point3d(
		Tcw.translation()(0,0), 
		Tcw.translation()(1,0), 
		Tcw.translation()(2,0)
	    	);
	//画出两帧VO的终点
	cv::viz::WLine line(point_begin, point_end, cv::viz::Color::green());
	lines.push_back(line);
	
	//
	for(auto iter=lines.begin(); iter!=lines.end(); ++iter) {
	    string id = to_string(count++);
	    vis.showWidget(id, *iter);
	}
	
	//更新起始点
	point_begin = point_end;	
	
	cv::imshow("imageLeft", imLeft );
        cv::imshow("imageRight", imRight);
        cv::waitKey(1);
        vis.setWidgetPose( "Camera", M);
        vis.spinOnce(1, false);	


    }

    //保存轨迹
    vis.saveScreenshot("Trajectory.png");

    return 0;
}


void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);

        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}









