//
// Created by dhp on 5/05/19.
//


#ifndef CAMERA_H
#define CAMERA_H

#include <myslam/common_include.h>
namespace myslam
{
//针孔相机模型
class Camera
{
public:
  //表示相机的智能指针
  typedef std::shared_ptr<Camera> Ptr;
  //表示相机的内部参数
  float fx_, fy_, cx_, cy_;
  //构造函数，通过该函数可以直接声明空类型Camera cam;
  Camera();
  //构造函数，将形参的值赋给相机的内部参数
  Camera(float fx, float fy, float cx, float cy):fx_(fx),fy_(fy),cx_(cx),cy_(cy) 
  {}
  Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
  Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w );
  Vector2d camera2pixel( const Vector3d& p_c );
  Vector3d pixel2camera( const Vector2d& p_p, double depth=1 ); 
  Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth=1 );
  Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w );
};

}
#endif
