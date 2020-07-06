//
// Created by dhp on 5/05/19.
//


#ifndef MAPPOINT_H
#define MAPPOINT_H
#include <myslam/common_include.h>
namespace myslam
{
class MapPoint 
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    // 该场景点的id号
    unsigned long      id_; 
    // 该场景点的三维位置
    Vector3d    pos_; 
    // 
    Vector3d    norm_;      
    // 描述子
    Mat         descriptor_; 
    // 
    int         observed_times_;    
    int         matched_times_;  
    MapPoint();
    MapPoint( long id, Vector3d position, Vector3d norm );
   
    
    //static MapPoint::Ptr createMapPoint();
};
}
#endif
