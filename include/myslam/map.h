//
// Created by dhp on 5/05/19.
//

#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam
{
class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    //所有场景点的集合，键值为场景点的id，keyframes_同理
    unordered_map<unsigned long, MapPoint::Ptr >  map_points_;        
    unordered_map<unsigned long, Frame::Ptr >     keyframes_;         

    Map() {}
    
    void insertKeyFrame( Frame::Ptr frame );
    void insertMapPoint( MapPoint::Ptr map_point );
};
}

#endif
