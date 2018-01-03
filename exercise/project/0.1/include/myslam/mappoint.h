#ifndef MAPPOINT_H
#define MAPPOINT_H
#include "camera.h"
namespace myslam
{
  class Frame;
  class MapPoint
  {
  public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long      id_; // ID
    Vector3d    pos_;       // Position in world
    Vector3d    norm_;      // Normal of viewing direction 什么意思？
    Mat         descriptor_; // Descriptor for matching 
    int         observed_times_;    // being observed by feature matching algo.观察次数
    int         correct_times_;     // being an inliner in pose estimation。匹配次数。
    
    MapPoint();
    MapPoint( long id, Vector3d position, Vector3d norm );

    // factory function
    static MapPoint::Ptr createMapPoint();
  };
}
#endif