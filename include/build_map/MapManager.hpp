#pragma once

#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <common.hpp>

typedef pcl::octree::OctreePointCloudSearch<PointType> OctreePointCloudSearch;
typedef OctreePointCloudSearch::Ptr OctreePtr;

class MapManager
{
public:
  MapManager(float map_resolution)
    : octree(new OctreePointCloudSearch(map_resolution)),
      map(new PointCloud),
      leafsize(map_resolution) {}

  ~MapManager(){}
  
  void update()
  {
    octree->setInputCloud(map);
    octree->addPointsFromInputCloud();
  }

  bool getNearestPoint(PointType &pointSch, PointType &pointSel)
  {
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    // K = 1
    if (octree->nearestKSearch(pointSch, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
      pointSel = octree->getInputCloud()->points[pointIdxNKNSearch[0]];
      return true;
    }
    else
    {
      return false;
    }
  }

  void AddFrameToMap(PointCloud::Ptr frame)
  {
    PointType tempPoint;
    for (int i = 0; i < frame->points.size(); i++)
    { // Add frame to Global Map
      tempPoint.x = frame->points[i].x;
      tempPoint.y = frame->points[i].y;
      tempPoint.z = frame->points[i].z;
      tempPoint.intensity = frame->points[i].intensity;
      tempPoint.data_n[0] = frame->points[i].data_n[0];
      tempPoint.data_n[1] = frame->points[i].data_n[1];
      if (!octree->isVoxelOccupiedAtPoint(tempPoint))
        octree->addPointToCloud(tempPoint, map);
    }
  }
  void UpdateMap(PointCloud::Ptr cur_frame)
  {
    octree->deleteTree();
    *map = *cur_frame;
    PointCloud::Ptr map_filtered(new PointCloud);
    grid.setInputCloud(map);
    grid.setLeafSize(leafsize, leafsize, leafsize);
    grid.filter(*map_filtered);

    *map = *map_filtered;
    octree->setInputCloud(map);
    octree->addPointsFromInputCloud();
  }
  PointCloud::Ptr getMapPtr() { return map; }

private:
  OctreePtr octree;
  PointCloud::Ptr map;
  pcl::VoxelGrid<PointType> grid;
  double leafsize;
};
#endif