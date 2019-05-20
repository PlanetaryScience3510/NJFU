//
// Created by dxc on 18-12-5.
//

#ifndef CRF_MEANSHIFT_H
#define CRF_MEANSHIFT_H

#include <vector>
#include <map>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#define bandwidth2 4.0
#define threshold2 1.5
#define threshold2_ 3.0

#define h1 1.2
#define h2 3


typedef pcl::PointXYZL PointT;

using namespace std;

double p2p(PointT a, PointT b);//点到点之间的距离

float Gaussian(float x);//高斯函数

vector<vector<int>> Meanshift(pcl::PointCloud<PointT>::Ptr &cloud, pcl::octree::OctreePointCloudSearch<PointT> &octree);

#endif //CRF_MEANSHIFT_H
