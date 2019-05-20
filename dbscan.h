//
// Created by dxc on 18-11-15.
//

#ifndef CRF_DBSCAN_H
#define CRF_DBSCAN_H

#include <iostream>
#include <vector>
#include <string>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/usc.h>

typedef pcl::PointXYZL PointT;

using namespace std;

class point
{
public:
    float x;
    float y;
    float z;
    int visited = 0;
    int pointtype = 1;//1噪声，2边界点，3核心点
    int cluster = 0;
    int label=0;
    vector<int> corepts;//存储邻域内点的索引
    point() {}
    point(float a, float b, float c)
    {
        x = a;
        y = b;
        z = c;
    }
    point(float a, float b, float c, int l)
    {
        x = a;
        y = b;
        z = c;
        label=l;
    }
};

//vector<point> corecloud;//构建核心点集
//vector<point> allcloud;

///普通的DBSCAN聚类
vector<vector<PointT>> DBSCAN(pcl::PointCloud<PointT>::Ptr cloud, double eps, int min_pets);

///先验概率的DBSCAN聚类
vector<vector<int>> DBSCAN(pcl::PointCloud<PointT>::Ptr cloud, double eps, int min_pets, vector<int> label);

#endif //CRF_DBSCAN_H