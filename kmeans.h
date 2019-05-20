//
// Created by dxc on 18-11-5.
//

#ifndef CRF_KMEANS_H
#define CRF_KMEANS_H

#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZL PointT;

using namespace std;

typedef struct st_pointxyzl
{
    double x;
    double y;
    double z;
    int l;
}st_pointxyzl;
typedef struct st_point
{
    st_pointxyzl pnt;
    int groupID;
    st_point()
    {

    }
    st_point(st_pointxyzl &p, int id)
    {
        pnt = p;
        groupID = id;
    }
}st_point;

class KMeans
{
public:
    int m_k;

    typedef std::vector<st_point> VecPoint_t;

    VecPoint_t mv_pntcloud;//要聚类的点云
    std::vector<VecPoint_t> m_grp_pntcloud;//K类，每一类存储若干点
    std::vector<st_pointxyzl> mv_center;//每个类的中心

    KMeans()
    {
        m_k = 2;
    }

    inline void SetK()
    {
        m_k = 2;
        m_grp_pntcloud.resize(m_k);
    }

    //设置输入点云 
    //bool SetInputCloud(pcl::PointCloud<PointT>::Ptr pPntCloud);
    bool SetInputCloud(vector<PointT> filename);

    //初始化最初的K个类的中心
    bool InitKCenter();

    //聚类
    bool Cluster();

    //更新K类的中心
    bool UpdateGroupCenter(std::vector<VecPoint_t> &grp_pntcloud, std::vector<st_pointxyzl> &center);

    //计算两个点间的欧氏距离
    double DistBetweenPoints(st_pointxyzl &p1, st_pointxyzl &p2);

    //是否存在中心点移动
    bool ExistCenterShift(std::vector<st_pointxyzl> &prev_center, std::vector<st_pointxyzl> &cur_center);

    //将聚类的点分别存到各自的txt文件中
    vector<vector<PointT>> SaveFile();

    //将点数大于m的点集二分类并输出
    vector<vector<PointT>> segment(vector<vector<PointT>> DB, int m);

};

#endif //CRF_KMEANS_H
