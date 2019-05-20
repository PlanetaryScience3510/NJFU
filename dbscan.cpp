//
// Created by dxc on 18-11-15.
//

#include "dbscan.h"

vector<vector<PointT>> DBSCAN(pcl::PointCloud<PointT>::Ptr cloud, double eps, int min_pets){
    vector<point> corecloud;//构建核心点集
    vector<point> allcloud;

    int len=cloud->points.size();

    pcl::octree::OctreePointCloudSearch<PointT> octree(0.5);//初始化octree
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    for(size_t i=0;i<len;i++){
        point pt=point(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,cloud->points[i].label);
        allcloud.push_back(pt);
    }

    for (size_t i = 0; i < len; i++)
    {
        vector<int> radiussearch;//存放点的索引
        vector<float> radiusdistance;//存放点的距离平方
        octree.radiusSearch(cloud->points[i], eps, radiussearch, radiusdistance);//八叉树的邻域搜索
        if (radiussearch.size() > min_pets) {
            allcloud[i].pointtype = 3;
            corecloud.push_back(allcloud[i]);
        }
    }

    pcl::PointCloud<PointT>::Ptr corepointcloud(new pcl::PointCloud<PointT>);
    corepointcloud->points.resize(corecloud.size());
    for (int i = 0; i < corecloud.size(); i++) {
        corepointcloud->points[i].x = corecloud[i].x;
        corepointcloud->points[i].y = corecloud[i].y;
        corepointcloud->points[i].z = corecloud[i].z;
    }
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(corepointcloud);

    for (int i = 0; i<corecloud.size(); i++) {
        vector<int> pointIdxNKNSearch;
        vector<float> pointRadiusSquaredDistance;
        kdtree.radiusSearch(corepointcloud->points[i], eps, pointIdxNKNSearch, pointRadiusSquaredDistance);
        for (int j = 0; j < pointIdxNKNSearch.size(); j++) {
            corecloud[i].corepts.push_back(pointIdxNKNSearch[j]);
        }
    }
    //将所有核心点根据是否密度可达归类，改变核心点cluster的值
    int outcluster = 0;
    for (int i = 0; i<corecloud.size(); i++) {
        stack<point*> ps;
        if (corecloud[i].visited == 1) continue;
        outcluster++;
        corecloud[i].cluster = outcluster;
        ps.push(&corecloud[i]);
        point *v;
        //将密度可达的核心点归为一类
        while (!ps.empty()) {
            v = ps.top();
            v->visited = 1;
            ps.pop();
            for (int j = 0; j<v->corepts.size(); j++) {
                if (corecloud[v->corepts[j]].visited == 1) continue;
                corecloud[v->corepts[j]].cluster = corecloud[i].cluster;
                corecloud[v->corepts[j]].visited = 1;
                ps.push(&corecloud[v->corepts[j]]);
            }
        }
    }
    //找出所有的边界点，噪声点，对边界点分类，更改其cluster
    for (int i = 0; i<len; i++) {
        if (allcloud[i].pointtype == 3) continue;
        vector<int> pointIdxNKNSearch;//存放点的索引
        vector<float> pointRadiusSquaredDistance;
        kdtree.nearestKSearch(cloud->points[i], 1, pointIdxNKNSearch, pointRadiusSquaredDistance);
        allcloud[i].pointtype = 2;
        allcloud[i].cluster = corecloud[pointIdxNKNSearch[0]].cluster;
    }

    vector < vector<point >> Db(outcluster);
    for (int i = 0; i<corecloud.size(); i++) {
        Db[corecloud[i].cluster - 1].push_back(corecloud[i]);
    }
    for (int i = 0; i < len;i++) {
        if (allcloud[i].pointtype != 3) {
            Db[allcloud[i].cluster - 1].push_back(allcloud[i]);
        }
    }

    vector<vector<PointT>> DB(Db.size());
    for(int i=0;i<Db.size();i++){
        for(int j=0;j<Db[i].size();j++){
            PointT p;
            p.x=Db[i][j].x;p.y=Db[i][j].y;p.z=Db[i][j].z;
            p.label=Db[i][j].label;
            DB[i].push_back(p);
        }
    }

    ofstream DBSCAN_display("./Output/DBSCAN.txt",ios::out);
    for (size_t j = 0; j < outcluster; ++j)
    {
        if (Db.size() > 0) {
            int r = rand() % 256;
            int g = rand() % 256;
            int b = rand() % 256;
            for (int i = 0; i < Db[j].size();++i) {
                DBSCAN_display << Db[j][i].x << "\t" << Db[j][i].y << "\t" << Db[j][i].z << "\t" << r << "\t" << g << "\t" << b << endl;
            }
        }
    }

    return DB;
};

vector<vector<int>> DBSCAN(pcl::PointCloud<PointT>::Ptr cloud, double eps, int min_pets, vector<int> label){
    vector<point> corecloud;//构建核心点集
    vector<point> allcloud;

    int len=cloud->points.size();

    pcl::octree::OctreePointCloudSearch<PointT> octree(1.0);//初始化octree
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    for(size_t i=0;i<len;i++){
        point pt=point(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,cloud->points[i].label);
        allcloud.push_back(pt);
    }

    for (size_t i = 0; i < len; i++)
    {
        vector<int> radiussearch;//存放点的索引
        vector<float> radiusdistance;//存放点的距离平方
        octree.radiusSearch(cloud->points[i], 2*eps, radiussearch, radiusdistance);//八叉树的邻域搜索
        int k=0;
        for(size_t j=0;j<radiussearch.size();j++){
            if(label[i]==label[radiussearch[j]])
                k++;
        }
        if (k > min_pets) {
            allcloud[i].pointtype = 3;
            corecloud.push_back(allcloud[i]);
        }
    }

    pcl::PointCloud<PointT>::Ptr corepointcloud(new pcl::PointCloud<PointT>);
    corepointcloud->points.resize(corecloud.size());
    for (int i = 0; i < corecloud.size(); i++) {
        corepointcloud->points[i].x = corecloud[i].x;
        corepointcloud->points[i].y = corecloud[i].y;
        corepointcloud->points[i].z = corecloud[i].z;
    }
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(corepointcloud);

    for (int i = 0; i<corecloud.size(); i++) {
        vector<int> pointIdxNKNSearch;
        vector<float> pointRadiusSquaredDistance;
        kdtree.radiusSearch(corepointcloud->points[i], eps, pointIdxNKNSearch, pointRadiusSquaredDistance);
        for (int j = 0; j < pointIdxNKNSearch.size(); j++) {
            corecloud[i].corepts.push_back(pointIdxNKNSearch[j]);
        }
    }
    //将所有核心点根据是否密度可达归类，改变核心点cluster的值
    int outcluster = 0;
    for (int i = 0; i<corecloud.size(); i++) {
        stack<point*> ps;
        if (corecloud[i].visited == 1) continue;
        outcluster++;
        corecloud[i].cluster = outcluster;
        ps.push(&corecloud[i]);
        point *v;
        //将密度可达的核心点归为一类
        while (!ps.empty()) {
            v = ps.top();
            v->visited = 1;
            ps.pop();
            for (int j = 0; j<v->corepts.size(); j++) {
                if (corecloud[v->corepts[j]].visited == 1) continue;
                corecloud[v->corepts[j]].cluster = corecloud[i].cluster;
                corecloud[v->corepts[j]].visited = 1;
                ps.push(&corecloud[v->corepts[j]]);
            }
        }
    }
    //找出所有的边界点，噪声点，对边界点分类，更改其cluster
    for (int i = 0; i<len; i++) {
        vector<int> pointIdxNKNSearch;//存放点的索引
        vector<float> pointRadiusSquaredDistance;
        kdtree.nearestKSearch(cloud->points[i], 1, pointIdxNKNSearch, pointRadiusSquaredDistance);
        if(sqrt(pointRadiusSquaredDistance[0])<eps)
            allcloud[i].cluster = corecloud[pointIdxNKNSearch[0]].cluster;
    }

    vector < vector<int >> Db(outcluster);
    for (int i = 0; i < len;i++) {
        if(allcloud[i].cluster!=0)
            Db[allcloud[i].cluster - 1].push_back(i);
    }

//    ofstream DBSCAN_display("./Output/DBSCAN_probility.txt",ios::out);
//    for (size_t i = 0; i < outcluster; ++i)
//    {
//        if (Db.size() > 0) {
//            int r = rand() % 256;
//            int g = rand() % 256;
//            int b = rand() % 256;
//            for (int j = 0; j < Db[i].size();++j) {
//                DBSCAN_display << cloud->points[Db[i][j]].x << "\t" << cloud->points[Db[i][j]].y << "\t" << cloud->points[Db[i][j]].z << "\t" << r << "\t" << g << "\t" << b << endl;
//            }
//        }
//    }

    return Db;
};