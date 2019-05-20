//
// Created by dxc on 18-12-5.
//
#include "meanshift.h"

double p2p(PointT a, PointT b) {//点到点之间的距离
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z));
}

float Gaussian(float x){//高斯函数
    return 0.4*exp(-x*x/2);
}

vector<vector<int>> Meanshift(pcl::PointCloud<PointT>::Ptr &cloud, pcl::octree::OctreePointCloudSearch<PointT> &octree){
    int len=cloud->points.size();
    ///
    vector<map<int, int>> points_label(len);//根据这个判断每个点的label
    set<int> index;//已经被标记过的点
    int clusters = 0;//目前有几类
    vector<PointT> centers;//最终的聚类中心,与clusters是对应关系

    ///MeanShift聚类
    for (int i = 0; i < len;i++) {
        auto iter1 = index.find(i);
        if (iter1 == index.end()) {
            int LABEL=cloud->points[i].label;

            int cluster = 0;//给每次迭代赋的label值
            vector<int> index_;//每次迭代包含的点的索引
            PointT center_past;
            center_past.x=0;center_past.y=0;center_past.z=-1000;//之后给一个不可能的数
            PointT center_curr = cloud->points[i];

            while (p2p(center_past, center_curr) > threshold2) {
                center_past = center_curr;
                vector<int> pointIdxRadiusSearch;
                vector<float> pointRadiusSquaredDistance;
                PointT center;
                center.x=0;center.y=0;center.z=0;//初值给0
                octree.radiusSearch(center_curr, bandwidth2, pointIdxRadiusSearch, pointRadiusSquaredDistance);

                float factors=0;
                for (int j = 0; j < pointIdxRadiusSearch.size();j++) {
                    index_.push_back(pointIdxRadiusSearch[j]);

                    float factor1=1;
                    if(cloud->points[pointIdxRadiusSearch[j]].label!=LABEL)
                        factor1=0.3;

                    float factor2=Gaussian(pow(p2p(center_past,cloud->points[pointIdxRadiusSearch[j]])/(h2*h2),2));
                    float factor=factor1*factor2;

                    center.x+=cloud->points[pointIdxRadiusSearch[j]].x*factor;
                    center.y+=cloud->points[pointIdxRadiusSearch[j]].y*factor;
                    center.z+=cloud->points[pointIdxRadiusSearch[j]].z*factor;

                    factors+=factor;
                }
                center.x/=factors;
                center.y/=factors;
                center.z/=factors;

                center_curr = center;
            }
            if (centers.size() == 0) {
                centers.push_back(center_curr);
                cluster = clusters;
                clusters++;
            }
            else {
                int iter2;//存放最近的中心点再centers中的索引
                float min_dis = threshold2_+1.0;//用于迭代
                for (int j = 0; j < centers.size();j++) {
                    if (p2p(center_curr, centers[j]) < min_dis) {
                        min_dis = p2p(center_curr, centers[j]);
                        iter2 = j;
                    }
                }
                if (min_dis < threshold2_)
                    cluster = iter2;
                else {
                    centers.push_back(center_curr);
                    cluster = clusters;
                    clusters++;
                }
            }
            for (vector<int>::iterator it = index_.begin(); it != index_.end(); it++) {
                index.insert(*it);
                points_label[*it][cluster]++;
            }
        }
    }

    ///返回结果
    vector<vector<int>> result(clusters);
    for (int i = 0; i < len;i++) {
        int label=0;
        int max=0;
        for(auto it=points_label[i].begin();it!=points_label[i].end();it++){
            if(it->second>max){
                max=it->second;
                label=it->first;
            }
        }
        result[label].push_back(i);
    }

    vector<vector<int>> result1;
    for(int i=0;i<result.size();i++){
        if(result[i].size()>0){
            result1.push_back(result[i]);
        }
    }

    return result1;
}