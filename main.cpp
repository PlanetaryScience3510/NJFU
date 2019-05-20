#include <iostream>
#include <fstream>
#include <vector>
#include <stack>
#include <set>
#include <string>
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "kmeans.h"
#include "dbscan.h"
#include "feature.h"
#include "meanshift.h"
#include "statistics.h"

#include "sospd.hpp"


#define eps 1
#define min_pets 8
#define NUM_Kmeans 100



using namespace std;

struct Probability{
    int label;
    double p2;
    double p1;
    double p0;
    Probability(int l,double a,double b,double c){
        label=l;
        p2=a;
        p1=b;
        p0=c;
    }
};

int main() {
    clock_t startTime,endTime;
    startTime = clock();
    ///---------------------------读取数据------------------
    cout<<"读取点云数据..."<<endl;
    pcl::PointCloud<PointT>::Ptr Traincloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr Testcloud(new pcl::PointCloud<PointT>);

    fstream OpenFile;
    string TrainNmae("./Input/allpoint_train_n.txt");
    string TestNmae("./Input/allpoint_test_n.txt");

    OpenFile.open(TrainNmae,ios::in);
    while(!OpenFile.eof()){
        double x,y,z,l,nx,ny,nz;
        OpenFile>>x>>y>>z>>l>>nx>>ny>>nz;
        PointT p;
        p.x=x;p.y=y;p.z=z;p.label=l;
        Traincloud->points.push_back(p);
    }
    OpenFile.close();

    OpenFile.open(TestNmae,ios::in);
    while(!OpenFile.eof()){
        double x,y,z,l,nx,ny,nz;
        OpenFile>>x>>y>>z>>l>>nx>>ny>>nz;
        PointT p;
        p.x=x;p.y=y;p.z=z;p.label=l;
        Testcloud->points.push_back(p);
    }
    OpenFile.close();

    ///----------------------------------------DBSCAN-------------------------------------
    cout<<"DBSCAN聚类..."<<endl;
    vector<vector<PointT>> DB_train = DBSCAN(Traincloud,eps,min_pets);
    vector<vector<PointT>> DB_test = DBSCAN(Testcloud,eps,min_pets);
    cout<<"训练集DBSCAN聚类数: "<<DB_train.size()<<endl;
    cout<<"测试集DBSCAN聚类数: "<<DB_test.size()<<endl;

    ///---------------------------------------Kmeans---------------------------------------
    cout<<"KMeans过分割..."<<endl;
    KMeans KM;
    vector<vector<PointT>> overseg_train = KM.segment(DB_train, NUM_Kmeans);
    vector<vector<PointT>> overseg_test = KM.segment(DB_test, NUM_Kmeans);
    cout<<"训练集过分割后数量: "<<overseg_train.size()<<endl;
    cout<<"测试集过分割后数量: "<<overseg_test.size()<<endl;

    ///-------------------------------------提取特征并分类-------------------------------
    cout<<"提取特征..."<<endl;
    ofstream TrainFeatures("./Output/TrainFeatures.txt",ios::out);
    ofstream TestFeatures("./Output/TestFeatures.txt",ios::out);

    vector<int> label_train_true(overseg_train.size(),0);
    vector<int> label_test_true(overseg_test.size(),0);

    Feature F;

    F.Initialization(Traincloud);
    for(size_t i=0;i<overseg_train.size();i++){
        //获取真值标签
        int l0=0,l1=0,l2=0;
        for(int j=0;j<overseg_train[i].size();j++){
            if(overseg_train[i][j].label==0)
                l0++;
            else if(overseg_train[i][j].label==1)
                l1++;
            else
                l2++;
        }
        if(l0>l1&&l0>l2)
            label_train_true[i]=0;
        else if(l1>l0&&l1>l2)
            label_train_true[i]=1;
        else
            label_train_true[i]=2;

        if(overseg_train[i].size()<5)
            continue;

        //输出特征
        vector<double> feature_train = F.ExtraFeature(overseg_train[i]);
        TrainFeatures<<label_train_true[i];
        for(size_t j=0;j<feature_train.size();j++) {
            TrainFeatures << "\t" << j + 1 << ":" << feature_train[j];
        }
        TrainFeatures<<endl;
    }


    F.Initialization(Testcloud);
    for(size_t i=0;i<overseg_test.size();i++){
        //获取真值标签
        int l0=0,l1=0,l2=0;
        for(int j=0;j<overseg_test[i].size();j++){
            if(overseg_test[i][j].label==0)
                l0++;
            else if(overseg_test[i][j].label==1)
                l1++;
            else
                l2++;
        }
        if(l0>l1&&l0>l2)
            label_test_true[i]=0;
        else if(l1>l0&&l1>l2)
            label_test_true[i]=1;
        else
            label_test_true[i]=2;

        //输出特征
        vector<double> feature_test = F.ExtraFeature(overseg_test[i]);
        TestFeatures<<label_test_true[i];
        for(size_t j=0;j<feature_test.size();j++) {
            TestFeatures << "\t" << j + 1 << ":" << feature_test[j];
        }
        TestFeatures<<endl;
    }

    cout<<"训练数据..."<<endl;
    system("svm-train -h 0 -b 1 ./Output/TrainFeatures.txt ./Output/classify.model");
    cout<<"数据分类..."<<endl;
    system("svm-predict -b 1 ./Output/TestFeatures.txt ./Output/classify.model ./Output/result");

    //预测出的概率值
    vector<Probability> P;
    vector<int> label_first;
    OpenFile.open("./Output/result");
    string s;
    getline(OpenFile,s);
    while(!OpenFile.eof()){
        int l;
        double p2,p1,p0;
        OpenFile>>l>>p2>>p1>>p0;
        Probability pro(l,p2,p1,p0);
        P.push_back(pro);
        label_first.push_back(l);
    }
    P.pop_back();
    label_first.pop_back();

    OpenFile.close();

    STATISTICS(overseg_test,label_first,"./Output/Classfy_first.txt");


    ///--------------------------------------MeanShift-------------------------------------
    cout<<"MeanShift聚类..."<<endl<<endl;

    pcl::PointCloud<PointT>::Ptr cloud_overseg(new pcl::PointCloud<PointT>);
    for(size_t i=0;i<overseg_test.size();i++){
        PointT p;
        double x=0,y=0,z=0;
        for(size_t j=0;j<overseg_test[i].size();j++){
            x+=overseg_test[i][j].x;
            y+=overseg_test[i][j].y;
            z+=overseg_test[i][j].z;
        }
        p.x=x/overseg_test[i].size();
        p.y=y/overseg_test[i].size();
        p.z=z/overseg_test[i].size();
        p.label=P[i].label;

        cloud_overseg->points.push_back(p);
    }

    pcl::octree::OctreePointCloudSearch<PointT> octree(5.0);
    octree.setInputCloud(cloud_overseg);
    octree.addPointsFromInputCloud();

    vector<vector<int>> ClusterIndex = Meanshift(cloud_overseg, octree);
    //vector<vector<int>> ClusterIndex = DBSCAN(cloud_overseg, 3,3,label_first);
    cout<<"MeanShift聚类数量:  "<<ClusterIndex.size()<<endl;

    ofstream MS("./Output/MeanShift.txt", std::ios_base::out);
    for(int i=0;i<ClusterIndex.size();i++) {
        int r=rand()%255;
        int g=rand()%255;
        int b=rand()%255;
        for(int j=0;j<ClusterIndex[i].size();j++){
            for(int k=0;k<overseg_test[ClusterIndex[i][j]].size();k++){
                MS << overseg_test[ClusterIndex[i][j]][k].x << " " << overseg_test[ClusterIndex[i][j]][k].y << " " << overseg_test[ClusterIndex[i][j]][k].z << " "
                   << r << " " << g << " " << b << " " <<endl;
            }
        }
    }

    ///----------------------CRF提取特征并分类---------------
    cout<<"设置能量函数..."<<endl;
    vector<int> label_optimize(label_first.size(),1);
    int max_label=3;//标签数量
    MultilabelEnergy energy(max_label);//初始化能量函数,3表示max_label

    //向能量函数中添加一阶项
    energy.addVar(label_first.size());//节点的数量
    for(size_t i=0;i<label_first.size();i++){
        vector<REAL> coeff(max_label);
        coeff[0]=(1-P[i].p0)*10000;
        coeff[1]=(1-P[i].p1)*10000;
        coeff[2]=(1-P[i].p2)*10000;

        energy.addUnaryTerm(i,coeff);
    }


    //向能量函数中添加二阶项
    for(size_t i=0;i<label_first.size();i++){
        //使用octree搜索cloud_overseg
        vector<int> pointIdx;
        vector<float> pointDistance;
        octree.nearestKSearch(cloud_overseg->points[i],2,pointIdx,pointDistance);

        const int a=pointIdx.size();

        std::unique_ptr<Clique> HighClique(std::unique_ptr<Clique>(new TwoClique<2>(pointIdx,0,10000*exp(-sqrt(pointDistance[1])/10))));
        energy.addClique( std::unique_ptr<Clique>(HighClique.release()));
    }

    //向能量函数中添加高阶项
    int mmm=0;
    for(size_t i=0;i<ClusterIndex.size();i++){
        //高阶鲁棒PnPotts模型的标签定义为当前的标签众数
        if(ClusterIndex[i].size()==1)
            continue;
        std::unique_ptr<Clique> HighClique(std::unique_ptr<Clique>(new RubustPottsClique(ClusterIndex[i],10,100000)));
        energy.addClique( std::unique_ptr<Clique>(HighClique.release()));
        if(ClusterIndex[i].size()>mmm)
            mmm=ClusterIndex[i].size();
    }
    cout<<mmm<<endl;

    cout<<"SOSPD方法优化能量函数..."<<endl;
    SubmodularIBFSParams sosParams;
    sosParams.ub = SoSGraph::UBfn::cvpr14;
    //SoSPD<> Sos(&energy,sosParams);
    SoSPD<SubmodularIBFS> Sos(&energy);
    //Sos.SetProposalCallback(AlphaProposal);
    Sos.SetLowerBound(true);
    Sos.Solve(100);

    cout<<"求解完成"<<endl;

    for(int i=0;i<label_optimize.size();i++) {
        label_optimize[i] = Sos.GetLabel(i);
    }

    STATISTICS(overseg_test,label_optimize,"./Output/Classfy_optimize.txt");

    endTime = clock();
    cout << "整体耗时 : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "秒" << endl;

    return 0;
}