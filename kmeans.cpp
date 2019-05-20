//
// Created by dxc on 18-11-5.
//

#include "kmeans.h"

const double DIST_NEAR_ZERO = 0.01;//聚类中心变化距离限制


bool KMeans::InitKCenter()
{
    mv_center.resize(m_k);
    mv_center[0] = { mv_pntcloud[0].pnt.x, mv_pntcloud[0].pnt.y, mv_pntcloud[0].pnt.z, 0};
    mv_center[1] = { mv_pntcloud[mv_pntcloud.size()-1].pnt.x, mv_pntcloud[mv_pntcloud.size() - 1].pnt.y, mv_pntcloud[mv_pntcloud.size() - 1].pnt.z, 0};
    return true;
}


bool KMeans::SetInputCloud(vector<PointT> filename)
{
    mv_pntcloud.clear();

    int linenum = filename.size();

    for (int i = 0; i <linenum; ++i)
    {
        st_point point;
        point.pnt.x = filename[i].x;
        point.pnt.y = filename[i].y;
        point.pnt.z = filename[i].z;
        point.pnt.l = filename[i].label;

        point.groupID = 0;

        mv_pntcloud.push_back(point);
    }

    return true;
}

bool KMeans::Cluster()
{
    std::vector<st_pointxyzl> v_center(mv_center.size());

    do
    {
        for (int i = 0, pntCount = mv_pntcloud.size(); i < pntCount; ++i)
        {
            double min_dist = DBL_MAX;
            int pnt_grp = 0;
            for (int j = 0; j < m_k; ++j)
            {
                double dist = DistBetweenPoints(mv_pntcloud[i].pnt, mv_center[j]);
                if (min_dist - dist > 0.000001)
                {
                    min_dist = dist;
                    pnt_grp = j;
                }
            }
            m_grp_pntcloud[pnt_grp].push_back(st_point(mv_pntcloud[i].pnt, pnt_grp));
        }

        //保存上一次迭代的中心点
        for (size_t i = 0; i < mv_center.size(); ++i)
        {
            v_center[i] = mv_center[i];
        }

        if (!UpdateGroupCenter(m_grp_pntcloud, mv_center))
        {
            return false;
        }
        if (!ExistCenterShift(v_center, mv_center))
        {
            break;
        }
        for (int i = 0; i < m_k; ++i) {
            m_grp_pntcloud[i].clear();
        }

    } while (true);
    return true;
}

double KMeans::DistBetweenPoints(st_pointxyzl &p1, st_pointxyzl &p2)
{
    double dist = 0;
    double x_diff = 0, y_diff = 0, z_diff = 0; //R_diff = 0, G_diff = 0, B_diff = 0, I_diff = 0;//加入强度
    x_diff = p1.x - p2.x;
    y_diff = p1.y - p2.y;
    z_diff = p1.z - p2.z;

    dist = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff); //+ R_diff * R_diff + G_diff * G_diff + B_diff * B_diff + I_diff * I_diff);
    return dist;
}

bool KMeans::UpdateGroupCenter(std::vector<VecPoint_t> &grp_pntcloud, std::vector<st_pointxyzl> &center)
{
    if (center.size() != m_k)
    {
        cout << "类别的个数不为K\n" << endl;
        return false;
    }
    for (int i = 0; i < m_k; ++i)
    {
        double x = 0, y = 0, z = 0, R = 0, G = 0, B = 0, I = 0;//加入强度
        int pnt_num_in_grp = grp_pntcloud[i].size();
        for (int j = 0; j < pnt_num_in_grp; ++j)
        {
            x += grp_pntcloud[i][j].pnt.x;
            y += grp_pntcloud[i][j].pnt.y;
            z += grp_pntcloud[i][j].pnt.z;
        }
        x /= pnt_num_in_grp;
        y /= pnt_num_in_grp;
        z /= pnt_num_in_grp;

        center[i].x = x;
        center[i].y = y;
        center[i].z = z;
    }
    return true;
}
//是否存在中心点移动
bool KMeans::ExistCenterShift(std::vector<st_pointxyzl> &prev_center, std::vector<st_pointxyzl> &cur_center)
{
    for (int i = 0; i < m_k; ++i)
    {
        double dist = DistBetweenPoints(prev_center[i], cur_center[i]);
        if (dist > DIST_NEAR_ZERO)
        {
            return true;
        }
    }
    return false;
}
//将聚类的点分别存到各自的txt文件中
vector<vector<PointT>> KMeans::SaveFile()
{
    vector<vector<PointT>> out(2);
    for (int i = 0; i < m_grp_pntcloud[0].size();i++) {
        PointT x1;
        x1.x=m_grp_pntcloud[0][i].pnt.x;x1.y=m_grp_pntcloud[0][i].pnt.y;x1.z=m_grp_pntcloud[0][i].pnt.z;
        x1.label=m_grp_pntcloud[0][i].pnt.l;
        out[0].push_back(x1);
    }
    for (int i = 0; i < m_grp_pntcloud[1].size(); i++) {
        PointT x1;
        x1.x=m_grp_pntcloud[1][i].pnt.x;x1.y=m_grp_pntcloud[1][i].pnt.y;x1.z=m_grp_pntcloud[1][i].pnt.z;
        x1.label=m_grp_pntcloud[1][i].pnt.l;
        out[1].push_back(x1);
    }

    return out;
}

//将点数大于m的点集二分类并输出
vector<vector<PointT>> KMeans::segment(vector<vector<PointT>> DB, int m){
    vector<vector<PointT>> overseg;//最终存储每个过分割块
    stack<vector<PointT>> stacks;
    for (int i = 0; i < DB.size();i++) {
        stacks.push(DB[i]);
    }

    //循环stack，调用kmeans，if vector<point>.size()>NUM_Kmeans,则加入stack
    while (!stacks.empty()) {
        vector<PointT> kmean ;
        kmean= stacks.top();
        stacks.pop();
        vector<PointT> kmean1;//存储kmeans后的第一个点云
        vector<PointT> kmean2;//存储kmeans后的第二个点云
        if (kmean.size() > m) {
            SetInputCloud(kmean);
            SetK();
            InitKCenter();
            Cluster();
            kmean1 = SaveFile()[0];
            kmean2 = SaveFile()[1];

            if (kmean1.size() > m)
                stacks.push(kmean1);
            else if(kmean1.size()!=0)
                overseg.push_back(kmean1);
            if (kmean2.size() > m)
                stacks.push(kmean2);
            else if(kmean2.size()!=0)
                overseg.push_back(kmean2);
        }
        else if(kmean.size()>0)
            overseg.push_back(kmean);
    }

    ofstream overseg_display("./Output/Overseg.txt", std::ios_base::out);
    ofstream overseg_centroid("./Output/Centroid.txt", std::ios_base::out);
    for(int i=0;i<overseg.size();i++){
        int r=rand()%255;
        int g=rand()%255;
        int b=rand()%255;
        double sumx=0,sumy=0,sumz=0;
        for(int j=0;j<overseg[i].size();j++){
            sumx+=overseg[i][j].x;
            sumy+=overseg[i][j].y;
            sumz+=overseg[i][j].z;
            overseg_display << overseg[i][j].x << " " << overseg[i][j].y << " " << overseg[i][j].z << " "
                            << r << " " << g << " " << b << " " <<endl;
        }
        overseg_centroid << sumx/overseg[i].size() << " " << sumy/overseg[i].size() << " " << sumz/overseg[i].size() << " "
                        << r << " " << g << " " << b << " " <<endl;
    }

    for(int i=0;i<overseg.size();i++){
        if(overseg[i].size()==0)
            cout<<"出错.........................."<<endl;
    }

    return overseg;
}