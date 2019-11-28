#include <map.h>
#include <tf/LinearMath/Scalar.h>
#include "gaussian_newton_method.h"


const double GN_PI = 3.1415926;

//进行角度正则化．
double GN_NormalizationAngle(double angle)
{
    if(angle > GN_PI)
        angle -= 2*GN_PI;
    else if(angle < -GN_PI)
        angle += 2*GN_PI;

    return angle;
}

Eigen::Matrix3d GN_V2T(Eigen::Vector3d vec)
{
    Eigen::Matrix3d T;
    T  << cos(vec(2)),-sin(vec(2)),vec(0),
            sin(vec(2)), cos(vec(2)),vec(1),
            0,           0,     1;

    return T;
}

//对某一个点进行转换．
Eigen::Vector2d GN_TransPoint(Eigen::Vector2d pt,Eigen::Matrix3d T)
{
    Eigen::Vector3d tmp_pt(pt(0),pt(1),1);
    tmp_pt = T * tmp_pt;
    return Eigen::Vector2d(tmp_pt(0),tmp_pt(1));
}



//用激光雷达数据创建势场．
map_t* CreateMapFromLaserPoints(Eigen::Vector3d map_origin_pt,
                                std::vector<Eigen::Vector2d> laser_pts,
                                double resolution)
{
    map_t* map = map_alloc();

    map->origin_x = map_origin_pt(0);
    map->origin_y = map_origin_pt(1);
    map->resolution = resolution;

    //固定大小的地图，必要时可以扩大．
    map->size_x = 10000;
    map->size_y = 10000;

    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);

    //高斯平滑的sigma－－固定死
    map->likelihood_sigma = 0.5;

    Eigen::Matrix3d Trans = GN_V2T(map_origin_pt);

    //设置障碍物
    for(int i = 0; i < laser_pts.size();i++)
    {
        Eigen::Vector2d tmp_pt = GN_TransPoint(laser_pts[i],Trans);

        int cell_x,cell_y;
        cell_x = MAP_GXWX(map,tmp_pt(0));
        cell_y = MAP_GYWY(map,tmp_pt(1));

        map->cells[MAP_INDEX(map,cell_x,cell_y)].occ_state = CELL_STATUS_OCC;
    }

    //进行障碍物的膨胀--最大距离固定死．
    map_update_cspace(map,0.5);

    return map;
}


/**
 * @brief InterpMapValueWithDerivatives
 * 在地图上的进行插值，得到coords处的势场值和对应的关于位置的梯度．
 * 返回值为Eigen::Vector3d ans
 * ans(0)表示势场值
 * ans(1:2)表示梯度
 * @param map
 * @param coords
 * @return
 */
Eigen::Vector3d InterpMapValueWithDerivatives(map_t* map,Eigen::Vector2d& coords)
{
    Eigen::Vector3d ans;
    //TODO
    double x = coords(0);
    double y = coords(1);
    int y0 = floor(coords(1));
    int y1 = ceil(coords[1]);
    int x0 = floor(coords[0]);
    int x1 = ceil(coords[0]);
//    assert(x0 > 0);
//    assert(x1 < map->size_x);
//    assert(y0 > 0);
//    assert(y1 < map->size_y);
    Eigen::Vector2d coord00(x0, y0);
    Eigen::Vector2d coord01(x0, y1);
    Eigen::Vector2d coord10(x1, y0);
    Eigen::Vector2d coord11(x1, y1);
//    std::cout<<"xy ="<<coords.transpose()<<"  "<<"x0y0 = "<<coord00.transpose()<<"  "<<"x0y1 = "<<coord01.transpose()<<"   x1y0 = "<<coord10.transpose()<<"   x1y1 = "<<coord11.transpose()<<std::endl;
//    std::cout<<"x1y1_index = "<<MAP_INDEX(map, x1, y1)<<std::endl;
//    std::cout<<"size_x = " <<map->size_x<<std::endl;
    double M_11 = map->cells[MAP_INDEX(map, x1, y1)].score;
    double M_01 = map->cells[MAP_INDEX(map, x0, y1)].score;
    double M_10 = map->cells[MAP_INDEX(map, x1, y0)].score;
    double M_00 = map->cells[MAP_INDEX(map, x0, y0)].score;
//    std::cout<<"M11 = "<<M_11<<std::endl;
    double y_y0 = y - y0;
    double y1_y0 = y1 - y0;
    double x_x0 = x - x0;
    double x1_x0 = x1 - x0;
    double y1_y = y1 - y;
    double x1_x = x1 - x;
    ans(0) = y_y0 / y1_y0 * (x_x0 / x1_x0 * M_11 + x1_x / x1_x0 * M_01) + y1_y / y1_y0 * (x_x0 / x1_x0 * M_10 + x1_x / x1_x0 * M_00);
    ans(1) = ( y_y0 / y1_y0 * (M_11- M_01) + y1_y / y1_y0 * (M_10 - M_00) ) / x1_x0;
    ans(2) = ( x_x0 / x1_x0 * (M_11- M_10) + x1_x / x1_x0 * (M_01 - M_00) ) / y1_y0;
    //END OF TODO

    return ans;
}


/**
 * @brief ComputeCompleteHessianAndb
 * 计算H*dx = b中的H和b
 * @param map
 * @param now_pose
 * @param laser_pts
 * @param H
 * @param b
 */
void ComputeHessianAndb(map_t* map, Eigen::Vector3d now_pose,
                        std::vector<Eigen::Vector2d>& laser_pts,
                        Eigen::Matrix3d& H, Eigen::Vector3d& b)
{
    H = Eigen::Matrix3d::Zero();
    b = Eigen::Vector3d::Zero();

    //TODO
    for (int i = 0; i < laser_pts.size(); ++i) {
        Eigen::Matrix3d now_T;
        now_T << cos(now_pose(2)), -sin(now_pose(2)), now_pose(0),
                sin(now_pose(2)), cos(now_pose(2)), now_pose(1),
                0,               0,               1;
        Eigen::Vector3d w_laser_pts = now_T * Eigen::Vector3d(laser_pts[i][0], laser_pts[i][1], 1);
        Eigen::Vector2d map_laser_pts;
        map_laser_pts[0] = (w_laser_pts[0] - map->origin_x) / map->resolution + map->size_x / 2.0;
        map_laser_pts[1] = (w_laser_pts[1] - map->origin_y) / map->resolution + map->size_y / 2.0;
        Eigen::Vector3d score_gradient = InterpMapValueWithDerivatives(map,map_laser_pts);
        Eigen::Vector2d M_mi(score_gradient[1], score_gradient[2]);
        Eigen::Matrix2d mi_si;
        mi_si << 1/map->resolution, 0,
                    0,             1/map->resolution;
        Eigen::Matrix<double,2,3> si_T;
        si_T << 1,  0,   -sin(now_pose[2])*laser_pts[i][0]-cos(now_pose[2])*laser_pts[i][1],
                0,  1,   cos(now_pose[2])*laser_pts[i][0]-sin(now_pose[2])*laser_pts[i][1];
        Eigen::Matrix<double,1,3> J = M_mi.transpose() * mi_si * si_T;
        H += J.transpose() * J;
        b += J.transpose() * (1 - score_gradient[0]);

    }

    //END OF TODO
}


/**
 * @brief GaussianNewtonOptimization
 * 进行高斯牛顿优化．
 * @param map
 * @param init_pose
 * @param laser_pts
 */
void GaussianNewtonOptimization(map_t*map,Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts)
{
    int maxIteration = 20;
    Eigen::Vector3d now_pose = init_pose;

    for(int i = 0; i < maxIteration;i++)
    {
        //TODO
        Eigen::Matrix3d H;
        Eigen::Vector3d b;
        ComputeHessianAndb(map, now_pose, laser_pts, H, b);
        Eigen::Vector3d delta_x = H.colPivHouseholderQr().solve(b);
//        Eigen::Vector3d delta_x = H.ldlt().solve(b);
//        Eigen::Matrix3d delta_T;
//        delta_T << cos(delta_x(2)), -sin(delta_x(2)), delta_x(0),
//                    sin(delta_x(2)), cos(delta_x(2)), delta_x(1),
//                    0,               0,               1;
//        Eigen::Matrix3d now_T;
//        now_T << cos(now_pose(2)), -sin(now_pose(2)), now_pose(0),
//                sin(now_pose(2)), cos(now_pose(2)), now_pose(1),
//                0,               0,               1;
//        now_T = now_T * delta_T;
        now_pose += delta_x;
        now_pose(2) = tfNormalizeAngle(now_pose(2));
//        now_pose[0] = now_T(0,2);
//        now_pose[1] = now_T(1,2);
//        now_pose[2] = atan2(now_T(1,0),now_T(0,0));
        //END OF TODO
    }
    init_pose = now_pose;

}
