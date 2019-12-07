#include "gaussian_newton.h"
#include <eigen3/Eigen/Jacobi>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Householder>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/LU>

#include <iostream>


//位姿-->转换矩阵
Eigen::Matrix3d PoseToTrans(Eigen::Vector3d x)
{
    Eigen::Matrix3d trans;
    trans << cos(x(2)),-sin(x(2)),x(0),
             sin(x(2)), cos(x(2)),x(1),
                     0,         0,    1;

    return trans;
}


//转换矩阵－－＞位姿
Eigen::Vector3d TransToPose(Eigen::Matrix3d trans)
{
    Eigen::Vector3d pose;
    pose(0) = trans(0,2);
    pose(1) = trans(1,2);
    pose(2) = atan2(trans(1,0),trans(0,0));

    return pose;
}

//计算整个pose-graph的误差
double ComputeError(std::vector<Eigen::Vector3d>& Vertexs,
                    std::vector<Edge>& Edges)
{
    double sumError = 0;
    for(int i = 0; i < Edges.size();i++)
    {
        Edge tmpEdge = Edges[i];
        Eigen::Vector3d xi = Vertexs[tmpEdge.xi];
        Eigen::Vector3d xj = Vertexs[tmpEdge.xj];
        Eigen::Vector3d z = tmpEdge.measurement;
        Eigen::Matrix3d infoMatrix = tmpEdge.infoMatrix;

        Eigen::Matrix3d Xi = PoseToTrans(xi);
        Eigen::Matrix3d Xj = PoseToTrans(xj);
        Eigen::Matrix3d Z  = PoseToTrans(z);

        Eigen::Matrix3d Ei = Z.inverse() *  Xi.inverse() * Xj;

        Eigen::Vector3d ei = TransToPose(Ei);


        sumError += ei.transpose() * infoMatrix * ei;
    }
    return sumError;
}


/**
 * @brief CalcJacobianAndError
 *         计算jacobian矩阵和error
 * @param xi    fromIdx
 * @param xj    toIdx
 * @param z     观测值:xj相对于xi的坐标
 * @param ei    计算的误差
 * @param Ai    相对于xi的Jacobian矩阵
 * @param Bj    相对于xj的Jacobian矩阵
 */
void CalcJacobianAndError(Eigen::Vector3d xi,Eigen::Vector3d xj,Eigen::Vector3d z,
                          Eigen::Vector3d& ei,Eigen::Matrix3d& Ai,Eigen::Matrix3d& Bj)
{
    //TODO--Start
    Ai.setZero();
    Bj.setZero();
    Eigen::Matrix3d Xi = PoseToTrans(xi);
    Eigen::Matrix3d Xj = PoseToTrans(xj);
    Eigen::Matrix3d Z  = PoseToTrans(z);
    Eigen::Matrix3d Ei = Z.inverse() *  Xi.inverse() * Xj;
    ei = TransToPose(Ei);
    Eigen::Matrix2d Xi_R = Xi.block(0,0,2,2);
    Eigen::Matrix2d Xj_R = Xj.block(0,0,2,2);
    Eigen::Matrix2d Z_R = Z.block(0,0,2,2);
    Ai.block(0,0,2,2) = -Z_R.transpose() * Xi_R.transpose();
    Eigen::Matrix2d RiT_xi;
    RiT_xi << -sin(xi(2)),cos(xi(2)),
                -cos(xi(2)),-sin(xi(2));
    Ai.block(0,2,2,1) = Z_R.transpose() * RiT_xi * (xj.segment(0,2) - xi.segment(0,2));
    Ai(2,2) = -1;
    Bj.block(0,0,2,2) = Z_R.transpose() * Xi_R.transpose();
    Bj(2,2) = 1;
    //TODO--end
}

/**
 * @brief LinearizeAndSolve
 *        高斯牛顿方法的一次迭代．
 * @param Vertexs   图中的所有节点
 * @param Edges     图中的所有边
 * @return          位姿的增量
 */
Eigen::VectorXd  LinearizeAndSolve(std::vector<Eigen::Vector3d>& Vertexs,
                                   std::vector<Edge>& Edges)
{
    //申请内存
    Eigen::MatrixXd H(Vertexs.size() * 3,Vertexs.size() * 3);
    Eigen::VectorXd b(Vertexs.size() * 3);

    H.setZero();
    b.setZero();

    //固定第一帧
    Eigen::Matrix3d I;
    I.setIdentity();
    H.block(0,0,3,3) += I;

    //构造H矩阵　＆ b向量
    for(int i = 0; i < Edges.size();i++)
    {
        //提取信息
        Edge tmpEdge = Edges[i];
        Eigen::Vector3d xi = Vertexs[tmpEdge.xi];
        Eigen::Vector3d xj = Vertexs[tmpEdge.xj];
        Eigen::Vector3d z = tmpEdge.measurement;
        Eigen::Matrix3d infoMatrix = tmpEdge.infoMatrix;

        //计算误差和对应的Jacobian
        Eigen::Vector3d ei;
        Eigen::Matrix3d Ai;
        Eigen::Matrix3d Bj;
        CalcJacobianAndError(xi,xj,z,ei,Ai,Bj);

         //TODO--Start
         H.block(3*tmpEdge.xi,3*tmpEdge.xi,3,3) += Ai.transpose() * infoMatrix * Ai;
         H.block(3*tmpEdge.xi,3*tmpEdge.xj,3,3) += Ai.transpose() * infoMatrix * Bj;
         H.block(3*tmpEdge.xj,3*tmpEdge.xj,3,3) += Bj.transpose() * infoMatrix * Bj;
         H.block(3*tmpEdge.xj,3*tmpEdge.xi,3,3) += H.block(3*tmpEdge.xi,3*tmpEdge.xj,3,3).transpose();
         b.segment(3*tmpEdge.xi,3) += Ai.transpose() * infoMatrix * ei;
         b.segment(3*tmpEdge.xj,3) += Bj.transpose() * infoMatrix * ei;
        //TODO--End
    }

    //求解
    Eigen::VectorXd dx;

    //TODO--Start
    dx = H.ldlt().solve(-b);
    //TODO--End

    return dx;
}











