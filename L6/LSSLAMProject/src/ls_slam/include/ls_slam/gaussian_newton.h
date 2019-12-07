#ifndef GAUSSIAN_NEWTON_H
#define GAUSSIAN_NEWTON_H

#include <vector>
#include <eigen3/Eigen/Core>

typedef struct edge
{
  int xi,xj;
  Eigen::Vector3d measurement;
  Eigen::Matrix3d infoMatrix;
}Edge;


Eigen::VectorXd  LinearizeAndSolve(std::vector<Eigen::Vector3d>& Vertexs,
                                   std::vector<Edge>& Edges);

double ComputeError(std::vector<Eigen::Vector3d>& Vertexs,
                    std::vector<Edge>& Edges);

//位姿-->转换矩阵
Eigen::Matrix3d PoseToTrans(Eigen::Vector3d x);
//转换矩阵－－＞位姿
Eigen::Vector3d TransToPose(Eigen::Matrix3d trans);
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
                          Eigen::Vector3d& ei,Eigen::Matrix3d& Ai,Eigen::Matrix3d& Bj);




#endif
