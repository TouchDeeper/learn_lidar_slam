#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char** argv)
{
    // 机器人B在坐标系O中的坐标：
    Eigen::Vector3d O_P_B(3, 4, M_PI);

    // 坐标系B到坐标O的转换矩阵：
    Eigen::Matrix3d O_T_B;
    O_T_B << cos(O_P_B(2)), -sin(O_P_B(2)), O_P_B(0),
           sin(O_P_B(2)),  cos(O_P_B(2)), O_P_B(1),
              0,          0,        1;

    // 坐标系O到坐标B的转换矩阵:
    Eigen::Matrix3d B_T_O = O_T_B.inverse();

    // 机器人A在坐标系O中的坐标：
    Eigen::Vector3d O_P_A(1, 3, -M_PI / 2);

    // 求机器人A在机器人B中的坐标：
    Eigen::Vector3d B_P_A;
    // TODO 参照PPT
    // start your code here (5~10 lines)
    Eigen::Matrix3d O_T_A;
    O_T_A << cos(O_P_A(2)), -sin(O_P_A(2)), O_P_A(0),
            sin(O_P_A(2)),  cos(O_P_A(2)), O_P_A(1),
            0,          0,        1;
    Eigen::Matrix3d B_T_A = B_T_O * O_T_A;
    B_P_A[0] = B_T_A(0,2);
    B_P_A[1] = B_T_A(1,2);
    B_P_A[2] = atan2(B_T_A(1,0),B_T_A(0,0));

    // end your code here

    cout << "The right answer is BA: 2 1 1.5708" << endl;
    cout << "Your answer is BA: " << B_P_A.transpose() << endl;

    return 0;
}
