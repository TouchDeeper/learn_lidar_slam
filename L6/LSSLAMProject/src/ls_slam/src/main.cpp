#include <gaussian_newton.h>
#include <readfile.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <ceres/ceres.h>

//for visual
void PublishGraphForVisulization(ros::Publisher* pub,
                                 std::vector<Eigen::Vector3d>& Vertexs,
                                 std::vector<Edge>& Edges,
                                 int color = 0)
{
    visualization_msgs::MarkerArray marray;

    //point--red
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.id = 0;
    m.ns = "ls-slam";
    m.type = visualization_msgs::Marker::SPHERE;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;

    if(color == 0)
    {
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
    }
    else
    {
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
    }

    m.color.a = 1.0;
    m.lifetime = ros::Duration(0);

    //linear--blue
    visualization_msgs::Marker edge;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.action = visualization_msgs::Marker::ADD;
    edge.ns = "karto";
    edge.id = 0;
    edge.type = visualization_msgs::Marker::LINE_STRIP;
    edge.scale.x = 0.1;
    edge.scale.y = 0.1;
    edge.scale.z = 0.1;

    if(color == 0)
    {
        edge.color.r = 0.0;
        edge.color.g = 0.0;
        edge.color.b = 1.0;
    }
    else
    {
        edge.color.r = 1.0;
        edge.color.g = 0.0;
        edge.color.b = 1.0;
    }
    edge.color.a = 1.0;

    m.action = visualization_msgs::Marker::ADD;
    uint id = 0;

    //加入节点
    for (uint i=0; i<Vertexs.size(); i++)
    {
        m.id = id;
        m.pose.position.x = Vertexs[i](0);
        m.pose.position.y = Vertexs[i](1);
        marray.markers.push_back(visualization_msgs::Marker(m));
        id++;
    }

    //加入边
    for(int i = 0; i < Edges.size();i++)
    {
        Edge tmpEdge = Edges[i];
        edge.points.clear();

        geometry_msgs::Point p;
        p.x = Vertexs[tmpEdge.xi](0);
        p.y = Vertexs[tmpEdge.xi](1);
        edge.points.push_back(p);

        p.x = Vertexs[tmpEdge.xj](0);
        p.y = Vertexs[tmpEdge.xj](1);
        edge.points.push_back(p);
        edge.id = id;

        marray.markers.push_back(visualization_msgs::Marker(edge));
        id++;
    }

    pub->publish(marray);
}

class LoopClosureAnalytic:public ceres::SizedCostFunction<3,3,3>{
public:
    LoopClosureAnalytic(Eigen::Vector3d observation, Eigen::Matrix3d information){
        observation_ = observation;
        sqrt_information_ = information.array().sqrt();
    }
    virtual ~LoopClosureAnalytic(){}
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {

        Eigen::Vector3d xi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Vector3d xj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Vector3d ei;
        Eigen::Matrix3d Ai;
        Eigen::Matrix3d Bj;
        CalcJacobianAndError(xi,xj,observation_,ei,Ai,Bj);
//        Eigen::Matrix3d Xi = PoseToTrans(xi);
//        Eigen::Matrix3d Xj = PoseToTrans(xj);
//        Eigen::Matrix3d Z  = PoseToTrans(observation_);
//        Eigen::Matrix3d Ei = Z.inverse() *  Xi.inverse() * Xj;
//        Eigen::Matrix3d ei = TransToPose(Ei);

        ei = sqrt_information_ * ei;
        residuals[0] = ei(0);
        residuals[1] = ei(1);
        residuals[2] = ei(2);
        if(jacobians){
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> > jacobian_xi(jacobians[0]);
                jacobian_xi = Ai;
            }
            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> > jacobian_xj(jacobians[1]);
                jacobian_xj = Bj;
            }
        }
        return true;

    }

private:
    Eigen::Vector3d observation_;
    Eigen::Matrix3d sqrt_information_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ls_slam");

    ros::NodeHandle nodeHandle;

    // beforeGraph
    ros::Publisher beforeGraphPub,afterGraphPub,ceresGraphPub;
    beforeGraphPub = nodeHandle.advertise<visualization_msgs::MarkerArray>("beforePoseGraph",1,true);
    afterGraphPub  = nodeHandle.advertise<visualization_msgs::MarkerArray>("afterPoseGraph",1,true);
    ceresGraphPub  = nodeHandle.advertise<visualization_msgs::MarkerArray>("ceresPoseGraph",1,true);


    std::string VertexPath = "/home/wang/CLionProjects/lidar_slam/L6/LSSLAMProject/src/ls_slam/data/test_quadrat-v.dat";
    std::string EdgePath = "/home/wang/CLionProjects/lidar_slam/L6/LSSLAMProject/src/ls_slam/data/test_quadrat-e.dat";

//    std::string VertexPath = "/home/eventec/LSSLAMProject/src/ls_slam/data/intel-v.dat";
//    std::string EdgePath = "/home/eventec/LSSLAMProject/src/ls_slam/data/intel-e.dat";

    std::vector<Eigen::Vector3d> Vertexs;
    std::vector<Edge> Edges;

    ReadVertexInformation(VertexPath,Vertexs);
    ReadEdgesInformation(EdgePath,Edges);

    PublishGraphForVisulization(&beforeGraphPub,
                                Vertexs,
                                Edges);

    double initError = ComputeError(Vertexs,Edges);
    std::cout <<"initError:"<<initError<<std::endl;

    int maxIteration = 100;
    double epsilon = 1e-4;
    //ceres
    double vertex[Vertexs.size()][3];
    ceres::Problem problem;
    for (int l = 0; l < Vertexs.size(); ++l) {
        vertex[l][0] = Vertexs[l](0);
        vertex[l][1] = Vertexs[l](1);
        vertex[l][2] = Vertexs[l](2);
        problem.AddParameterBlock(vertex[l],3);
    }
    problem.SetParameterBlockConstant(vertex[0]);
    for (int m = 0; m < Edges.size(); ++m) {
        ceres::CostFunction* cost_function = new LoopClosureAnalytic(Edges[m].measurement, Edges[m].infoMatrix);
        problem.AddResidualBlock(cost_function,NULL,vertex[Edges[m].xi],vertex[Edges[m].xj]);//pointer
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);
    std::cout << summary.BriefReport() << "\n";
    std::vector<Eigen::Vector3d> Vertexs_ceres(Vertexs.size());
    for (int n = 0; n < Vertexs_ceres.size(); ++n) {
        Vertexs_ceres[n](0) = vertex[n][0];
        Vertexs_ceres[n](1) = vertex[n][1];
        Vertexs_ceres[n](2) = vertex[n][2];
    }

    double finalError_ceres  = ComputeError(Vertexs_ceres,Edges);

    std::cout <<"FinalError_ceres:"<<finalError_ceres<<std::endl;
    PublishGraphForVisulization(&ceresGraphPub,
                                Vertexs_ceres,
                                Edges);

    //gaussian newton by hand
    for(int i = 0; i < maxIteration;i++)
    {
        std::cout <<"Iterations:"<<i<<std::endl;
        Eigen::VectorXd dx = LinearizeAndSolve(Vertexs,Edges);

        //进行更新
        //TODO--Start
        for (int j = 0; j < Vertexs.size(); ++j) {
            Vertexs[j](0) += dx(3*j);
            Vertexs[j](1) += dx(3*j+1);
            Vertexs[j](2) += dx(3*j+2);
        }
        //TODO--End

        double maxError = -1;
        for(int k = 0; k < 3 * Vertexs.size();k++)
        {
            if(maxError < std::fabs(dx(k)))
            {
                maxError = std::fabs(dx(k));
            }
        }

        if(maxError < epsilon)
            break;
    }


    double finalError  = ComputeError(Vertexs,Edges);

    std::cout <<"FinalError:"<<finalError<<std::endl;

    PublishGraphForVisulization(&afterGraphPub,
                                Vertexs,
                                Edges,1);

    ros::spin();



    return 0;
}




