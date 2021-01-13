#ifndef VERTEX_FORCE_OPTIMIZER_H
#define VERTEX_FORCE_OPTIMIZER_H

#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/solvers/BackEnd.h>

namespace ikbe
{

class VertexForceOptimizer
{

public:

    typedef std::shared_ptr<VertexForceOptimizer> Ptr;
    typedef std::unique_ptr<VertexForceOptimizer> UniquePtr;

    VertexForceOptimizer(const std::string& sensor_frame,
                         const std::vector<std::string>& corner_frames,
                         XBot::ModelInterface::ConstPtr model);

    const Eigen::VectorXd& compute(const Eigen::Vector6d& wrench);

    const std::vector<std::string>& getCornerFrames(){ return _corner_frames;}

private:

    OpenSoT::solvers::BackEnd::Ptr _solver;
    XBot::ModelInterface::ConstPtr _model;
    std::vector<std::string> _corner_frames;

    Eigen::MatrixXd _H;
    Eigen::MatrixXd _A;
    Eigen::VectorXd _b;

    Eigen::VectorXd _g;

    Eigen::VectorXd _lb;
    Eigen::VectorXd _ub;
};

}


#endif // VERTEX_FORCE_OPTIMIZER_H
