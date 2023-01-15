#include "base_estimation/vertex_force_optimizer.h"
#include <OpenSoT/solvers/BackEndFactory.h>

using namespace ikbe;
using namespace XBot;

VertexForceOptimizer::VertexForceOptimizer(const std::string& sensor_frame,
                                           const std::vector<std::string>& corner_frames,
                                           ModelInterface::ConstPtr model):
    _corner_frames(corner_frames),
    _model(model)
{
    // num corners
    const int ncorners = corner_frames.size();

    // get qp solver
    _solver = OpenSoT::solvers::BackEndFactory(OpenSoT::solvers::solver_back_ends::qpOASES,
                                               ncorners,
                                               0,
                                               OpenSoT::HessianType::HST_POSDEF,
                                               1e6);

    // initialize A matrix as ones
    _A.setOnes(3, corner_frames.size());

    // fill A matrix
    for(int i = 0; i < ncorners; i++)
    {
        Eigen::Affine3d T;
        _model->getPose(corner_frames[i], sensor_frame, T);
        _A(1, i) = T.translation()[0];
        _A(2, i) = T.translation()[1];
    }

    // compute hessian (it's constant!)
    _H = _A.transpose()*_A;

    // initialize b vector
    _b.setZero(3);

    // set bounds
    const double fz_max = 1e6;
    _lb.setZero(ncorners);
    _ub.setZero(ncorners);
    _ub = fz_max * Eigen::VectorXd::Ones(ncorners);

    // initialize gradient
    _g.setZero(corner_frames.size());

    // initialize solver
    _solver->initProblem(_H,
                         _g,
                         Eigen::MatrixXd(0, 0),
                         Eigen::VectorXd(0),
                         Eigen::VectorXd(0),
                         _lb,
                         _ub);
}

const Eigen::VectorXd& VertexForceOptimizer::compute(const Eigen::Vector6d& wrench)
{
    // set b vector
    _b[0] = wrench[2];  // fz
    _b[1] = -wrench[4]; // -my
    _b[2] = wrench[3];  // mx

    // compute gradient
    _g.noalias() = -_A.transpose()*_b;

    // solve qp
    _solver->updateTask(_H, _g);
    _solver->solve();  // note: we assume it always succeeds

    return _solver->getSolution();
}
