#include <base_estimation/base_estimation.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/detail/common.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using namespace ikbe;

auto base_estimation_init = [](XBot::ModelInterface::Ptr model, std::string yaml_string, BaseEstimation::Options opt)
{
    return BaseEstimation(model, YAML::Load(yaml_string), opt);
};

auto update = [](BaseEstimation& self)
{
    Eigen::Affine3d pose;
    Eigen::Vector6d vel;
    Eigen::Vector6d raw_vel;

    self.update(pose, vel, raw_vel);

    return std::make_tuple(pose, vel, raw_vel);
};

PYBIND11_MODULE(pybase_estimation, m)
{
    py::class_<BaseEstimation::Options>(m, "Options")
        .def(py::init<>())
        .def_readwrite("dt", &BaseEstimation::Options::dt)
        .def_readwrite("log_enabled", &BaseEstimation::Options::log_enabled)
        .def_readwrite("contact_release_thr", &BaseEstimation::Options::contact_release_thr)
        .def_readwrite("contact_attach_thr", &BaseEstimation::Options::contact_attach_thr);

    py::class_<BaseEstimation, BaseEstimation::UniquePtr>(m, "BaseEstimation")
        .def(py::init(base_estimation_init),
             py::arg("model"),
             py::arg("est_model_pb"),
             py::arg("opt") = BaseEstimation::Options())
        .def_static("CreateDummyFtSensor", &BaseEstimation::CreateDummyFtSensor)
        .def("addImu", &BaseEstimation::addImu)
        .def("addSurfaceContact", &BaseEstimation::addSurfaceContact)
        .def("update", update)
        .def("reset", (bool (BaseEstimation::*) (const std::string&)) &BaseEstimation::reset)
        .def("reset", (void (BaseEstimation::*) ()) &BaseEstimation::reset);
}

