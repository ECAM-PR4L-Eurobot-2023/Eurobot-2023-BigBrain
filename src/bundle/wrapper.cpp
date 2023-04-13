#include <cmath>
#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>

#include "gs2_computation.h"
#include "gs2_struct.h"

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::vector<int>);
PYBIND11_MAKE_OPAQUE(std::vector<float>);

PYBIND11_MODULE(wrapper, m) {
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("convert_distance_iterator", &convert_distance_iterator, "Compute the distance and angles", py::return_value_policy::reference);

    py::class_<DeviceParametersCpp>(m, "DeviceParametersCpp")
        .def(py::init<const float, const float, const float, const float, const float>())
        .def_readwrite("k0", &DeviceParametersCpp::k0)
        .def_readwrite("b0", &DeviceParametersCpp::b0)
        .def_readwrite("k1", &DeviceParametersCpp::k1)
        .def_readwrite("b1", &DeviceParametersCpp::b1)
        .def_readwrite("bias", &DeviceParametersCpp::bias);

    py::class_<SpatialParametersCpp>(m, "SpatialParametersCpp")
        .def(py::init<const float, const float, const float, const float, const float>())
        .def_readwrite("angle_offset", &SpatialParametersCpp::angle_offset)
        .def_readwrite("x_offset", &SpatialParametersCpp::x_offset)
        .def_readwrite("y_offset", &SpatialParametersCpp::y_offset)
        .def_readwrite("norm", &SpatialParametersCpp::norm)
        .def_readwrite("angle", &SpatialParametersCpp::angle);
}
