#ifndef GS2_COMPUTATION_H
#define GS2_COMPUTATION_H

#include <stdlib.h>

#include <pybind11/numpy.h>

#include "gs2_struct.h"

#define CLOUD_POINT           (160)
#define ANGLE_P_X             (1.22)
#define ANGLE_P_Y             (5.315)
#define ANGLE_P_ANGLE         (22.5)
#define MIN_DISTANCE          (25)  // mm
#define MAX_DISTANCE          (300)  // mm
#define COMPUTE_DISTANCE_MASK (0x01FF)  // Because the distance is coded with 9 bits

namespace py = pybind11;

static void convert_distances_cpp(float *distance, float *angle, int calc_distance, int n, SpatialParametersCpp spatial_param, DeviceParametersCpp device_param) {
    float temp_theta, temp_dist, temp_x, temp_y;
    float angle_p_bias = (ANGLE_P_ANGLE + device_param.bias) * M_PI / 180;

    // Left camera
    if (n < CLOUD_POINT / 2) {
        n = (CLOUD_POINT / 2) - n;

        temp_theta = (device_param.b0 > 1 ? (device_param.k0 * n) - device_param.b0 : atan(device_param.k0 * n - device_param.b0) * 180 / M_PI);
        temp_dist = (calc_distance - ANGLE_P_X) / cos(((ANGLE_P_ANGLE + device_param.bias) - (temp_theta)) * M_PI / 180);
        temp_theta = temp_theta * M_PI / 180;
        temp_x = cos(angle_p_bias) * temp_dist * cos(temp_theta) + sin(angle_p_bias) * temp_dist * sin(temp_theta);
        temp_y = -sin(angle_p_bias) * temp_dist * cos(temp_theta) + cos(angle_p_bias) * temp_dist * sin(temp_theta);
        temp_x += ANGLE_P_X;
        temp_y -= ANGLE_P_Y;
    }   
    else {
        n = CLOUD_POINT - n;
        
        temp_theta = (device_param.b1 > 1 ? (device_param.k1 * n) - device_param.b1 : atan(device_param.k1 * n - device_param.b1) * 180 / M_PI);
        temp_dist = (calc_distance - ANGLE_P_X) / cos(((ANGLE_P_ANGLE + device_param.bias) + temp_theta) * M_PI / 180);
        temp_theta = temp_theta * M_PI / 180;
        temp_x = cos(-angle_p_bias) * temp_dist * cos(temp_theta) + sin(-angle_p_bias) * temp_dist * sin(temp_theta);
        temp_y = -sin(-angle_p_bias) * temp_dist * cos(temp_theta) + cos(-angle_p_bias) * temp_dist * sin(temp_theta);
        temp_x += ANGLE_P_X;
        temp_y += ANGLE_P_Y;
    }

    // Compute distance and angle
    float cam_dist = sqrt((temp_x * temp_x) + (temp_y * temp_y));
    float cam_angle = (temp_x != 0 ? atan(temp_y / temp_x) : 90) + spatial_param.angle_offset;
    float total_x = cam_dist * sin(cam_angle) + spatial_param.norm * sin(spatial_param.angle);
    float total_y = cam_dist * cos(cam_angle) + spatial_param.norm * cos(spatial_param.angle);

    (*distance) = sqrt((total_x * total_x) + (total_y * total_y));
    (*angle) =  cam_angle + spatial_param.angle;
    (*angle) = fmod((*angle) + (2.0 * M_PI), 2.0 * M_PI);

    if ((*angle) < 0)
        (*angle) += 2.0 * M_PI;
}

void convert_distance_iterator(py::array_t<float, py::array::c_style> &distances, py::array_t<float, py::array::c_style> &angles, py::array_t<int, py::array::c_style> &datas, SpatialParametersCpp spatial_param, DeviceParametersCpp device_param) {
    py::buffer_info buf_distances = distances.request(), buf_angles = angles.request(), buf_datas = datas.request();
    float *distances_ptr = static_cast<float *>(buf_distances.ptr);
    float *angles_ptr = static_cast<float *>(buf_angles.ptr);
    int* datas_ptr = static_cast<int *>(buf_datas.ptr);

    for (int i = 2, j = 0; i < buf_datas.shape[0]; i += 2, j++) {
        int calc_distance = ((datas_ptr[i + 1] << 8) | (datas_ptr[i] & 0xFF)) & COMPUTE_DISTANCE_MASK;

        convert_distances_cpp(distances_ptr + j, angles_ptr + j, calc_distance, j, spatial_param, device_param);
    }
}

#endif /* GS2_COMPUTATION_H */