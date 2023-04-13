#ifndef GS2_STRUCT_H
#define GS2_STRUCT_H

struct DeviceParametersCpp {
    DeviceParametersCpp(const float k0, const float b0, const float k1, const float b1, const float bias) 
        : k0(k0), b0(b0), k1(k1), b1(b1), bias(bias) {}
    float k0;
    float b0;
    float k1;
    float b1;
    float bias;
};

struct SpatialParametersCpp {
    SpatialParametersCpp(const float angle_offset, const float x_offset, const float y_offset, 
        const float norm, const float angle) 
        : angle_offset(angle_offset), x_offset(x_offset), y_offset(y_offset), 
        norm(norm), angle(angle) {}
    float angle_offset;
    float x_offset;
    float y_offset;
    float norm;
    float angle;
};

#endif /* GS2_STRUCT_H */
