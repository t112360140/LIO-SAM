#ifndef POINT_COLORIZER_CUH
#define POINT_COLORIZER_CUH

#include <cstdint>

// GPU 影像校正 (Remap) 核心
void launch_rectify_kernel(
    const uint8_t* d_src_image, uint8_t* d_dst_image,
    const float* d_map_x, const float* d_map_y,
    int width, int height, int channels
);

// GPU 點雲著色核心
void launch_colorization_kernel(
    const float* d_x, const float* d_y, const float* d_z,
    uint8_t* d_r, uint8_t* d_g, uint8_t* d_b, bool* d_is_colored, bool* d_is_removed,
    const float* K, const float* T,
    const uint8_t* d_image, int img_width, int img_height, int img_step, int channels, int num_points,
    float max_range
);

#endif // POINT_COLORIZER_CUH
