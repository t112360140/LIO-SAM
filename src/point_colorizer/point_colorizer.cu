#include "point_colorizer.cuh"
#include <cuda_runtime.h>
#include <iostream>

// ==========================================
// 1. GPU 影像校正核心 (雙線性插值 Bilinear Interpolation)
// ==========================================
__global__ void rectify_kernel(
    const uint8_t* src, uint8_t* dst,
    const float* map_x, const float* map_y,
    int width, int height, int channels)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= width || y >= height) return;

    int dst_idx = (y * width + x) * channels;

    float src_x = map_x[y * width + x];
    float src_y = map_y[y * width + x];

    if (src_x < 0 || src_x >= width - 1 || src_y < 0 || src_y >= height - 1) {
        for (int c = 0; c < channels; ++c) dst[dst_idx + c] = 0; // 全黑且透明
        return;
    }

    int x0 = (int)src_x; int y0 = (int)src_y;
    int x1 = x0 + 1;     int y1 = y0 + 1;

    float dx = src_x - (float)x0; float dy = src_y - (float)y0;
    float w00 = (1.0f - dx) * (1.0f - dy); float w10 = dx * (1.0f - dy);
    float w01 = (1.0f - dx) * dy;          float w11 = dx * dy;

    for (int c = 0; c < channels; ++c) {
        float val = w00 * src[(y0 * width + x0) * channels + c] +
                    w10 * src[(y0 * width + x1) * channels + c] +
                    w01 * src[(y1 * width + x0) * channels + c] +
                    w11 * src[(y1 * width + x1) * channels + c];
        dst[dst_idx + c] = (uint8_t)val;
    }
}

void launch_rectify_kernel(
    const uint8_t* d_src_image, uint8_t* d_dst_image,
    const float* d_map_x, const float* d_map_y,
    int width, int height, int channels)
{
    dim3 threads(16, 16);
    dim3 blocks((width + threads.x - 1) / threads.x, (height + threads.y - 1) / threads.y);
    rectify_kernel<<<blocks, threads>>>(d_src_image, d_dst_image, d_map_x, d_map_y, width, height, channels);
    cudaDeviceSynchronize(); // 確保影像校正完成
}


// ==========================================
// 2. GPU 點雲著色核心 (保持不變)
// ==========================================
__global__ void colorize_kernel(
    const float* x, const float* y, const float* z,
    uint8_t* r, uint8_t* g, uint8_t* b, bool* is_colored, bool* is_removed,
    float K0, float K2, float K4, float K5, 
    float T0, float T1, float T2, float T3, 
    float T4, float T5, float T6, float T7,
    float T8, float T9, float T10, float T11,
    const uint8_t* image, int width, int height, int step, int channels, int num_points, 
    float max_range_sq)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;
    if (is_colored[idx]) return;
    if (is_removed[idx]) return;

    float pt_x = x[idx], pt_y = y[idx], pt_z = z[idx];
    
    if ((pt_x * pt_x + pt_y * pt_y + pt_z * pt_z) > max_range_sq) return;
    
    float cam_x = T0 * pt_x + T1 * pt_y + T2  * pt_z + T3;
    float cam_y = T4 * pt_x + T5 * pt_y + T6  * pt_z + T7;
    float cam_z = T8 * pt_x + T9 * pt_y + T10 * pt_z + T11;

    if (cam_z <= 0.0f) return;

    int u = __float2int_rn((K0 * cam_x + K2 * cam_z) / cam_z);
    int v = __float2int_rn((K4 * cam_y + K5 * cam_z) / cam_z);

    if (u >= 0 && u < width && v >= 0 && v < height) {
        int img_idx = v * step + u * channels;
        if (channels == 4 && image[img_idx + 3] < 128) {
            is_removed[idx] = true;
            return;
        }
        b[idx] = image[img_idx];    
        g[idx] = image[img_idx + 1];
        r[idx] = image[img_idx + 2];
        is_colored[idx] = true;
    }
}

void launch_colorization_kernel(
    const float* d_x, const float* d_y, const float* d_z,
    uint8_t* d_r, uint8_t* d_g, uint8_t* d_b, bool* d_is_colored, bool* d_is_removed,
    const float* K, const float* T,
    const uint8_t* d_image, int img_width, int img_height, int img_step, int channels, int num_points,
    float max_range)
{
    int threads_per_block = 256;
    int blocks_per_grid = (num_points + threads_per_block - 1) / threads_per_block;
    float max_range_sq = max_range * max_range;

    colorize_kernel<<<blocks_per_grid, threads_per_block>>>(
        d_x, d_y, d_z, d_r, d_g, d_b, d_is_colored, d_is_removed,
        K[0], K[2], K[4], K[5],       
        T[0], T[1], T[2], T[3],       
        T[4], T[5], T[6], T[7],       
        T[8], T[9], T[10], T[11],     
        d_image, img_width, img_height, img_step, channels, num_points, 
        max_range_sq
    );
    cudaDeviceSynchronize();
}
