extern "C" __global__
void pointToPlaneICP(
    const unsigned short* srcDepth,
    size_t srcPitch,
    const unsigned short* tgtDepth,
    size_t tgtPitch,
    const float3* tgtNormals,
    int width,
    int height,
    float fx, float fy, float cx, float cy,
    float maxDepthDiff,
    float* ATA,   // 6x6 row-major
    float* ATb    // 6x1
)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    // Read depths
    unsigned short d_src =
        *row(col(srcDepth, x), srcPitch, y);
    unsigned short d_tgt =
        *row(col(tgtDepth, x), tgtPitch, y);

    if (d_src == 0 || d_tgt == 0) return;

    float z_src = d_src * 0.001f;
    float z_tgt = d_tgt * 0.001f;

    if (fabsf(z_src - z_tgt) > maxDepthDiff) return;

    // Back-project
    float3 p;
    p.x = (x - cx) * z_src / fx;
    p.y = (y - cy) * z_src / fy;
    p.z = z_src;

    float3 q;
    q.x = (x - cx) * z_tgt / fx;
    q.y = (y - cy) * z_tgt / fy;
    q.z = z_tgt;

    float3 n = tgtNormals[y * width + x];
    if (!isfinite(n.x)) return;

    // Residual
    float3 diff = make_float3(
        p.x - q.x,
        p.y - q.y,
        p.z - q.z
    );
    float r = dot(n, diff);

    // Jacobian
    float3 pxn = cross(p, n);

    float J[6] = {
        pxn.x, pxn.y, pxn.z,
        n.x,   n.y,   n.z
    };

    // Accumulate ATA and ATb
    for (int i = 0; i < 6; ++i)
    {
        atomicAdd(&ATb[i], J[i] * r);
        for (int j = 0; j < 6; ++j)
        {
            atomicAdd(&ATA[i * 6 + j], J[i] * J[j]);
        }
    }
}
