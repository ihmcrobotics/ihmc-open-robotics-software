namespace PerceptionUtils
{
    /*
     * Get a pointer to a row of a matrix.
     * Can be used with col() to get a pointer to a specific cell.
     * E.g. cell = row(col(matrix, x), pitch, y)
     */
    template<typename T>
    __device__ T* row(const T* matrix, long pitch, int row)
    {
        return (T*)((char*) matrix + pitch * row);
    }

    /*
     * Get a pointer to a column of a matrix.
     * Can be used with row() to get a pointer to a specific cell.
     * E.g. cell = row(col(matrix, x), pitch, y)
     */
     template<typename T>
    __device__ T* col(const T* matrix, int column)
    {
        return (T*)(matrix + column);
    }

    /**
     * Returns the matrix entry at the given row and column
     */
    template <typename T>
    __device__ __forceinline__
    T get2d(const T* matrix, long pitch, int columnIndex, int rowIndex)
    {
        const T* column = col(matrix, columnIndex);
        const T* entry = row(column, pitch, rowIndex);
        return *entry;
    }

    /**
     * Sets the value of the matrix at the given row and column
     */
    template <typename T>
    __device__ __forceinline__
    void set2d(T* matrix, long pitch, int columnIndex, int rowIndex, T value)
    {
        T* column = col(matrix, columnIndex);
        T* entry = row(column, pitch, rowIndex);
        *entry = value;
    }

    __device__ float3 pixelDepthToPoint3D(int pixelX, int pixelY, float z, float fx, float fy, float cx, float cy)
    {
        float x = (pixelX - cx) / fx * z;
        float y = (pixelY - cy) / fy * z;
        return make_float3(z, -x, -y);
    }

    /**
     * Confidence weight in (0, 1] for a depth measurement at range z (metres).
     *
     * ZED X Mini depth comes from stereo triangulation, whose accuracy degrades with range
     * (roughly a quadratic relationship per Stereolabs' depth spec). We therefore model the
     * measurement variance as growing with range and weight each observation by its inverse
     * variance, normalised so that a measurement at referenceRange gets weight 1:
     *
     *     weight(z) = cameraTrust * (referenceRange / z) ^ falloffExponent
     *
     * Measurements nearer than referenceRange are capped at 1. Use falloffExponent = 4 for
     * strict inverse-variance with a quadratic accuracy falloff (sigma proportional to z^2),
     * or 2 for a gentler practical falloff. The cameraTrust scalar lets two cameras be
     * weighted differently on top of the range effect; the nearer-ranging camera
     * (e.g. belly/pelvis) is favoured automatically because it observes the ground at
     * shorter range.
     */
    __device__ __forceinline__
    float depthConfidenceWeight(float z, float referenceRange, float falloffExponent, float cameraTrust)
    {
        float ratio  = referenceRange / fmaxf(z, 1e-3f);
        float weight = cameraTrust * powf(ratio, falloffExponent);
        return fminf(fmaxf(weight, 0.0f), 1.0f);
    }
}
