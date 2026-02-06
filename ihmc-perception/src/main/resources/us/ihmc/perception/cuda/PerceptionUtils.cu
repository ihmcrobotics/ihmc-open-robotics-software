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
}
