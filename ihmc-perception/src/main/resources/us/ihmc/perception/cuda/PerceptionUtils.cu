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
}