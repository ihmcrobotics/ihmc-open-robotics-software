package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.dim3;

/**
 * This class contains the information that the {@link CUDAProgram} needs to successfully run the loaded CUDA kernel. This consists of the name of the kernel,
 * as well as the grid size and block size. These are then retrieved by the program when running using {@link CUDAKernelHandle#getName()},
 * {@link CUDAKernelHandle#getBlockSize()}, and {@link CUDAKernelHandle#getGridSize()} .
 *
 * <p>
 * By default, the grid size and block size are loaded as zero dimensional {@link dim3} objects. This can be overriden (or set) by calling
 * {@link CUDAKernelHandle#setBlockSize(dim3)} and {@link CUDAKernelHandle#setGridSize(dim3)}.
 * </p>
 */
public class CUDAKernelHandle
{
   private final String name;

   // TODO figure out how to load these from the inputs when the kernel is loaded.
   private dim3 gridSize = new dim3();
   private dim3 blockSize = new dim3();

   public CUDAKernelHandle(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   public void setGridSize(dim3 gridSize)
   {
      this.gridSize = gridSize;
   }

   public void setBlockSize(dim3 blockSize)
   {
      this.blockSize = blockSize;
   }

   public dim3 getGridSize()
   {
      return gridSize;
   }

   public dim3 getBlockSize()
   {
      return blockSize;
   }
}
