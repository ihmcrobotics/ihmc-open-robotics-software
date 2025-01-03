package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUfunc_st;
import org.bytedeco.cuda.cudart.CUmod_st;
import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.LongPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;

import java.util.ArrayList;
import java.util.List;

import static org.bytedeco.cuda.global.cudart.cuLaunchKernel;
import static org.bytedeco.cuda.global.cudart.cuModuleGetFunction;
import static us.ihmc.perception.cuda.CUDATools.checkCUDAError;

@SuppressWarnings("resource")
public class CUDAKernel implements AutoCloseable
{
   private final CUfunc_st kernelFunction = new CUfunc_st();
   private final List<Pointer> parameters = new ArrayList<>();

   private int error;

   public CUDAKernel(String name, CUmod_st kernelModule)
   {
      error = cuModuleGetFunction(kernelFunction, kernelModule, name);
      checkCUDAError(error);
   }

   /**
    * Launches the CUDA kernel.
    *
    * @param stream CUDA stream on which the kernel will be synchronized.
    * @param gridSize Grid size of the kernel execution.
    * @param blockSize Block size of the kernel execution. Should not exceed maximum block size of the device.
    * @param sharedMemorySize Size, in byte, of the memory shared by threads in each block.
    */
   public void run(CUstream_st stream, dim3 gridSize, dim3 blockSize, int sharedMemorySize)
   {
      PointerPointer<Pointer> parametersPointer = new PointerPointer<>(parameters.size());
      for (int i = 0; i < parameters.size(); ++i)
         parametersPointer.put(i, parameters.get(i));

      error = cuLaunchKernel(kernelFunction,
                             gridSize.x(),
                             gridSize.y(),
                             gridSize.z(),
                             blockSize.x(),
                             blockSize.y(),
                             blockSize.z(),
                             sharedMemorySize,
                             stream,
                             parametersPointer,
                             new PointerPointer<>());
      CUDATools.checkCUDAError(error);
      parametersPointer.close();
   }

   public CUDAKernel withByte(byte value)
   {
      parameters.add(new BytePointer(1L).put(value));
      return this;
   }

   public CUDAKernel withInt(int value)
   {
      parameters.add(new IntPointer(1L).put(value));
      return this;
   }

   public CUDAKernel withLong(long value)
   {
      parameters.add(new LongPointer(1L).put(value));
      return this;
   }

   public CUDAKernel withFloat(float value)
   {
      parameters.add(new FloatPointer(1L).put(value));
      return this;
   }

   public CUDAKernel withDouble(double value)
   {
      parameters.add(new DoublePointer(1L).put(value));
      return this;
   }

   public CUDAKernel withPointer(Pointer pointer)
   {
      parameters.add(new PointerPointer<>(1L).put(pointer));
      return this;
   }

   /**
    * Clears the set parameters
    */
   public void clearParameters()
   {
      for (Pointer parameter : parameters)
         parameter.close();
      parameters.clear();
   }

   @Override
   public void close()
   {
      clearParameters();
      kernelFunction.close();
   }
}
