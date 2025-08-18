package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUevent_st;
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
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import static org.bytedeco.cuda.global.cudart.*;
import static us.ihmc.perception.cuda.CUDATools.throwCUDAError;

@SuppressWarnings("resource")
public class CUDAKernel implements AutoCloseable
{
   private final String name;
   private final CUfunc_st kernelFunction = new CUfunc_st();
   private final List<Pointer> parameters = new ArrayList<>();
   private boolean retainParameters = false;
   private boolean enableKernelTimings = false;

   private CUDAKernelTimings kernelTimings = null;
   private final CUevent_st start = new CUevent_st();
   private final CUevent_st end = new CUevent_st();

   private int error;

   public CUDAKernel(String name, CUmod_st kernelModule) throws Exception
   {
      this.name = name;
      error = cuModuleGetFunction(kernelFunction, kernelModule, name);
      throwCUDAError(error);
   }

   /**
    * Setting this to true enables the ability to run timings on the specific kernel.
    * The timing checks perform synchronization calls.
    */
   public void enableKernelTimings(boolean enableKernelTimings)
   {
      this.enableKernelTimings = enableKernelTimings;
      if (enableKernelTimings && kernelTimings == null)
      {
         kernelTimings = new CUDAKernelTimings();
         cudaEventCreate(start);
         cudaEventCreate(end);
      }
   }

   public void retainParameters(boolean retainParameters)
   {
      this.retainParameters = retainParameters;
   }

   /**
    * Launches the CUDA kernel. The parameters used for this launch are cleared unless {@code retainParameters == true}.
    *
    * @param stream           CUDA stream on which the kernel will be synchronized.
    * @param gridSize         Grid size of the kernel execution.
    * @param blockSize        Block size of the kernel execution. Should not exceed maximum block size of the device.
    * @param sharedMemorySize Size, in byte, of the memory shared by threads in each block.
    */
   public void run(CUstream_st stream, dim3 gridSize, dim3 blockSize, int sharedMemorySize)
   {
      PointerPointer<Pointer> parametersPointer = new PointerPointer<>(parameters.size());
      for (int i = 0; i < parameters.size(); ++i)
         parametersPointer.put(i, parameters.get(i));

      if (enableKernelTimings)
         cudaEventRecord(start, stream);

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

      if (enableKernelTimings)
      {
         cudaEventRecord(end, stream);
         cudaEventSynchronize(end);

         kernelTimings.addExecutionTime(start, end);
         kernelTimings.printTimesForKernel();
      }

      CUDATools.checkCUDAError(error);

      if (!retainParameters)
         clearParameters();
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

      if (kernelTimings != null)
      {
         cudaEventDestroy(start);
         cudaEventDestroy(end);
      }
   }

   /**
    * This class handles the kernel timings.
    * With options to compute the min/max, average, and variance of the dataset
    */
   private class CUDAKernelTimings
   {
      private static final int MAX_ENTRIES = 1000;
      private final LinkedList<Float> executionTimes = new LinkedList<>();
      private final float[] milliseconds = new float[1];

      private void addExecutionTime(CUevent_st start, CUevent_st end)
      {
         milliseconds[0] = 0.0f;
         cudaEventElapsedTime(milliseconds, start, end);
         executionTimes.add(milliseconds[0]);

         if (executionTimes.size() > MAX_ENTRIES)
         {
            executionTimes.pollFirst();
         }
      }

      public double getAverageTime(String kernelName)
      {
         if (executionTimes.isEmpty())
         {
            LogTools.info("No recorded times for " + kernelName);
            return Float.NaN;
         }
         else
         {
            return executionTimes.stream().mapToDouble(Float::doubleValue).average().orElse(0.0);
         }
      }

      public Float getMinTime(String kernelName)
      {
         if (executionTimes.isEmpty())
         {
            LogTools.info("No recorded times for " + kernelName);
            return Float.NaN;
         }
         Optional<Float> min = executionTimes.stream().min(Float::compareTo);
         return min.orElse(null);
      }

      public  Float getMaxTime(String kernelName)
      {
         if (executionTimes.isEmpty())
         {
            LogTools.info("No recorded times for " + kernelName);
            return Float.NaN;
         }

         Optional<Float> max = executionTimes.stream().max(Float::compareTo);
         return max.orElse(null);
      }

      public double getStandardDeviation(String kernelName)
      {
         if (executionTimes.isEmpty())
         {
            LogTools.info("No recorded times for " + kernelName);
            return Float.NaN;
         }

         double average = executionTimes.stream().mapToDouble(Float::doubleValue).average().orElse(0.0);
         double variance = executionTimes.stream().mapToDouble(time -> Math.pow(time - average, 2)).average().orElse(0.0);
         return Math.sqrt(variance);
      }

      public void printTimesForKernel()
      {
         if (executionTimes.isEmpty())
         {
            LogTools.info("No recorded times for " + CUDAKernel.this.name);
         }

         double average = getAverageTime(CUDAKernel.this.name);
         double variance = getStandardDeviation(CUDAKernel.this.name);
         double min = getMinTime(CUDAKernel.this.name);
         double max = getMaxTime(CUDAKernel.this.name);

         LogTools.info("Timings for kernel " + CUDAKernel.this.name + " in milliseconds!");
         LogTools.info("|   Average time: " + average);
         LogTools.info("|   Variance time: " + variance);
         LogTools.info("|   Min time: " + min);
         LogTools.info("|   Max time: " + max);
         LogTools.warn("******************************************");
      }
   }
}
