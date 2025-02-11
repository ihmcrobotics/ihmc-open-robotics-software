package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.ShortPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.net.URL;
import java.util.ArrayList;
import java.util.List;

import static org.bytedeco.cuda.global.cudart.*;

public class FilteredRapidHeightMapExtractor
{
   private final List<GpuMat> gpuLayers;
   private final ShortPointer pointerTo3DArray;
   private final long pitchForLayer;
   private final GpuMat gpuMatExample;
   private int currentIndex;
   int layers = 2;

   private final CUstream_st stream;
   private final int rows;
   private final int cols;
   private final CUDAKernel kernel;
   private final CUDAProgram program;
   private int loopTracker = 0;

   public FilteredRapidHeightMapExtractor(CUstream_st stream, int rows, int cols)
   {
      this.stream = stream;
      this.rows = rows;
      this.cols = cols;

      // Load header and main file
      URL kernelPath = getClass().getResource("FilteredRapidHeightMapExtractor.cu");
      try
      {
         program = new CUDAProgram(kernelPath);
         kernel = program.loadKernel("filterRapidHeightMap");
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      gpuMatExample = new GpuMat(rows, cols, opencv_core.CV_16UC1);
      pitchForLayer = gpuMatExample.step();

      pointerTo3DArray = new ShortPointer();
      cudaMalloc(pointerTo3DArray, layers * pitchForLayer * rows);
      int error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      gpuLayers = new ArrayList<>(layers);
      currentIndex = 0;

      for (int i = 0; i < layers; i++)
      {
         // To get the average to be a whole number do (i * 2 + 2)
         Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(0));

         // Upload that data to the gpu so we can allocate the memory for it
         GpuMat gpuData = new GpuMat();
         gpuData.upload(cpuData);

         // Allocate memory for each layer, creating a 3d array
         cudaMemcpy2D(pointerTo3DArray.position(i * pitchForLayer * rows),
                      pitchForLayer,
                      gpuData.data(),
                      gpuData.step(),
                      cols * Short.BYTES,
                      rows,
                      cudaMemcpyHostToDevice);

         // Note we can't close the GpuMat here cause we need to access the data later in the program, so add it to a list, and close the list at the end
         gpuLayers.add(gpuData);
      }
   }

   public GpuMat update(GpuMat latestGlobalHeightMap)
   {
      // Only want to compute the average if we have the past values to use
      if (loopTracker < layers)
      {
         loopTracker++;
         latestGlobalHeightMap.convertTo(gpuLayers.get(currentIndex), latestGlobalHeightMap.type());
         Mat temp = new Mat();
         gpuLayers.get(currentIndex).download(temp);
         PerceptionDebugTools.printMat("s", temp, 1);
         currentIndex = (currentIndex + 1) % layers;
         return latestGlobalHeightMap;
      }

      int error;

      GpuMat result = new GpuMat(rows, cols, opencv_core.CV_16UC1);

      kernel.withPointer(pointerTo3DArray).withLong(gpuMatExample.step());
      kernel.withPointer(result.data()).withLong(result.step());
      kernel.withLong(pitchForLayer * rows);
      kernel.withInt(rows);
      kernel.withInt(cols);
      kernel.withInt(layers);

      dim3 gridDim = new dim3();
      dim3 blockDim = new dim3(cols, rows, 1);
      kernel.run(stream, gridDim, blockDim, 0);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // Print the result to make sure we get what we expect
      // In this example we go through 2,4,6,8 so we expect the average to be 5
      Mat cpuResult = new Mat();
      result.download(cpuResult);
      PerceptionDebugTools.printMat("Result", cpuResult, 1);

      gpuLayers.get(currentIndex).setTo(new Scalar(1), latestGlobalHeightMap);
      currentIndex = (currentIndex + 1) % layers;

      return result;
   }

   public void reset()
   {
      loopTracker = 0;
   }

   public void close()
   {
      program.close();
      kernel.close();
   }
}
