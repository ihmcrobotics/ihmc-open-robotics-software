package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

public class CUDADepthColorizer
{
   private static final int BLOCK_DIM_XY = 16;

   private final CUstream_st stream;
   private final CUDAProgram program;
   private final CUDAKernel encoder;
   private final CUDAKernel decoder;

   public CUDADepthColorizer()
   {
      stream = CUDAStreamManager.getStream();

      URL programURL = CUDADepthColorizer.class.getResource("DepthColorization.cu");
      URL utilsURL = CUDATools.getUtilsFile();
      program = new CUDAProgram(programURL, utilsURL);

      encoder = program.loadKernel("colorizeDepth");
      decoder = program.loadKernel("deColorizeDepth");
   }

   public GpuMat colorizeDepth(GpuMat depthImage)
   {
      GpuMat colorizedDepth = new GpuMat(depthImage.size(), opencv_core.CV_8UC3);

      int gridSizeX = (depthImage.cols() / BLOCK_DIM_XY) / 4;
      int gridSizeY = (depthImage.rows() / BLOCK_DIM_XY) / 4;

      try (dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);
           dim3 blockSize = new dim3(BLOCK_DIM_XY, BLOCK_DIM_XY, 1))
      {
         encoder.clearParameters();
         encoder.withPointer(depthImage.data()).withLong(depthImage.step())
                .withPointer(colorizedDepth.data()).withLong(colorizedDepth.step())
                .withInt(depthImage.rows()).withInt(depthImage.cols())
                .run(stream, gridSize, blockSize, 0);
      }

      int error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      return colorizedDepth;
   }

   public GpuMat deColorizeDepth(GpuMat colorizedDepthImage)
   {
      GpuMat deColorizedDepth = new GpuMat(colorizedDepthImage.size(), opencv_core.CV_16UC1);

      int gridSizeX = (colorizedDepthImage.cols() / BLOCK_DIM_XY) / 4;
      int gridSizeY = (colorizedDepthImage.rows() / BLOCK_DIM_XY) / 4;

      try (dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);
           dim3 blockSize = new dim3(BLOCK_DIM_XY, BLOCK_DIM_XY, 1))
      {
         decoder.clearParameters();
         decoder.withPointer(colorizedDepthImage.data()).withLong(colorizedDepthImage.step())
                .withPointer(deColorizedDepth.data()).withLong(deColorizedDepth.step())
                .withInt(colorizedDepthImage.rows()).withInt(colorizedDepthImage.cols())
                .run(stream, gridSize, blockSize, 0);
      }

      int error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      return deColorizedDepth;
   }

   public void destroy()
   {
      program.close();
      encoder.close();
      decoder.close();
   }
}
