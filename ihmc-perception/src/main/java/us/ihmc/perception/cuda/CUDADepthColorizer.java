package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.perception.opencv.OpenCVTools;

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

   public void colorizeDepth(GpuMat depthImage, GpuMat colorizedDepth)
   {
      ensureCorrectAllocation(colorizedDepth, depthImage.rows(), depthImage.cols(), opencv_core.CV_8UC3);

      int gridSizeX = (depthImage.cols() / BLOCK_DIM_XY) / 4;
      int gridSizeY = (depthImage.rows() / BLOCK_DIM_XY) / 4;

      try (dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);
           dim3 blockSize = new dim3(BLOCK_DIM_XY, BLOCK_DIM_XY, 1))
      {
         encoder.withPointer(depthImage.data()).withLong(depthImage.step())
                .withPointer(colorizedDepth.data()).withLong(colorizedDepth.step())
                .withInt(depthImage.rows()).withInt(depthImage.cols())
                .run(stream, gridSize, blockSize, 0);
      }

      int error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);
   }

   public void deColorizeDepth(GpuMat colorizedDepthImage, GpuMat deColorizedDepth)
   {
      ensureCorrectAllocation(deColorizedDepth, colorizedDepthImage.rows(), colorizedDepthImage.cols(), opencv_core.CV_16UC1);

      int gridSizeX = (colorizedDepthImage.cols() / BLOCK_DIM_XY) / 4;
      int gridSizeY = (colorizedDepthImage.rows() / BLOCK_DIM_XY) / 4;

      try (dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);
           dim3 blockSize = new dim3(BLOCK_DIM_XY, BLOCK_DIM_XY, 1))
      {
         decoder.withPointer(colorizedDepthImage.data()).withLong(colorizedDepthImage.step())
                .withPointer(deColorizedDepth.data()).withLong(deColorizedDepth.step())
                .withInt(colorizedDepthImage.rows()).withInt(colorizedDepthImage.cols())
                .run(stream, gridSize, blockSize, 0);
      }

      int error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);
   }

   public void destroy()
   {
      program.close();
      encoder.close();
      decoder.close();
   }

   private void ensureCorrectAllocation(GpuMat mat, int correctRows, int correctCols, int correctType)
   {
      // If the GpuMat is already the correct type and size, just return
      if (mat.type() == correctType && mat.rows() == correctRows && mat.cols() == correctCols)
         return;

      // Try to create the GpuMat and return if it worked
      mat.create(correctRows, correctCols, correctType);
      if (mat.type() == correctType && mat.rows() == correctRows && mat.cols() == correctCols)
         return;

      // Bad GpuMat was given. Throw exception
      throw new IllegalArgumentException(
            "The provided GpuMat should be unallocated, or allocated using the correct size (%d rows by %d cols) and type (%d)"
                  .formatted(correctRows, correctCols, correctType));
   }
}
