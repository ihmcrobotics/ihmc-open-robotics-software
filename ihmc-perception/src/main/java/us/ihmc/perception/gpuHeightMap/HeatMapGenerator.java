package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionDebugTools;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKey;

/**
 * OpenCL based heat map generator for 16-bit grayscale images. Visualize any 16-bit grayscale image (scalars like steppability, traversability for each cell)
 * as a colored image with any desired color scheme.
 */
public class HeatMapGenerator
{
   private final _cl_program program;
   private final OpenCLManager openCLManager;
   private final OpenCLFloatParameters parametersBuffer;
   private final _cl_kernel heatMapKernel;

   private BytedecoImage inputImage;
   private BytedecoImage heatMapImage;

   public HeatMapGenerator()
   {
      this.openCLManager = new OpenCLManager();
      this.program = openCLManager.loadProgram("HeatMapGenerator");
      this.parametersBuffer = new OpenCLFloatParameters();
      this.heatMapKernel = openCLManager.createKernel(program, "heatMapKernel");
   }

   public Mat generateHeatMap(Mat inputValueImage)
   {
      if (heatMapImage == null)
      {
         heatMapImage = new BytedecoImage(inputValueImage.cols(), inputValueImage.rows(), opencv_core.CV_8UC4);
         heatMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

         inputImage = new BytedecoImage(inputValueImage.cols(), inputValueImage.rows(), opencv_core.CV_8UC1);
         inputImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      }

      inputValueImage.copyTo(inputImage.getBytedecoOpenCVMat());
      inputImage.writeOpenCLImage(openCLManager);

      parametersBuffer.setParameter(inputValueImage.cols());
      parametersBuffer.setParameter(inputValueImage.rows());
      parametersBuffer.writeOpenCLBufferObject(openCLManager);

      openCLManager.setKernelArgument(heatMapKernel, 0, inputImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(heatMapKernel, 1, heatMapImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(heatMapKernel, 2, parametersBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(heatMapKernel, inputValueImage.rows(), inputValueImage.cols());

      heatMapImage.readOpenCLImage(openCLManager);
      return heatMapImage.getBytedecoOpenCVMat();
   }

   public void release()
   {
      openCLManager.releaseBufferObject(heatMapImage.getOpenCLImageObject());
   }
}
