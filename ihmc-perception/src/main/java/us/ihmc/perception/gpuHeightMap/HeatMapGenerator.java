package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLManager;

public class HeatMapGenerator
{
   private final _cl_program program;
   private final OpenCLManager openCLManager;
   private final OpenCLFloatParameters parametersBuffer;
   private final _cl_kernel heatMapKernel;

   private BytedecoImage heatMapImage;

   public HeatMapGenerator()
   {
      this.openCLManager = new OpenCLManager();
      this.program = openCLManager.loadProgram("HeatMapGenerator");
      this.parametersBuffer = new OpenCLFloatParameters();
      this.heatMapKernel = openCLManager.createKernel(program, "heatMapKernel");
   }

   public Mat generateHeatMap(BytedecoImage inputValueImage)
   {
      if (heatMapImage == null)
      {
         heatMapImage = new BytedecoImage(inputValueImage.getImageWidth(), inputValueImage.getImageHeight(), opencv_core.CV_8UC3);
      }

      if (heatMapImage.getImageWidth() != inputValueImage.getImageWidth() || heatMapImage.getImageHeight() != inputValueImage.getImageHeight())
      {
         heatMapImage.destroy(openCLManager);
         heatMapImage = new BytedecoImage(inputValueImage.getImageWidth(), inputValueImage.getImageHeight(), opencv_core.CV_8UC3);
      }

      parametersBuffer.setParameter(inputValueImage.getImageWidth());
      parametersBuffer.setParameter(inputValueImage.getImageHeight());

      openCLManager.setKernelArgument(heatMapKernel, 0, inputValueImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(heatMapKernel, 1, heatMapImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(heatMapKernel, 2, parametersBuffer.getOpenCLBufferObject());

      openCLManager.execute2D(heatMapKernel, inputValueImage.getImageHeight(), inputValueImage.getImageWidth());

      heatMapImage.readOpenCLImage(openCLManager);

      return heatMapImage.getBytedecoOpenCVMat();
   }

   public void release()
   {
      openCLManager.releaseBufferObject(heatMapImage.getOpenCLImageObject());
   }
}
