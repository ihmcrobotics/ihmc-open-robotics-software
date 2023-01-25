package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;

public class RapidHeightMapExtractor
{
   private final int TOTAL_NUM_PARAMS = 5;

   private float gridLengthInMeters = 8.0f;
   private float gridWidthInMeters = 6.0f;
   private float cellSizeXYInMeters = 0.1f;

   private int gridLength = (int) (gridLengthInMeters / cellSizeXYInMeters);
   private int gridWidth = (int) (gridWidthInMeters / cellSizeXYInMeters);

   private OpenCLManager openCLManager;
   private OpenCLFloatBuffer parametersBuffer;
   private _cl_program rapidHeightMapUpdaterProgram;
   private _cl_kernel heightMapUpdateKernel;
   private BytedecoImage inputDepthImage;
   private BytedecoImage outputHeightMapImage;

   private boolean firstRun = true;
   private boolean patchSizeChanged = true;
   private boolean modified = true;
   private boolean processing = false;

   public void create(OpenCLManager openCLManager, _cl_program program, BytedecoImage depthImage, BytedecoImage outputHeightMapImage)
   {
      this.outputHeightMapImage = outputHeightMapImage;
      this.inputDepthImage = depthImage;
      this.openCLManager = openCLManager;
      this.rapidHeightMapUpdaterProgram = program;

      parametersBuffer = new OpenCLFloatBuffer(TOTAL_NUM_PARAMS);
      parametersBuffer.createOpenCLBufferObject(openCLManager);

      heightMapUpdateKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapUpdateKernel");
   }

   public void update()
   {
      if (!processing)
      {
         // Upload input depth image
         inputDepthImage.writeOpenCLImage(openCLManager);

         //// Fill parameters buffer
         //parametersBuffer.getBytedecoFloatBufferPointer().put(0, gridLengthInMeters);
         //parametersBuffer.getBytedecoFloatBufferPointer().put(1, gridWidthInMeters);
         //parametersBuffer.getBytedecoFloatBufferPointer().put(2, cellSizeXYInMeters);
         //parametersBuffer.getBytedecoFloatBufferPointer().put(3, (float) inputDepthImage.getImageHeight());
         //parametersBuffer.getBytedecoFloatBufferPointer().put(4, (float) inputDepthImage.getImageWidth());
         //parametersBuffer.writeOpenCLBufferObject(openCLManager);
         //
         //// Set kernel arguments for the height map kernel
         //openCLManager.setKernelArgument(heightMapUpdateKernel, 0, inputDepthImage.getOpenCLImageObject());
         //openCLManager.setKernelArgument(heightMapUpdateKernel, 1, parametersBuffer.getOpenCLBufferObject());
         //openCLManager.setKernelArgument(heightMapUpdateKernel, 2, outputHeightMapImage.getOpenCLImageObject());
         //
         //// Execute kernel with length and width parameters
         //openCLManager.execute2D(heightMapUpdateKernel, gridWidth, gridLength);
         //
         //// Read height map image into CPU memory
         //outputHeightMapImage.readOpenCLImage(openCLManager);
      }
   }

   public boolean isProcessing()
   {
      return processing;
   }

   public void setProcessing(boolean processing)
   {
      this.processing = processing;
   }

   public boolean isModified()
   {
      return modified;
   }

   public void setModified(boolean modified)
   {
      this.modified = modified;
   }
}
