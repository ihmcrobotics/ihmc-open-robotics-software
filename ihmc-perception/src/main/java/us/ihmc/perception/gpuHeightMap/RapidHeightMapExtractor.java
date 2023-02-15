package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.tools.PerceptionEuclidTools;

public class RapidHeightMapExtractor
{
   private final int TOTAL_NUM_PARAMS = 7;

   private float gridLengthInMeters = 8.0f;
   private float gridWidthInMeters = 6.0f;
   private float cellSizeXYInMeters = 0.02f;

   private int gridLength = (int) (gridLengthInMeters / cellSizeXYInMeters);
   private int gridWidth = (int) (gridWidthInMeters / cellSizeXYInMeters);

   private OpenCLManager openCLManager;
   private OpenCLFloatBuffer parametersBuffer;

   private OpenCLFloatBuffer worldToSensorTransformBuffer;
   private OpenCLFloatBuffer sensorToWorldTransformBuffer;

   private OpenCLFloatBuffer groundPlaneBuffer;
   private _cl_program rapidHeightMapUpdaterProgram;
   private _cl_kernel heightMapUpdateKernel;
   private BytedecoImage inputDepthImage;
   private BytedecoImage outputHeightMapImage;

   private boolean firstRun = true;
   private boolean patchSizeChanged = true;
   private boolean modified = true;
   private boolean processing = false;

   public void create(OpenCLManager openCLManager, _cl_program program, BytedecoImage depthImage)
   {
      this.inputDepthImage = depthImage;
      this.openCLManager = openCLManager;
      this.rapidHeightMapUpdaterProgram = program;

      parametersBuffer = new OpenCLFloatBuffer(TOTAL_NUM_PARAMS);
      parametersBuffer.createOpenCLBufferObject(openCLManager);

      worldToSensorTransformBuffer = new OpenCLFloatBuffer(16);
      worldToSensorTransformBuffer.createOpenCLBufferObject(openCLManager);

      sensorToWorldTransformBuffer = new OpenCLFloatBuffer(16);
      sensorToWorldTransformBuffer.createOpenCLBufferObject(openCLManager);

      groundPlaneBuffer = new OpenCLFloatBuffer(4);
      groundPlaneBuffer.createOpenCLBufferObject(openCLManager);

      outputHeightMapImage = new BytedecoImage(gridWidth, gridLength, opencv_core.CV_16UC1);
      outputHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      heightMapUpdateKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapUpdateKernel");
   }

   public void update(RigidBodyTransform sensorToWorldTransform, float planeHeight)
   {
      if (!processing)
      {
         // Upload input depth image
         inputDepthImage.writeOpenCLImage(openCLManager);

         //// Fill parameters buffer
         parametersBuffer.getBytedecoFloatBufferPointer().put(0, gridLengthInMeters);
         parametersBuffer.getBytedecoFloatBufferPointer().put(1, gridWidthInMeters);
         parametersBuffer.getBytedecoFloatBufferPointer().put(2, cellSizeXYInMeters);
         parametersBuffer.getBytedecoFloatBufferPointer().put(3, (float) inputDepthImage.getImageHeight());
         parametersBuffer.getBytedecoFloatBufferPointer().put(4, (float) inputDepthImage.getImageWidth());
         parametersBuffer.getBytedecoFloatBufferPointer().put(5, (float) gridLength);
         parametersBuffer.getBytedecoFloatBufferPointer().put(6, (float) gridWidth);
         parametersBuffer.writeOpenCLBufferObject(openCLManager);


         // Fill ground plane buffer
         RigidBodyTransform worldToSensorTransform = new RigidBodyTransform(sensorToWorldTransform);
         worldToSensorTransform.invert();

         // Fill world-to-sensor transform buffer
         worldToSensorTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(PerceptionEuclidTools.toFloatArray(worldToSensorTransform));
         worldToSensorTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Fill sensor-to-world transform buffer
         sensorToWorldTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(PerceptionEuclidTools.toFloatArray(sensorToWorldTransform));
         sensorToWorldTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Generate a +Z vector in world frame
         Vector3D groundNormalSensorFrame = new Vector3D(0.0, 0.0, 1.0);
         worldToSensorTransform.transform(groundNormalSensorFrame);

         LogTools.info("Ground normal in sensor frame: " + groundNormalSensorFrame);

         groundPlaneBuffer.getBytedecoFloatBufferPointer().asBuffer().put(new float[] {groundNormalSensorFrame.getX32(), groundNormalSensorFrame.getY32(),
                                                                                       groundNormalSensorFrame.getZ32(),
                                                                                       (float) (planeHeight - sensorToWorldTransform.getTranslationZ())});
         groundPlaneBuffer.writeOpenCLBufferObject(openCLManager);

         // Set kernel arguments for the height map kernel
         openCLManager.setKernelArgument(heightMapUpdateKernel, 0, inputDepthImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 1, outputHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 2, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 3, sensorToWorldTransformBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 4, worldToSensorTransformBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 5, groundPlaneBuffer.getOpenCLBufferObject());

         // Execute kernel with length and width parameters
         openCLManager.execute2D(heightMapUpdateKernel, gridWidth, gridLength);

         // Read height map image into CPU memory
         outputHeightMapImage.readOpenCLImage(openCLManager);
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

   public float getCellSizeXYInMeters()
   {
      return cellSizeXYInMeters;
   }

   public BytedecoImage getOutputHeightMapImage()
   {
      return outputHeightMapImage;
   }

   public float getGridLengthInMeters()
   {
      return gridLengthInMeters;
   }

   public float getGridWidthInMeters()
   {
      return gridWidthInMeters;
   }

   public int getGridLength()
   {
      return gridLength;
   }

   public int getGridWidth()
   {
      return gridWidth;
   }
}
