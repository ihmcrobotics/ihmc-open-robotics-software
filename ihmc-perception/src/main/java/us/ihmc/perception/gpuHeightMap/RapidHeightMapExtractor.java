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
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

public class RapidHeightMapExtractor
{
   private static final float gridWidthInMeters = 8.0f;
   private static final float cellSizeXYInMeters = 0.02f;

   private int centerIndex;
   private int cellsPerAxis;

   private OpenCLManager openCLManager;
   private OpenCLFloatParameters parametersBuffer;

   private OpenCLFloatBuffer worldToSensorTransformBuffer;
   private float[] worldToSensorTransformArray = new float[16];

   private OpenCLFloatBuffer sensorToWorldTransformBuffer;
   private float[] sensorToWorldTransformArray = new float[16];

   private OpenCLFloatBuffer groundPlaneBuffer;
   private _cl_program rapidHeightMapUpdaterProgram;
   private _cl_kernel heightMapUpdateKernel;
   private BytedecoImage inputDepthImage;
   private BytedecoImage outputHeightMapImage;

   private boolean firstRun = true;
   private boolean patchSizeChanged = true;
   private boolean modified = true;
   private boolean processing = false;

   public void create(OpenCLManager openCLManager,  BytedecoImage depthImage)
   {
      this.inputDepthImage = depthImage;
      this.openCLManager = openCLManager;
      rapidHeightMapUpdaterProgram = openCLManager.loadProgram("RapidHeightMapExtractor", "HeightMapUtils.cl");

      centerIndex = HeightMapTools.computeCenterIndex(gridWidthInMeters, cellSizeXYInMeters);
      cellsPerAxis = 2 * centerIndex + 1;

      parametersBuffer = new OpenCLFloatParameters();

      worldToSensorTransformBuffer = new OpenCLFloatBuffer(16);
      worldToSensorTransformBuffer.createOpenCLBufferObject(openCLManager);

      sensorToWorldTransformBuffer = new OpenCLFloatBuffer(16);
      sensorToWorldTransformBuffer.createOpenCLBufferObject(openCLManager);

      groundPlaneBuffer = new OpenCLFloatBuffer(4);
      groundPlaneBuffer.createOpenCLBufferObject(openCLManager);

      outputHeightMapImage = new BytedecoImage(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);
      outputHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      heightMapUpdateKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapUpdateKernel");
   }

   private void populateParameterBuffer()
   {
      //// Fill parameters buffer
      parametersBuffer.setParameter(cellSizeXYInMeters);
      parametersBuffer.setParameter(centerIndex);
      parametersBuffer.setParameter((float) inputDepthImage.getImageHeight());
      parametersBuffer.setParameter((float) inputDepthImage.getImageWidth());

      parametersBuffer.writeOpenCLBufferObject(openCLManager);
   }


   public void update(RigidBodyTransform sensorToWorldTransform, float planeHeight)
   {
      if (!processing)
      {
         // Upload input depth image
         inputDepthImage.writeOpenCLImage(openCLManager);

         populateParameterBuffer();

         // Fill ground plane buffer
         RigidBodyTransform worldToSensorTransform = new RigidBodyTransform(sensorToWorldTransform);
         worldToSensorTransform.invert();

         // Fill world-to-sensor transform buffer
         worldToSensorTransform.get(worldToSensorTransformArray);
         worldToSensorTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(worldToSensorTransformArray);
         worldToSensorTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Fill sensor-to-world transform buffer
         sensorToWorldTransform.get(sensorToWorldTransformArray);
         sensorToWorldTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(sensorToWorldTransformArray);
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
         openCLManager.execute2D(heightMapUpdateKernel, cellsPerAxis, cellsPerAxis);

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

   public int getCellsPerAxis()
   {
      return cellsPerAxis;
   }
}
