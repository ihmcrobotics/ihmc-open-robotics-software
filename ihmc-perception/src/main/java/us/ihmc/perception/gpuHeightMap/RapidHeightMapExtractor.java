package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

public class RapidHeightMapExtractor
{
   public int sequenceNumber = 0;

   public static float LOCAL_WIDTH_IN_METERS = 3.0f; // localWidthInMeters
   public static float LOCAL_CELL_SIZE_IN_METERS = 0.02f; // localCellSizeInMeters

   public static float GLOBAL_WIDTH_IN_METERS = 6.0f; // globalWidthInMeters
   public static float GLOBAL_CELL_SIZE_IN_METERS = 0.02f; // globalCellSizeInMeters

   private static int centerIndex;
   private static int localCellsPerAxis;

   private static int globalCenterIndex;
   private static int globalCellsPerAxis;
   private static float heightScalingFactor = 10000.0f;

   private static int searchWindowHeight = 250;
   private static int searchWindowWidth = 140;

   private static float minHeightRegistration = -0.1f;
   private static float maxHeightRegistration = 0.7f;
   private static float minHeightDifference = -0.05f;
   private static float maxHeightDifference = 0.1f;

   private static float robotCollisionCylinderRadius = 0.5f;
   private static float gridOffsetX = LOCAL_WIDTH_IN_METERS / 2.0f;
   private static float heightFilterAlpha = 0.65f;

   private static int mode = 0; // 0 -> Ouster, 1 -> Realsense

   private OpenCLManager openCLManager;
   private OpenCLFloatParameters parametersBuffer;

   private RigidBodyTransform currentSensorToWorldTransform = new RigidBodyTransform();

   private OpenCLFloatBuffer worldToGroundTransformBuffer;
   private float[] groundToWorldTransformArray = new float[16];

   private OpenCLFloatBuffer groundToSensorTransformBuffer;
   private float[] groundToSensorTransformArray = new float[16];

   private OpenCLFloatBuffer sensorToGroundTransformBuffer;
   private float[] sensorToGroundTransformArray = new float[16];

   private final Point3D gridCenter = new Point3D();

   private OpenCLFloatBuffer groundPlaneBuffer;
   private _cl_program rapidHeightMapUpdaterProgram;
   private _cl_kernel heightMapUpdateKernel;
   private _cl_kernel heightMapRegistrationKernel;
   private BytedecoImage inputDepthImage;
   private BytedecoImage localHeightMapImage;
   private BytedecoImage globalHeightMapImage;

   private CameraIntrinsics cameraIntrinsics;

   private boolean firstRun = true;
   private boolean patchSizeChanged = true;
   private boolean modified = true;
   private boolean processing = false;
   private boolean heightMapDataAvailable = false;

   public void create(OpenCLManager openCLManager,  BytedecoImage depthImage, int mode)
   {
      this.mode = mode;
      this.inputDepthImage = depthImage;
      this.openCLManager = openCLManager;
      rapidHeightMapUpdaterProgram = openCLManager.loadProgram("RapidHeightMapExtractor", "HeightMapUtils.cl");

      centerIndex = HeightMapTools.computeCenterIndex(LOCAL_WIDTH_IN_METERS, LOCAL_CELL_SIZE_IN_METERS);
      localCellsPerAxis = 2 * centerIndex + 1;

      globalCenterIndex = HeightMapTools.computeCenterIndex(GLOBAL_WIDTH_IN_METERS, GLOBAL_CELL_SIZE_IN_METERS);
      globalCellsPerAxis = 2 * globalCenterIndex + 1;

      parametersBuffer = new OpenCLFloatParameters();

      groundToSensorTransformBuffer = new OpenCLFloatBuffer(16);
      groundToSensorTransformBuffer.createOpenCLBufferObject(openCLManager);

      sensorToGroundTransformBuffer = new OpenCLFloatBuffer(16);
      sensorToGroundTransformBuffer.createOpenCLBufferObject(openCLManager);

      worldToGroundTransformBuffer = new OpenCLFloatBuffer(16);
      worldToGroundTransformBuffer.createOpenCLBufferObject(openCLManager);

      groundPlaneBuffer = new OpenCLFloatBuffer(4);
      groundPlaneBuffer.createOpenCLBufferObject(openCLManager);

      localHeightMapImage = new BytedecoImage(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_16UC1);
      localHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      globalHeightMapImage = new BytedecoImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);
      globalHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      heightMapUpdateKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapUpdateKernel");
      heightMapRegistrationKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapRegistrationKernel");
   }

   public void update(RigidBodyTransform sensorToWorldTransform, RigidBodyTransform sensorToGroundTransform, RigidBodyTransform groundToWorldTransform)
   {
      if (!processing)
      {
         currentSensorToWorldTransform.set(sensorToWorldTransform);
         sensorToGroundTransform.getTranslation().setZ(sensorToWorldTransform.getTranslationZ());

         // Upload input depth image
         inputDepthImage.writeOpenCLImage(openCLManager);

         // Fill ground plane buffer
         RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
         groundToSensorTransform.invert();

         RigidBodyTransform worldToGroundTransform = new RigidBodyTransform(groundToWorldTransform);
         worldToGroundTransform.invert();

         gridCenter.set(sensorToWorldTransform.getTranslation());

         populateParameterBuffer(gridCenter);

         // Fill world-to-sensor transform buffer
         groundToSensorTransform.get(groundToSensorTransformArray);
         groundToSensorTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(groundToSensorTransformArray);
         groundToSensorTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Fill sensor-to-world transform buffer
         sensorToGroundTransform.get(sensorToGroundTransformArray);
         sensorToGroundTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(sensorToGroundTransformArray);
         sensorToGroundTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Fill ground-to-world transform buffer
         worldToGroundTransform.get(groundToWorldTransformArray);
         worldToGroundTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(groundToWorldTransformArray);
         worldToGroundTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Set kernel arguments for the height map kernel
         openCLManager.setKernelArgument(heightMapUpdateKernel, 0, inputDepthImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 1, localHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 2, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 3, sensorToGroundTransformBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 4, groundToSensorTransformBuffer.getOpenCLBufferObject());

         // Set kernel arguments for the height map registration kernel
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 0, localHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 1, globalHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 2, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 3, worldToGroundTransformBuffer.getOpenCLBufferObject());

         // Execute kernel with length and width parameters
         openCLManager.execute2D(heightMapUpdateKernel, localCellsPerAxis, localCellsPerAxis);

         // Execute kernel with length and width parameters
         openCLManager.execute2D(heightMapRegistrationKernel, globalCellsPerAxis, globalCellsPerAxis);

         // Read height map image into CPU memory
         localHeightMapImage.readOpenCLImage(openCLManager);
         globalHeightMapImage.readOpenCLImage(openCLManager);

         sequenceNumber++;
      }
   }

   private void populateParameterBuffer(Tuple3DReadOnly gridCenter)
   {
      //// Fill parameters buffer
      parametersBuffer.setParameter(LOCAL_CELL_SIZE_IN_METERS);
      parametersBuffer.setParameter(centerIndex);
      parametersBuffer.setParameter((float) inputDepthImage.getImageHeight());
      parametersBuffer.setParameter((float) inputDepthImage.getImageWidth());
      parametersBuffer.setParameter((float) gridCenter.getX());
      parametersBuffer.setParameter((float) gridCenter.getY());
      parametersBuffer.setParameter((float) mode);
      parametersBuffer.setParameter((float) cameraIntrinsics.getCx());
      parametersBuffer.setParameter((float) cameraIntrinsics.getCy());
      parametersBuffer.setParameter((float) cameraIntrinsics.getFx());
      parametersBuffer.setParameter((float) cameraIntrinsics.getFy());
      parametersBuffer.setParameter(GLOBAL_CELL_SIZE_IN_METERS);
      parametersBuffer.setParameter((float) globalCenterIndex);
      parametersBuffer.setParameter(robotCollisionCylinderRadius);
      parametersBuffer.setParameter(gridOffsetX);
      parametersBuffer.setParameter(heightFilterAlpha);
      parametersBuffer.setParameter(localCellsPerAxis);
      parametersBuffer.setParameter(globalCellsPerAxis);
      parametersBuffer.setParameter(heightScalingFactor);
      parametersBuffer.setParameter(minHeightRegistration);
      parametersBuffer.setParameter(maxHeightRegistration);
      parametersBuffer.setParameter(minHeightDifference);
      parametersBuffer.setParameter(maxHeightDifference);
      parametersBuffer.setParameter(searchWindowHeight);
      parametersBuffer.setParameter(searchWindowWidth);

      parametersBuffer.writeOpenCLBufferObject(openCLManager);
   }

   public void setDepthIntrinsics(double fx, double fy, double cx, double cy)
   {
      cameraIntrinsics = new CameraIntrinsics();
      cameraIntrinsics.setFx(fx);
      cameraIntrinsics.setFy(fy);
      cameraIntrinsics.setCx(cx);
      cameraIntrinsics.setCy(cy);
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

   public BytedecoImage getLocalHeightMapImage()
   {
      return localHeightMapImage;
   }

   public BytedecoImage getGlobalHeightMapImage()
   {
      return globalHeightMapImage;
   }

   public int getLocalCellsPerAxis()
   {
      return localCellsPerAxis;
   }

   public int getGlobalCellsPerAxis()
   {
      return globalCellsPerAxis;
   }

   public int getCenterIndex()
   {
      return centerIndex;
   }

   public int getGlobalCenterIndex()
   {
      return globalCenterIndex;
   }

   public Point3D getGridCenter()
   {
      return gridCenter;
   }

   public int getSequenceNumber()
   {
      return sequenceNumber;
   }

   public void setDepthIntrinsics(CameraIntrinsics cameraIntrinsics)
   {
      this.cameraIntrinsics = cameraIntrinsics;
   }

   public RigidBodyTransform getSensorToWorldTransform()
   {
      return currentSensorToWorldTransform;
   }

   public void setHeightMapDataAvailable(boolean heightMapDataAvailable)
   {
      this.heightMapDataAvailable = heightMapDataAvailable;
   }

   public boolean isHeightMapDataAvailable()
   {
      return heightMapDataAvailable;
   }
}
