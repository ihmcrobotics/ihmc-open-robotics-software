package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

public class RapidHeightMapExtractor
{
   public int sequenceNumber = 0;

   public static float LOCAL_WIDTH_IN_METERS = 3.0f; // localWidthInMeters
   public static float LOCAL_CELL_SIZE_IN_METERS = 0.02f; // localCellSizeInMeters

   public static float GLOBAL_WIDTH_IN_METERS = 4.0f; // globalWidthInMeters
   public static float GLOBAL_CELL_SIZE_IN_METERS = 0.02f; // globalCellSizeInMeters

   private float internalGlobalWidthInMeters = 30.0f; // globalWidthInMeters
   private float internalGlobalCellSizeInMeters = 0.02f; // globalCellSizeInMeters

   public static float HEIGHT_SCALE_FACTOR = 10000.0f;
   public static int CROP_WINDOW_SIZE = 201;

   private int centerIndex;
   private int localCellsPerAxis;

   private int globalCenterIndex;
   private int globalCellsPerAxis;

   private int searchWindowHeight = 250;
   private int searchWindowWidth = 140;

   private float minHeightRegistration = -0.1f;
   private float maxHeightRegistration = 0.7f;
   private float minHeightDifference = -0.05f;
   private float maxHeightDifference = 0.1f;

   private float robotCollisionCylinderRadius = 0.5f;
   private float gridOffsetX = LOCAL_WIDTH_IN_METERS / 2.0f;
   private float heightFilterAlpha = 0.65f;

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

   private final Point3D sensorOrigin = new Point3D();

   private OpenCLFloatBuffer groundPlaneBuffer;
   private _cl_program rapidHeightMapUpdaterProgram;
   private _cl_kernel heightMapUpdateKernel;
   private _cl_kernel heightMapRegistrationKernel;
   private _cl_kernel contactMapKernel;
   private BytedecoImage inputDepthImage;
   private BytedecoImage localHeightMapImage;
   private BytedecoImage globalHeightMapImage;
   private BytedecoImage contactMapImage;

   private Rect cropWindowRectangle = new Rect((globalCellsPerAxis - CROP_WINDOW_SIZE) / 2, (globalCellsPerAxis - CROP_WINDOW_SIZE) / 2,
                                               CROP_WINDOW_SIZE,
                                               CROP_WINDOW_SIZE);

   private CameraIntrinsics cameraIntrinsics;

   private boolean modified = true;
   private boolean processing = false;
   private boolean heightMapDataAvailable = false;

   public void create(OpenCLManager openCLManager, BytedecoImage depthImage, int mode)
   {
      this.mode = mode;
      this.inputDepthImage = depthImage;
      this.openCLManager = openCLManager;
      rapidHeightMapUpdaterProgram = openCLManager.loadProgram("RapidHeightMapExtractor", "HeightMapUtils.cl");

      centerIndex = HeightMapTools.computeCenterIndex(LOCAL_WIDTH_IN_METERS, LOCAL_CELL_SIZE_IN_METERS);
      localCellsPerAxis = 2 * centerIndex + 1;

      globalCenterIndex = HeightMapTools.computeCenterIndex(internalGlobalWidthInMeters, internalGlobalCellSizeInMeters);
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

      contactMapImage = new BytedecoImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      contactMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      heightMapUpdateKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapUpdateKernel");
      heightMapRegistrationKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapRegistrationKernel");
      contactMapKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "contactMapKernel");
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

         sensorOrigin.set(sensorToWorldTransform.getTranslation());

         populateParameterBuffer(sensorOrigin);

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

         // Set kernel arguments for the contact map kernel
         openCLManager.setKernelArgument(contactMapKernel, 0, globalHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(contactMapKernel, 1, contactMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(contactMapKernel, 2, parametersBuffer.getOpenCLBufferObject());

         // Execute kernels with length and width parameters
         openCLManager.execute2D(heightMapUpdateKernel, localCellsPerAxis, localCellsPerAxis);
         openCLManager.execute2D(heightMapRegistrationKernel, globalCellsPerAxis, globalCellsPerAxis);
         openCLManager.execute2D(contactMapKernel, globalCellsPerAxis, globalCellsPerAxis);

         // Read height map image into CPU memory
         localHeightMapImage.readOpenCLImage(openCLManager);
         globalHeightMapImage.readOpenCLImage(openCLManager);
         contactMapImage.readOpenCLImage(openCLManager);

         PerceptionDebugTools.printMat("Contact Map", contactMapImage.getBytedecoOpenCVMat(), 16);

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
      parametersBuffer.setParameter(HEIGHT_SCALE_FACTOR);
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

   public void reset()
   {
      localHeightMapImage.getBytedecoOpenCVMat().put(new Scalar(0));
      globalHeightMapImage.getBytedecoOpenCVMat().put(new Scalar(0));
      sequenceNumber = 0;
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

   public BytedecoImage getInternalGlobalHeightMapImage()
   {
      return globalHeightMapImage;
   }

   public Mat getCroppedGlobalHeightMapImage()
   {
      int xIndex = HeightMapTools.coordinateToIndex(sensorOrigin.getX(), 0, GLOBAL_CELL_SIZE_IN_METERS, globalCenterIndex);
      int yIndex = HeightMapTools.coordinateToIndex(sensorOrigin.getY(), 0, GLOBAL_CELL_SIZE_IN_METERS, globalCenterIndex);
      cropWindowRectangle = new Rect((yIndex - CROP_WINDOW_SIZE / 2), (xIndex - CROP_WINDOW_SIZE / 2), CROP_WINDOW_SIZE, CROP_WINDOW_SIZE);
      return globalHeightMapImage.getBytedecoOpenCVMat().apply(cropWindowRectangle);
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

   public Point3D getSensorOrigin()
   {
      return sensorOrigin;
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
