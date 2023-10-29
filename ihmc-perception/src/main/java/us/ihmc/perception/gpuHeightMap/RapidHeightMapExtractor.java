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
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

public class RapidHeightMapExtractor
{
   public int sequenceNumber = 0;

   public static float GLOBAL_WIDTH_IN_METERS = 4.0f; // globalWidthInMeters
   public static float GLOBAL_CELL_SIZE_IN_METERS = 0.02f; // globalCellSizeInMeters

   public static int CROP_WINDOW_SIZE = 201;

   private int centerIndex;
   private int localCellsPerAxis;

   private int globalCenterIndex;
   private int globalCellsPerAxis;

   private float gridOffsetX;

   private int mode = 1; // 0 -> Ouster, 1 -> Realsense

   private static HeightMapParameters heightMapParameters = new HeightMapParameters("GPU");

   private OpenCLManager openCLManager;
   private OpenCLFloatParameters parametersBuffer;

   private RigidBodyTransform currentSensorToWorldTransform = new RigidBodyTransform();

   private OpenCLFloatBuffer worldToGroundTransformBuffer;
   private float[] worldToGroundTransformArray = new float[16];

   private OpenCLFloatBuffer groundToWorldTransformBuffer;
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
   private _cl_kernel croppingKernel;
   private _cl_kernel terrainCostKernel;
   private _cl_kernel contactMapKernel;

   private BytedecoImage inputDepthImage;
   private BytedecoImage localHeightMapImage;
   private BytedecoImage globalHeightMapImage;
   private BytedecoImage globalHeightVarianceImage;
   private BytedecoImage sensorCroppedHeightMapImage;
   private BytedecoImage terrainCostImage;
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

      centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getLocalCellSizeInMeters());
      localCellsPerAxis = 2 * centerIndex + 1;

      gridOffsetX = (float) heightMapParameters.getLocalWidthInMeters() / 2.0f;

      globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getInternalGlobalWidthInMeters(),
                                                            heightMapParameters.getInternalGlobalCellSizeInMeters());
      globalCellsPerAxis = 2 * globalCenterIndex + 1;

      parametersBuffer = new OpenCLFloatParameters();

      groundToSensorTransformBuffer = new OpenCLFloatBuffer(16);
      groundToSensorTransformBuffer.createOpenCLBufferObject(openCLManager);

      sensorToGroundTransformBuffer = new OpenCLFloatBuffer(16);
      sensorToGroundTransformBuffer.createOpenCLBufferObject(openCLManager);

      worldToGroundTransformBuffer = new OpenCLFloatBuffer(16);
      worldToGroundTransformBuffer.createOpenCLBufferObject(openCLManager);

      groundToWorldTransformBuffer = new OpenCLFloatBuffer(16);
      groundToWorldTransformBuffer.createOpenCLBufferObject(openCLManager);

      groundPlaneBuffer = new OpenCLFloatBuffer(4);
      groundPlaneBuffer.createOpenCLBufferObject(openCLManager);

      localHeightMapImage = new BytedecoImage(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_16UC1);
      localHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      globalHeightMapImage = new BytedecoImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);
      globalHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      globalHeightVarianceImage = new BytedecoImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      globalHeightVarianceImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      sensorCroppedHeightMapImage = new BytedecoImage(CROP_WINDOW_SIZE, CROP_WINDOW_SIZE, opencv_core.CV_16UC1);
      sensorCroppedHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      terrainCostImage = new BytedecoImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      terrainCostImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      contactMapImage = new BytedecoImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      contactMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      heightMapUpdateKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapUpdateKernel");
      heightMapRegistrationKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapRegistrationKernel");
      croppingKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "croppingKernel");
      terrainCostKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "terrainCostKernel");
      contactMapKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "contactMapKernel");
      
      reset();
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

         // Fill world-to-ground transform buffer
         worldToGroundTransform.get(worldToGroundTransformArray);
         worldToGroundTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(worldToGroundTransformArray);
         worldToGroundTransformBuffer.writeOpenCLBufferObject(openCLManager);

         //fill ground-to-world transform buffer
         groundToWorldTransform.get(groundToWorldTransformArray);
         groundToWorldTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(groundToWorldTransformArray);
         groundToWorldTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Set kernel arguments for the height map kernel
         openCLManager.setKernelArgument(heightMapUpdateKernel, 0, inputDepthImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 1, localHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 2, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 3, sensorToGroundTransformBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 4, groundToSensorTransformBuffer.getOpenCLBufferObject());

         // Set kernel arguments for the height map registration kernel
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 0, localHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 1, globalHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 2, globalHeightVarianceImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 3, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 4, worldToGroundTransformBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapRegistrationKernel, 5, sensorToGroundTransformBuffer.getOpenCLBufferObject());

         // Set kernel arguments for the cropping kernel
         openCLManager.setKernelArgument(croppingKernel, 0, globalHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(croppingKernel, 1, sensorCroppedHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(croppingKernel, 2, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(croppingKernel, 3, groundToWorldTransformBuffer.getOpenCLBufferObject());

         // Set kernel arguments for the terrain cost kernel
         openCLManager.setKernelArgument(terrainCostKernel, 0, globalHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(terrainCostKernel, 1, terrainCostImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(terrainCostKernel, 2, parametersBuffer.getOpenCLBufferObject());

         // Set kernel arguments for the contact map kernel
         openCLManager.setKernelArgument(contactMapKernel, 0, terrainCostImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(contactMapKernel, 1, contactMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(contactMapKernel, 2, parametersBuffer.getOpenCLBufferObject());

         // Execute kernels with length and width parameters
         openCLManager.execute2D(heightMapUpdateKernel, localCellsPerAxis, localCellsPerAxis);
         openCLManager.execute2D(heightMapRegistrationKernel, globalCellsPerAxis, globalCellsPerAxis);
         openCLManager.execute2D(croppingKernel, CROP_WINDOW_SIZE, CROP_WINDOW_SIZE);
         openCLManager.execute2D(terrainCostKernel, globalCellsPerAxis, globalCellsPerAxis);
         openCLManager.execute2D(contactMapKernel, globalCellsPerAxis, globalCellsPerAxis);

         // Read height map image into CPU memory
         localHeightMapImage.readOpenCLImage(openCLManager);
         globalHeightMapImage.readOpenCLImage(openCLManager);
         sensorCroppedHeightMapImage.readOpenCLImage(openCLManager);
         terrainCostImage.readOpenCLImage(openCLManager);
         contactMapImage.readOpenCLImage(openCLManager);

         sequenceNumber++;
      }
   }

   private void populateParameterBuffer(Tuple3DReadOnly gridCenter)
   {
      //// Fill parameters buffer
      parametersBuffer.setParameter((float) heightMapParameters.getLocalCellSizeInMeters());
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
      parametersBuffer.setParameter((float) heightMapParameters.getGlobalCellSizeInMeters());
      parametersBuffer.setParameter((float) globalCenterIndex);
      parametersBuffer.setParameter((float) heightMapParameters.getRobotCollisionCylinderRadius());
      parametersBuffer.setParameter(gridOffsetX);
      parametersBuffer.setParameter((float) heightMapParameters.getHeightFilterAlpha());
      parametersBuffer.setParameter(localCellsPerAxis);
      parametersBuffer.setParameter(globalCellsPerAxis);
      parametersBuffer.setParameter((float) heightMapParameters.getHeightScaleFactor());
      parametersBuffer.setParameter((float) heightMapParameters.getMinHeightRegistration());
      parametersBuffer.setParameter((float) heightMapParameters.getMaxHeightRegistration());
      parametersBuffer.setParameter((float) heightMapParameters.getMinHeightDifference());
      parametersBuffer.setParameter((float) heightMapParameters.getMaxHeightDifference());
      parametersBuffer.setParameter((float) heightMapParameters.getSearchWindowHeight());
      parametersBuffer.setParameter((float) heightMapParameters.getSearchWindowWidth());
      parametersBuffer.setParameter((float) CROP_WINDOW_SIZE / 2);
      parametersBuffer.setParameter((float) heightMapParameters.getMinClampHeight());
      parametersBuffer.setParameter((float) heightMapParameters.getMaxClampHeight());
      parametersBuffer.setParameter((float) heightMapParameters.getHeightOffset());
      parametersBuffer.setParameter((float) heightMapParameters.getSteppingCosineThreshold());
      parametersBuffer.setParameter((float) heightMapParameters.getSteppingContactThreshold());
      parametersBuffer.setParameter((float) heightMapParameters.getContactWindowSize());
      parametersBuffer.setParameter((float) heightMapParameters.getSpatialAlpha());

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
      localHeightMapImage.getBytedecoOpenCVMat().put(new Scalar(32768));
      localHeightMapImage.writeOpenCLImage(openCLManager);
      globalHeightMapImage.getBytedecoOpenCVMat().put(new Scalar(32768));
      globalHeightMapImage.writeOpenCLImage(openCLManager);
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

   public BytedecoImage getInternalGlobalHeightMapImage()
   {
      return globalHeightMapImage;
   }

   public Mat getCroppedGlobalHeightMapImage()
   {
      return getCroppedImage(sensorOrigin, globalCenterIndex, globalHeightMapImage.getBytedecoOpenCVMat());
   }

   public BytedecoImage getSensorCroppedHeightMapImage()
   {
      return sensorCroppedHeightMapImage;
   }

   public Mat getCroppedTerrainCostImage()
   {
      return getCroppedImage(sensorOrigin, globalCenterIndex, terrainCostImage.getBytedecoOpenCVMat());
   }

   public Mat getCroppedContactMapImage()
   {
      return getCroppedImage(sensorOrigin, globalCenterIndex, contactMapImage.getBytedecoOpenCVMat());
   }

   public Mat getCroppedImage(Point3D origin, int globalCenterIndex, Mat imageToCrop)
   {
      int xIndex = HeightMapTools.coordinateToIndex(origin.getX(), 0, GLOBAL_CELL_SIZE_IN_METERS, globalCenterIndex);
      int yIndex = HeightMapTools.coordinateToIndex(origin.getY(), 0, GLOBAL_CELL_SIZE_IN_METERS, globalCenterIndex);
      cropWindowRectangle = new Rect((yIndex - CROP_WINDOW_SIZE / 2), (xIndex - CROP_WINDOW_SIZE / 2), CROP_WINDOW_SIZE, CROP_WINDOW_SIZE);
      return imageToCrop.apply(cropWindowRectangle);
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

   public static HeightMapParameters getHeightMapParameters()
   {
      return heightMapParameters;
   }
}
