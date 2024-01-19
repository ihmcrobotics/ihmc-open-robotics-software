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
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParametersBasics;
import us.ihmc.perception.steppableRegions.SteppableRegionsCalculator;
import us.ihmc.perception.steppableRegions.data.SteppableCell;
import us.ihmc.perception.steppableRegions.data.SteppableRegionsEnvironmentModel;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

/**
 * Extracts height map and some other cost metric maps on the GPU using OpenCL kernels
 *
 * There are two each of height map, terrain cost map, and contact map, corresponding to global and local (cropped) versions.
 * The terrain cost map is the single footstep steppability value, and the feasible contact map is the distance transform map
 * which computes the distance to closest unsteppable cell for each cell. Feasible contact map (16-bit scalar for distance transform of the terrain cost map, represents safety score
 * for distance away from boundaries and edges for each cell). For more information on Distance Transform visit:
 * https://en.wikipedia.org/wiki/Distance_transform
 * */
public class RapidHeightMapExtractor
{
   private int mode = 1; // 0 -> Ouster, 1 -> Realsense
   private float gridOffsetX;
   private int centerIndex;
   private int localCellsPerAxis;
   private int globalCenterIndex;
   private int cropCenterIndex;
   private int globalCellsPerAxis;
   public int sequenceNumber = 0;

   private static final boolean computeSteppability = false;

   private boolean initialized = false;
   private boolean modified = true;
   private boolean processing = false;
   private boolean heightMapDataAvailable = false;

   private final SteppableRegionCalculatorParameters steppableRegionParameters = new SteppableRegionCalculatorParameters();

   private static HeightMapParameters heightMapParameters = new HeightMapParameters("GPU");
   private final RigidBodyTransform currentSensorToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform currentGroundToWorldTransform = new RigidBodyTransform();
   private final Point3D sensorOrigin = new Point3D();
   private final TerrainMapStatistics terrainMapStatistics = new TerrainMapStatistics();

//   private HeightMapAutoencoder denoiser;
   private HumanoidReferenceFrames referenceFrames;
   private OpenCLManager openCLManager;
   private OpenCLFloatParameters parametersBuffer;
   private OpenCLFloatParameters snappingParametersBuffer;
   private OpenCLFloatBuffer worldToGroundTransformBuffer;
   private OpenCLFloatBuffer groundToWorldTransformBuffer;
   private OpenCLFloatBuffer groundToSensorTransformBuffer;
   private OpenCLFloatBuffer sensorToGroundTransformBuffer;
   private OpenCLFloatBuffer groundPlaneBuffer;

   private final OpenCLFloatParameters yaw = new OpenCLFloatParameters();

   private CameraIntrinsics cameraIntrinsics;
   private BytedecoImage inputDepthImage;
   private BytedecoImage localHeightMapImage;
   private BytedecoImage globalHeightMapImage;
   private BytedecoImage globalHeightVarianceImage;
   private BytedecoImage terrainCostImage;
   private BytedecoImage contactMapImage;

   private BytedecoImage sensorCroppedHeightMapImage;
   private BytedecoImage sensorCroppedTerrainCostImage;
   private BytedecoImage sensorCroppedContactMapImage;
   private Mat steppableRegionAssignmentMat;
   private Mat steppableRegionRingMat;

   private BytedecoImage steppabilityImage;
   private BytedecoImage snapHeightImage;
   private BytedecoImage snapNormalXImage;
   private BytedecoImage snapNormalYImage;
   private BytedecoImage snapNormalZImage;
   private BytedecoImage steppabilityConnectionsImage;

   private _cl_program rapidHeightMapUpdaterProgram;
   private _cl_kernel heightMapUpdateKernel;
   private _cl_kernel heightMapRegistrationKernel;
   private _cl_kernel terrainCostKernel;
   private _cl_kernel contactMapKernel;
   private _cl_kernel croppingKernel;

   private _cl_kernel computeSnappedValuesKernel;
   private _cl_kernel computeSteppabilityConnectionsKernel;

   private float[] worldToGroundTransformArray = new float[16];
   private float[] groundToWorldTransformArray = new float[16];
   private float[] groundToSensorTransformArray = new float[16];
   private float[] sensorToGroundTransformArray = new float[16];

   private TerrainMapData terrainMapData;
   private Mat denoisedHeightMapImage;
   private Rect cropWindowRectangle;

   public RapidHeightMapExtractor(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;
      //      denoiser = new HeightMapAutoencoder();
      rapidHeightMapUpdaterProgram = openCLManager.loadProgram("RapidHeightMapExtractor", "HeightMapUtils.cl");
   }

   public RapidHeightMapExtractor(OpenCLManager openCLManager, HumanoidReferenceFrames referenceFrames)
   {
      this(openCLManager);
      this.referenceFrames = referenceFrames;
   }

   public void initialize()
   {
      recomputeDerivedParameters();
      cropWindowRectangle = new Rect((globalCellsPerAxis - heightMapParameters.getCropWindowSize()) / 2,
                                     (globalCellsPerAxis - heightMapParameters.getCropWindowSize()) / 2,
                                     heightMapParameters.getCropWindowSize(),
                                     heightMapParameters.getCropWindowSize());

      parametersBuffer = new OpenCLFloatParameters();

      if (computeSteppability)
         snappingParametersBuffer = new OpenCLFloatParameters();

      groundToSensorTransformBuffer = new OpenCLFloatBuffer(16);
      sensorToGroundTransformBuffer = new OpenCLFloatBuffer(16);
      worldToGroundTransformBuffer = new OpenCLFloatBuffer(16);
      groundToWorldTransformBuffer = new OpenCLFloatBuffer(16);
      groundPlaneBuffer = new OpenCLFloatBuffer(4);

      groundToSensorTransformBuffer.createOpenCLBufferObject(openCLManager);
      sensorToGroundTransformBuffer.createOpenCLBufferObject(openCLManager);
      worldToGroundTransformBuffer.createOpenCLBufferObject(openCLManager);
      groundToWorldTransformBuffer.createOpenCLBufferObject(openCLManager);
      groundPlaneBuffer.createOpenCLBufferObject(openCLManager);

      terrainMapData = new TerrainMapData(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());
      denoisedHeightMapImage = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);
      steppableRegionAssignmentMat = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);
      steppableRegionRingMat = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);

      createLocalHeightMapImage(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_16UC1);
      createGlobalHeightMapImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);
      createGlobalHeightVarianceImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      createTerrainCostImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      createContactMapImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      createSteppabilityMapImages(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());
      createSteppabilityMapImages(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());
      createSensorCroppedHeightMapImage(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);
      createSensorCroppedTerrainCostImage(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);
      createSensorCroppedContactMapImage(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);

      heightMapUpdateKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapUpdateKernel");
      heightMapRegistrationKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapRegistrationKernel");
      terrainCostKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "terrainCostKernel");
      contactMapKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "contactMapKernel");
      croppingKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "croppingKernel");

      if (computeSteppability)
      {
         computeSnappedValuesKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "computeSnappedValuesKernel");
         computeSnappedValuesKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "computeSnappedValuesKernel");
         computeSteppabilityConnectionsKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "computeSteppabilityConnectionsKernel");
      }
   }

   public void create(BytedecoImage depthImage, int mode)
   {
      this.inputDepthImage = depthImage;
      this.mode = mode;

      initialize();
      reset();
   }

   public void recomputeDerivedParameters()
   {
      centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getLocalCellSizeInMeters());
      localCellsPerAxis = 2 * centerIndex + 1;
      gridOffsetX = (float) heightMapParameters.getLocalWidthInMeters() / 2.0f;
      globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getInternalGlobalWidthInMeters(),
                                                            heightMapParameters.getInternalGlobalCellSizeInMeters());
      globalCellsPerAxis = 2 * globalCenterIndex + 1;

      cropCenterIndex = (heightMapParameters.getCropWindowSize() - 1) / 2;

      if (2 * cropCenterIndex + 1 != heightMapParameters.getCropWindowSize())
         throw new RuntimeException("The crop center index was computed incorrectly.");
   }

   public void update(RigidBodyTransform sensorToWorldTransform, RigidBodyTransform sensorToGroundTransform, RigidBodyTransform groundToWorldTransform)
   {
      if (!processing)
      {
         terrainMapStatistics.startTotalTime();

         currentGroundToWorldTransform.set(groundToWorldTransform);
         currentSensorToWorldTransform.set(sensorToWorldTransform);
         sensorToGroundTransform.getTranslation().setZ(sensorToWorldTransform.getTranslationZ());

         // Upload input depth image
         terrainMapStatistics.startDepthUploadTime();
         inputDepthImage.writeOpenCLImage(openCLManager);
         terrainMapStatistics.endDepthUploadTime();

         terrainMapStatistics.startCPUProcessingTime();
         RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
         groundToSensorTransform.invert();

         RigidBodyTransform worldToGroundTransform = new RigidBodyTransform(groundToWorldTransform);
         worldToGroundTransform.invert();

         sensorOrigin.set(sensorToWorldTransform.getTranslation());

         populateParameterBuffers(heightMapParameters, cameraIntrinsics, sensorOrigin);

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

         terrainMapStatistics.endCPUProcessingTime();

         terrainMapStatistics.startGPUProcessingTime();
         // execute height map extraction and registration kernels
         openCLManager.execute2D(heightMapUpdateKernel, localCellsPerAxis, localCellsPerAxis);
         openCLManager.execute2D(heightMapRegistrationKernel, globalCellsPerAxis, globalCellsPerAxis);

         // compute and read terrain cost and contact map images
         computeContactMap();

         // compute the steppable height image
         if (computeSteppability)
         {
            computeSteppabilityImage();
         }

         terrainMapStatistics.startTerrainMapDownloadTime();

         terrainMapData.setSensorOrigin(groundToWorldTransform.getTranslationX(), groundToWorldTransform.getTranslationY());
         terrainMapData.setHeightMap(getCroppedImage_OpenCL(globalHeightMapImage, sensorCroppedHeightMapImage, parametersBuffer));
         terrainMapData.setContactMap(getCroppedImage_OpenCL(contactMapImage, sensorCroppedContactMapImage, parametersBuffer));
         //terrainMapData.setTerrainCostMap(getCroppedImageOnKernel(terrainCostImage, sensorCroppedTerrainCostImage, parametersBuffer));

         terrainMapStatistics.endTerrainMapDownloadTime();

         if (heightMapParameters.getDenoiserEnabled())
         {
//            denoisedHeightMapImage = denoiser.denoiseHeightMap(croppedHeightMapImage, heightMapParameters.getHeightOffset());
         }
         terrainMapStatistics.endGPUProcessingTime();

         terrainMapStatistics.endTotalTime();


         // Use for debugging by printing to console the height map and contact map values
         //PerceptionDebugTools.printMat("Internal Original Height Map", globalHeightMapImage.getBytedecoOpenCVMat(), 600, 600, 900, 900, 10);
         //PerceptionDebugTools.printMat("Internal Snap Height Map", snapHeightImage.getBytedecoOpenCVMat(), 600, 600, 900, 900, 10);
         //PerceptionDebugTools.printMat("Cropped Height Map", croppedHeightMapImage, 4);
         //PerceptionDebugTools.printMat("Cropped Snap Height Map", croppedSnappedMapImage, 4);

         if (computeSteppability)
         {
            SteppableRegionsEnvironmentModel environment = SteppableRegionsCalculator.createEnvironmentByMergingCellsIntoRegions(steppabilityImage,
                                                                                                                                 snapHeightImage,
                                                                                                                                 snapNormalXImage,
                                                                                                                                 snapNormalYImage,
                                                                                                                                 snapNormalZImage,
                                                                                                                                 steppabilityConnectionsImage,
                                                                                                                                 steppableRegionParameters,
                                                                                                                                 sensorOrigin.getX(),
                                                                                                                                 sensorOrigin.getY(),
                                                                                                                                 heightMapParameters.getGridResolutionXY(),
                                                                                                                                 cropCenterIndex);
            generateSteppableRegionDebugImage(environment);
         }

         //double cropWindowSize = cropCenterIndex * getHeightMapParameters().getGridResolutionXY() * 2.0;
         /*
         SteppableRegionsList regions = SteppableRegionsCalculator.createSteppableRegions(concaveHullParameters,
                                                                                          polygonizerParameters,
                                                                                          parameters,
                                                                                          environment,
                                                                                          sensorOrigin.getX(),
                                                                                          sensorOrigin.getY(),
                                                                                          cropWindowSize,
                                                                                          heightMapParameters.getGridResolutionXY(),
                                                                                          cropCenterIndex,
                                                                                          0.0);
          */

         sequenceNumber++;

         terrainMapStatistics.setPrintToConsole(true); // uncommint this to print time statistics to the console
         terrainMapStatistics.logToFile(heightMapParameters.getStatisticsLoggingEnabled());
      }
   }

   public void populateParameterBuffers(HeightMapParameters parameters, CameraIntrinsics cameraIntrinsics, Tuple3DReadOnly gridCenter)
   {
      //// Fill parameters buffer
      parametersBuffer.setParameter((float) parameters.getLocalCellSizeInMeters());
      parametersBuffer.setParameter(centerIndex);
      parametersBuffer.setParameter((float) cameraIntrinsics.getHeight());
      parametersBuffer.setParameter((float) cameraIntrinsics.getWidth());
      parametersBuffer.setParameter((float) gridCenter.getX());
      parametersBuffer.setParameter((float) gridCenter.getY());
      parametersBuffer.setParameter((float) mode);
      parametersBuffer.setParameter((float) cameraIntrinsics.getCx());
      parametersBuffer.setParameter((float) cameraIntrinsics.getCy());
      parametersBuffer.setParameter((float) cameraIntrinsics.getFx());
      parametersBuffer.setParameter((float) cameraIntrinsics.getFy());
      parametersBuffer.setParameter((float) parameters.getGlobalCellSizeInMeters());
      parametersBuffer.setParameter((float) globalCenterIndex);
      parametersBuffer.setParameter((float) parameters.getRobotCollisionCylinderRadius());
      parametersBuffer.setParameter(gridOffsetX);
      parametersBuffer.setParameter((float) parameters.getHeightFilterAlpha());
      parametersBuffer.setParameter(localCellsPerAxis);
      parametersBuffer.setParameter(globalCellsPerAxis);
      parametersBuffer.setParameter((float) parameters.getHeightScaleFactor());
      parametersBuffer.setParameter((float) parameters.getMinHeightRegistration());
      parametersBuffer.setParameter((float) parameters.getMaxHeightRegistration());
      parametersBuffer.setParameter((float) parameters.getMinHeightDifference());
      parametersBuffer.setParameter((float) parameters.getMaxHeightDifference());
      parametersBuffer.setParameter((float) parameters.getSearchWindowHeight());
      parametersBuffer.setParameter((float) parameters.getSearchWindowWidth());
      parametersBuffer.setParameter((float) cropCenterIndex);
      parametersBuffer.setParameter((float) parameters.getMinClampHeight());
      parametersBuffer.setParameter((float) parameters.getMaxClampHeight());
      parametersBuffer.setParameter((float) parameters.getHeightOffset());
      parametersBuffer.setParameter((float) parameters.getSteppingCosineThreshold());
      parametersBuffer.setParameter((float) parameters.getSteppingContactThreshold());
      parametersBuffer.setParameter((float) parameters.getContactWindowSize());
      parametersBuffer.setParameter((float) parameters.getSpatialAlpha());
      parametersBuffer.setParameter((float) parameters.getSearchSkipSize());
      parametersBuffer.setParameter((float) parameters.getVerticalSearchSize());
      parametersBuffer.setParameter((float) parameters.getVerticalSearchResolution());
      parametersBuffer.setParameter((float) parameters.getFastSearchSize());

      parametersBuffer.writeOpenCLBufferObject(openCLManager);

      if (computeSteppability)
      {
         snappingParametersBuffer.setParameter((float) gridCenter.getX());
         snappingParametersBuffer.setParameter((float) gridCenter.getY());
         snappingParametersBuffer.setParameter((float) parameters.getGlobalCellSizeInMeters());
         snappingParametersBuffer.setParameter(globalCenterIndex);
         snappingParametersBuffer.setParameter((float) cropCenterIndex);
         snappingParametersBuffer.setParameter((float) parameters.getHeightScaleFactor());
         snappingParametersBuffer.setParameter((float) parameters.getHeightOffset());
         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getFootLength());
         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getFootWidth());
         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getDistanceFromCliffTops());
         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getDistanceFromCliffBottoms());
         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getCliffStartHeightToAvoid());
         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getCliffEndHeightToAvoid());
         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getMinSupportAreaFraction());
         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getMinSnapHeightThreshold());
         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getSnapHeightThresholdAtSearchEdge());
         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getInequalityActivationSlope());

         snappingParametersBuffer.writeOpenCLBufferObject(openCLManager);
      }

      initialized = true;
   }

   public void computeContactMap()
   {
      // Set kernel arguments for the terrain cost kernel
      openCLManager.setKernelArgument(terrainCostKernel, 0, globalHeightMapImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(terrainCostKernel, 1, terrainCostImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(terrainCostKernel, 2, parametersBuffer.getOpenCLBufferObject());

      // Set kernel arguments for the contact map kernel
      openCLManager.setKernelArgument(contactMapKernel, 0, terrainCostImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(contactMapKernel, 1, contactMapImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(contactMapKernel, 2, parametersBuffer.getOpenCLBufferObject());

      // Execute kernels with length and width parameters
      openCLManager.execute2D(terrainCostKernel, globalCellsPerAxis, globalCellsPerAxis);
      openCLManager.execute2D(contactMapKernel, globalCellsPerAxis, globalCellsPerAxis);

      openCLManager.join();
   }

   public void readContactMapImage()
   {
      // Read height map image into CPU memory
      terrainCostImage.readOpenCLImage(openCLManager);
      contactMapImage.readOpenCLImage(openCLManager);
   }

   public void computeSteppabilityImage()
   {
      yaw.setParameter(0.0f); // we're only doing a single discretization, and then assuming the foot is a big rectangle
      yaw.writeOpenCLBufferObject(openCLManager);

      openCLManager.setKernelArgument(computeSnappedValuesKernel, 0, snappingParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 1, globalHeightMapImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 2, yaw.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 3, steppabilityImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 4, snapHeightImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 5, snapNormalXImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 6, snapNormalYImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSnappedValuesKernel, 7, snapNormalZImage.getOpenCLImageObject());

      openCLManager.execute2D(computeSnappedValuesKernel, heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());

      snapHeightImage.readOpenCLImage(openCLManager);
      snapNormalXImage.readOpenCLImage(openCLManager);
      snapNormalYImage.readOpenCLImage(openCLManager);
      snapNormalZImage.readOpenCLImage(openCLManager);

      openCLManager.setKernelArgument(computeSteppabilityConnectionsKernel, 0, snappingParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeSteppabilityConnectionsKernel, 1, steppabilityImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSteppabilityConnectionsKernel, 2, steppabilityConnectionsImage.getOpenCLImageObject());

      openCLManager.execute2D(computeSteppabilityConnectionsKernel, heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());

      steppabilityImage.readOpenCLImage(openCLManager);
      steppabilityConnectionsImage.readOpenCLImage(openCLManager);

      openCLManager.join();
   }

   public void readTerrainCostImage()
   {
      terrainCostImage.readOpenCLImage(openCLManager);
   }

   public void reset()
   {
      double thicknessOfTheFoot = 0.02;
      double height = 0.0f;

      if (referenceFrames != null)
      {
         height = referenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame().getTranslationZ() - thicknessOfTheFoot;
      }

      int offset = (int) ((height + heightMapParameters.getHeightOffset()) * heightMapParameters.getHeightScaleFactor());
      localHeightMapImage.getBytedecoOpenCVMat().put(new Scalar(offset));
      globalHeightMapImage.getBytedecoOpenCVMat().put(new Scalar(offset));

      localHeightMapImage.writeOpenCLImage(openCLManager);
      globalHeightMapImage.writeOpenCLImage(openCLManager);

      if (computeSteppability)
      {
         snapHeightImage.getBytedecoOpenCVMat().put(new Scalar(32768));
         snapHeightImage.writeOpenCLImage(openCLManager);
      }

      sequenceNumber = 0;
   }

   public void createLocalHeightMapImage(int height, int width, int type)
   {
      localHeightMapImage = new BytedecoImage(width, height, type);
      localHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createGlobalHeightMapImage(int height, int width, int type)
   {
      globalHeightMapImage = new BytedecoImage(width, height, type);
      globalHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createGlobalHeightVarianceImage(int height, int width, int type)
   {
      globalHeightVarianceImage = new BytedecoImage(width, height, type);
      globalHeightVarianceImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createSensorCroppedHeightMapImage(int height, int width, int type)
   {
      sensorCroppedHeightMapImage = new BytedecoImage(width, height, type);
      sensorCroppedHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createSensorCroppedTerrainCostImage(int height, int width, int type)
   {
      sensorCroppedTerrainCostImage = new BytedecoImage(width, height, type);
      sensorCroppedTerrainCostImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createSensorCroppedContactMapImage(int height, int width, int type)
   {
      sensorCroppedContactMapImage = new BytedecoImage(width, height, type);
      sensorCroppedContactMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createTerrainCostImage(int height, int width, int type)
   {
      terrainCostImage = new BytedecoImage(width, height, type);
      terrainCostImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createContactMapImage(int height, int width, int type)
   {
      contactMapImage = new BytedecoImage(width, height, type);
      contactMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void createSteppabilityMapImages(int height, int width)
   {
      steppabilityImage = new BytedecoImage(width, height, opencv_core.CV_8UC1);
      snapHeightImage = new BytedecoImage(width, height, opencv_core.CV_16UC1);
      snapNormalXImage = new BytedecoImage(width, height, opencv_core.CV_16UC1);
      snapNormalYImage = new BytedecoImage(width, height, opencv_core.CV_16UC1);
      snapNormalZImage = new BytedecoImage(width, height, opencv_core.CV_16UC1);
      steppabilityConnectionsImage = new BytedecoImage(width, height, opencv_core.CV_8UC1);

      steppabilityImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      snapHeightImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      snapNormalXImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      snapNormalYImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      snapNormalZImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      steppabilityConnectionsImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
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

   public TerrainMapData getTerrainMapData()
   {
      return terrainMapData;
   }

   public BytedecoImage getSteppableHeightMapImage()
   {
      return snapHeightImage;
   }

   public BytedecoImage getSteppabilityImage()
   {
      return steppabilityImage;
   }

   public BytedecoImage getSnapNormalZImage()
   {
      return snapNormalZImage;
   }

   //public Mat getCroppedGlobalHeightMapImage()
   //{
   //   return heightMapParameters.getDenoiserEnabled() ? denoisedHeightMapImage : terrainMapData.getHeightMap();
   //}
   //
   //public Mat getDenoisedHeightMapImage()
   //{
   //   return denoisedHeightMapImage;
   //}
   //
   //public Mat getSensorCroppedHeightMapImage()
   //{
   //   return getCroppedGlobalHeightMapImage();
   //}
   //
   //public Mat getCroppedTerrainCostImage()
   //{
   //   return terrainMapData.getTerrainCostMap();
   //}
   //
   //public Mat getCroppedContactMapImage()
   //{
   //   return terrainMapData.getContactMap();
   //}

   public Mat getSteppableRegionAssignmentMat()
   {
      return steppableRegionAssignmentMat;
   }

   public Mat getSteppableRegionRingMat()
   {
      return steppableRegionRingMat;
   }

   public BytedecoImage getSteppabilityConnectionsImage()
   {
      return steppabilityConnectionsImage;
   }

//   public Mat getDenoisedHeightMap()
//   {
//      return denoisedHeightMap;
//   }

   public Mat getGlobalContactImage()
   {
      return contactMapImage.getBytedecoOpenCVMat();
   }

   public Mat getCroppedImage_OpenCL(BytedecoImage inputMap, BytedecoImage croppedMap, OpenCLFloatParameters parametersBuffer)
   {
      openCLManager.setKernelArgument(croppingKernel, 0, inputMap.getOpenCLImageObject());
      openCLManager.setKernelArgument(croppingKernel, 1, croppedMap.getOpenCLImageObject());
      openCLManager.setKernelArgument(croppingKernel, 2, parametersBuffer.getOpenCLBufferObject());

      openCLManager.execute2D(croppingKernel, heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());
      croppedMap.readOpenCLImage(openCLManager);
      openCLManager.join();
      return croppedMap.getBytedecoOpenCVMat().clone();
   }

   private void generateSteppableRegionDebugImage(SteppableRegionsEnvironmentModel environmentModel)
   {
      int cellsPerSide = heightMapParameters.getCropWindowSize();

      for (int x = 0; x < cellsPerSide; x++)
      {
         for (int y = 0; y < cellsPerSide; y++)
         {
            SteppableCell steppableCell = environmentModel.getCellAt(x, y);
            int value;
            if (steppableCell == null)
               value = 0;
            else
               value = steppableCell.getRegion().regionNumber + 1;

            steppableRegionAssignmentMat.ptr(x, y).putShort((short) value);

            if (steppableCell == null)
            {
               if (Integer.bitCount(steppabilityConnectionsImage.getByteAsInteger(x, y)) != 0)
                  throw new RuntimeException("Crap");
               steppableRegionRingMat.ptr(x, y).putChar((char) 0);
            }
            else if (steppableCell.isBorderCell())
            {
               if (Integer.bitCount(steppabilityConnectionsImage.getByteAsInteger(x, y)) >= 8)
                  throw new RuntimeException("Crap");
               steppableRegionRingMat.ptr(x, y).putChar((char) 2); // outside, make it white
            }
            else
            {
               if (Integer.bitCount(steppabilityConnectionsImage.getByteAsInteger(x, y)) != 8)
                  throw new RuntimeException("Crap");
               steppableRegionRingMat.ptr(x, y).putChar((char) 1); // interior, make it gray
            }
         }
      }
   }

   public Mat getCroppedImage(Point3DReadOnly origin, int globalCenterIndex, Mat imageToCrop)
   {
      int xIndex = HeightMapTools.coordinateToIndex(origin.getX(), 0, RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(), globalCenterIndex);
      int yIndex = HeightMapTools.coordinateToIndex(origin.getY(), 0, RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(), globalCenterIndex);
      cropWindowRectangle = new Rect((yIndex - heightMapParameters.getCropWindowSize() / 2),
                                     (xIndex - heightMapParameters.getCropWindowSize() / 2),
                                     heightMapParameters.getCropWindowSize(),
                                     heightMapParameters.getCropWindowSize());
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

   public int getGlobalCenterIndex()
   {
      return globalCenterIndex;
   }

   public int getCenterIndex()
   {
      return centerIndex;
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

   public boolean isInitialized()
   {
      return initialized;
   }

   public TerrainMapStatistics getTerrainMapStatistics()
   {
      return terrainMapStatistics;
   }

   public RigidBodyTransform getCurrentGroundToWorldTransform()
   {
      return currentGroundToWorldTransform;
   }

   public boolean getComputeSnap()
   {
      return computeSteppability;
   }

   public void setModeSpherical()
   {
      mode = 0;
   }

   public void setModePerspective()
   {
      mode = 1;
   }

   public SteppableRegionCalculatorParametersBasics getSteppableRegionParameters()
   {
      return steppableRegionParameters;
   }
}
