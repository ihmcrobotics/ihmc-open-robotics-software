package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.neural.HeightMapAutoencoder;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.net.URISyntaxException;
import java.net.URL;

import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

public class RapidHeightMapExtractorCUDA
{
   private GpuMat inputDepthImage;
   private GpuMat transformedInputDepthImage;
   private GpuMat localHeightMapImage;
   private GpuMat globalHeightMapImage;
   private GpuMat globalHeightVarianceImage;
   private GpuMat terrainCostImage;
   private GpuMat contactMapImage;
   private GpuMat sensorCroppedHeightMapImage;
   private GpuMat sensorCroppedTerrainCostImage;
   private GpuMat sensorCroppedContactMapImage;

   private boolean initialized = false;
   private int mode = 1; // 0 -> Ouster, 1 -> Realsense
   private float gridOffsetX;
   private int centerIndex;
   private int localCellsPerAxis;
   private int globalCenterIndex;
   private int cropCenterIndex;
   private int globalCellsPerAxis;
   private CameraIntrinsics cameraIntrinsics;

   private TerrainMapData terrainMapData;
   private static HeightMapParameters heightMapParameters = new HeightMapParameters("GPU");
   private final SideDependentList<ReferenceFrame> footSoleFrames = new SideDependentList<>();

   private Rect cropWindowRectangle;
   private static final boolean computeSteppability = true;
   private CUDAProgram heightMapCUDAProgram;
   private CUstream_st stream;

   private final RigidBodyTransform currentSensorToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform currentGroundToWorldTransform = new RigidBodyTransform();
   private final Point3D sensorOrigin = new Point3D();
   private final TerrainMapStatistics terrainMapStatistics = new TerrainMapStatistics();

   private HeightMapAutoencoder denoiser;
   private Mat denoisedHeightMapImage;
   private Mat steppableRegionAssignmentMat;
   private Mat steppableRegionRingMat;
   public int sequenceNumber = 0;

   private float[] worldToGroundTransformArray = new float[16];
   private float[] groundToWorldTransformArray = new float[16];
   private float[] groundToSensorTransformArray = new float[16];
   private float[] sensorToGroundTransformArray = new float[16];

   private FloatPointer groundToSensorTransformHostPointer;
   private FloatPointer groundToSensorTransformDevicePointer;

   private FloatPointer sensorToGroundTransformHostPointer;
   private FloatPointer sensorToGroundTransformDevicePointer;

   private FloatPointer worldToGroundTransformHostPointer;
   private FloatPointer worldToGroundTransformDevicePointer;

   private FloatPointer groundToWorldTransformHostPointer;
   private FloatPointer groundToWorldTransformDevicePointer;

   private FloatPointer parametersHostPointer;
   private FloatPointer parametersDevicePointer;

   private CUDAKernel updateKernel;
   private CUDAKernel registerKernel;
   private CUDAKernel croppingKernel;
   private CUDAKernel preprocessKernel;

   private dim3 blockSize;
   private dim3 gridSizeKernel0;
   private dim3 gridSizeKernel1;
   private dim3 gridSizeKernel2;
   private dim3 gridSizeKernel3;

   float[] paramsArray;

   private boolean processing = false;

   public void setModeSpherical()
   {
      mode = 0;
   }

   public void setModePerspective()
   {
      mode = 1;
   }

   public RapidHeightMapExtractorCUDA(ReferenceFrame leftFootSoleFrame, ReferenceFrame rightFootSoleFrame)
   {
      footSoleFrames.put(RobotSide.LEFT, leftFootSoleFrame);
      footSoleFrames.put(RobotSide.RIGHT, rightFootSoleFrame);
   }

   public void setDepthIntrinsics(CameraIntrinsics cameraIntrinsics)
   {
      this.cameraIntrinsics = cameraIntrinsics;
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

   public int getSequenceNumber()
   {
      return sequenceNumber;
   }

   public boolean isInitialized()
   {
      return initialized;
   }

   public TerrainMapData getTerrainMapData()
   {
      return terrainMapData;
   }

   public Point3D getSensorOrigin()
   {
      return sensorOrigin;
   }

   public static HeightMapParameters getHeightMapParameters()
   {
      return heightMapParameters;
   }

   private void deallocateFloatPointer(FloatPointer hostPointer, Pointer devicePointer)
   {
      if (hostPointer != null)
      {
         hostPointer.close();
         System.out.println("Deallocated host pointer.");
      }

      if (devicePointer != null)
      {
         devicePointer.close();
         System.out.println("Deallocated device pointer.");
      }
   }

   public void initialize() throws URISyntaxException
   {
      stream = new CUstream_st();
      cudart.cudaStreamCreate(stream);

      // Load CUDA kernels
      URL kernelPath = getClass().getResource("RapidHeightMapExtractor.cu");
      heightMapCUDAProgram = new CUDAProgram(kernelPath);

      String updateKernelName = "heightMapUpdateKernel";
      String registerKernelName = "heightMapRegistrationKernel";
      String croppingKernelName = "croppingKernel";
      String preprocessKernelName = "preprocessImageKernel";

      updateKernel = heightMapCUDAProgram.loadKernel(updateKernelName);
      registerKernel = heightMapCUDAProgram.loadKernel(registerKernelName);
      croppingKernel = heightMapCUDAProgram.loadKernel(croppingKernelName);
      preprocessKernel = heightMapCUDAProgram.loadKernel(preprocessKernelName);

      recomputeDerivedParameters();

      cropWindowRectangle = new Rect((globalCellsPerAxis - heightMapParameters.getCropWindowSize()) / 2,
                                     (globalCellsPerAxis - heightMapParameters.getCropWindowSize()) / 2,
                                     heightMapParameters.getCropWindowSize(),
                                     heightMapParameters.getCropWindowSize());

      terrainMapData = new TerrainMapData(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());

      // Initialize matrices and images
      denoisedHeightMapImage = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);
      steppableRegionAssignmentMat = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);
      steppableRegionRingMat = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);

      localHeightMapImage = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_16UC1);

      transformedInputDepthImage = new GpuMat(inputDepthImage.cols(), inputDepthImage.rows(), opencv_core.CV_16UC1);

      globalHeightMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);

      globalHeightVarianceImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);

      terrainCostImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);

      contactMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);

      sensorCroppedHeightMapImage = new GpuMat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);

      sensorCroppedTerrainCostImage = new GpuMat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);

      sensorCroppedContactMapImage = new GpuMat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);

      // Initialize transformation pointers
      groundToSensorTransformHostPointer = new FloatPointer(16);
      groundToSensorTransformDevicePointer = new FloatPointer();

      sensorToGroundTransformHostPointer = new FloatPointer(16);
      sensorToGroundTransformDevicePointer = new FloatPointer();

      worldToGroundTransformHostPointer = new FloatPointer(16);
      worldToGroundTransformDevicePointer = new FloatPointer();

      groundToWorldTransformHostPointer = new FloatPointer(16);
      groundToWorldTransformDevicePointer = new FloatPointer();

      parametersHostPointer = new FloatPointer(37);
      parametersDevicePointer = new FloatPointer();
   }

   public void reset()
   {
      double thicknessOfTheFoot = 0.02;
      double height = 0.0;

      if (footSoleFrames.sides().length == 2)
      {
         height = Math.min(footSoleFrames.get(RobotSide.LEFT).getTransformToWorldFrame().getTranslationZ(),
                           footSoleFrames.get(RobotSide.RIGHT).getTransformToWorldFrame().getTranslationZ()) - thicknessOfTheFoot;
      }
      int offset = (int) ((height + heightMapParameters.getHeightOffset()) * heightMapParameters.getHeightScaleFactor());

      localHeightMapImage.setTo(new Scalar(offset));
      globalHeightMapImage.setTo(new Scalar(offset));

      sequenceNumber = 0;
   }

   public void create(GpuMat depthImage, int mode)
   {
      inputDepthImage = depthImage;

      this.mode = mode;
      try
      {
         initialize();
      }
      catch (URISyntaxException e)
      {
         throw new RuntimeException(e);
      }
      reset();
   }

   public void update(RigidBodyTransform sensorToWorldTransform, RigidBodyTransform sensorToGroundTransform, RigidBodyTransform groundToWorldTransform)
   {
      //Update the current transforms to be used for further calculations
      currentGroundToWorldTransform.set(groundToWorldTransform);
      currentSensorToWorldTransform.set(sensorToWorldTransform);

      // Update the Z translation of the sensor to match the world transform (to handle the sensor's vertical position)
      sensorToGroundTransform.getTranslation().setZ(sensorToWorldTransform.getTranslationZ());

      // Compute the inverse transforms for later use
      RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
      groundToSensorTransform.invert();
      RigidBodyTransform worldToGroundTransform = new RigidBodyTransform(groundToWorldTransform);
      worldToGroundTransform.invert();

      //Store the sensor's origin for later use in parameter population
      sensorOrigin.set(sensorToWorldTransform.getTranslation());

      // Populate parameter buffers with the necessary values
      populateParameterBuffers(heightMapParameters, cameraIntrinsics, sensorOrigin);

      //Extract the transform arrays for memory transfer
      groundToSensorTransform.get(groundToSensorTransformArray);
      sensorToGroundTransform.get(sensorToGroundTransformArray);
      worldToGroundTransform.get(worldToGroundTransformArray);
      groundToWorldTransform.get(groundToWorldTransformArray);

      //Transfer the transform arrays to the host memory
      groundToSensorTransformHostPointer.put(groundToSensorTransformArray);
      sensorToGroundTransformHostPointer.put(sensorToGroundTransformArray);
      worldToGroundTransformHostPointer.put(worldToGroundTransformArray);
      groundToWorldTransformHostPointer.put(groundToWorldTransformArray);
      parametersHostPointer.put(paramsArray);

      //Allocate memory on the GPU for each of the transforms and images
      //This step involves allocating CUDA memory asynchronously, and it's important to check for allocation errors
      CUDATools.mallocAsync(groundToSensorTransformDevicePointer, groundToSensorTransformArray.length, stream);
      CUDATools.mallocAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformArray.length, stream);
      CUDATools.mallocAsync(worldToGroundTransformDevicePointer, worldToGroundTransformArray.length, stream);
      CUDATools.mallocAsync(groundToWorldTransformDevicePointer, groundToWorldTransformArray.length, stream);
      CUDATools.mallocAsync(parametersDevicePointer, paramsArray.length, stream);

      cudaStreamSynchronize(stream);

      //Copy the data from host memory to device memory asynchronously
      // This ensures the device has the latest data available for kernel processing
      CUDATools.memcpyAsync(groundToSensorTransformDevicePointer, groundToSensorTransformHostPointer, groundToSensorTransformArray.length, stream);
      CUDATools.memcpyAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformHostPointer, sensorToGroundTransformArray.length, stream);
      CUDATools.memcpyAsync(worldToGroundTransformDevicePointer, worldToGroundTransformHostPointer, worldToGroundTransformArray.length, stream);
      CUDATools.memcpyAsync(groundToWorldTransformDevicePointer, groundToWorldTransformHostPointer, groundToWorldTransformArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, paramsArray.length, stream);

      cudaStreamSynchronize(stream);

      //Execute the CUDA kernels with the provided stream
      //Each kernel performs a specific task related to the height map update, registration, and cropping

      int blockSizeXY = 32;
      int gridSizeKernel0X = (inputDepthImage.rows() + blockSizeXY - 1) / blockSizeXY;
      int gridSizeKernel0Y = (inputDepthImage.cols() + blockSizeXY - 1) / blockSizeXY;
      int gridSizeKernel1XY = (localCellsPerAxis + blockSizeXY - 1) / blockSizeXY;
      int gridSizeKernel2XY = (globalCellsPerAxis + blockSizeXY - 1) / blockSizeXY;
      int gridSizeKernel3XY = (heightMapParameters.getCropWindowSize() + blockSizeXY - 1) / blockSizeXY;

      blockSize = new dim3(blockSizeXY, blockSizeXY, 1);
      gridSizeKernel0 = new dim3(gridSizeKernel0X, gridSizeKernel0Y, 1);
      gridSizeKernel1 = new dim3(gridSizeKernel1XY, gridSizeKernel1XY, 1);
      gridSizeKernel2 = new dim3(gridSizeKernel2XY, gridSizeKernel2XY, 1);
      gridSizeKernel3 = new dim3(gridSizeKernel3XY, gridSizeKernel3XY, 1);

      preprocessKernel.withPointer(inputDepthImage.data()).withLong(inputDepthImage.step());
      preprocessKernel.withPointer(transformedInputDepthImage.data()).withLong(transformedInputDepthImage.step());

      updateKernel.withPointer(inputDepthImage.data()).withLong(inputDepthImage.step());
      updateKernel.withPointer(localHeightMapImage.data()).withLong(localHeightMapImage.step());
      updateKernel.withPointer(parametersDevicePointer);
      updateKernel.withPointer(sensorToGroundTransformDevicePointer);
      updateKernel.withPointer(groundToSensorTransformDevicePointer);
      updateKernel.withInt(localCellsPerAxis);

      registerKernel.withPointer(localHeightMapImage.data()).withLong(localHeightMapImage.step());
      registerKernel.withPointer(globalHeightMapImage.data()).withLong(globalHeightMapImage.step());
      registerKernel.withPointer(parametersDevicePointer);
      registerKernel.withPointer(worldToGroundTransformDevicePointer);
      registerKernel.withPointer(sensorToGroundTransformDevicePointer);

      croppingKernel.withPointer(globalHeightMapImage.data()).withLong(globalHeightMapImage.step());
      croppingKernel.withPointer(sensorCroppedHeightMapImage.data()).withLong(sensorCroppedHeightMapImage.step());
      croppingKernel.withPointer(parametersDevicePointer);
      croppingKernel.withInt(heightMapParameters.getCropWindowSize());
      cudaStreamSynchronize(stream);

      preprocessKernel.run(stream, gridSizeKernel0, blockSize, 0);
      updateKernel.run(stream, gridSizeKernel1, blockSize, 0);
      cudaStreamSynchronize(stream);
      registerKernel.run(stream, gridSizeKernel2, blockSize, 0);

      cudaStreamSynchronize(stream);

      //Update the terrain map data with the new results
      terrainMapData.setSensorOrigin(groundToWorldTransform.getTranslationX(), groundToWorldTransform.getTranslationY());

      croppingKernel.run(stream, gridSizeKernel3, blockSize, 0);

      cudaStreamSynchronize(stream);

      Mat finalCroppedHeightMap = new Mat();  // Assuming the height map is 201x201
      sensorCroppedHeightMapImage.download(finalCroppedHeightMap);  // Download the image from the GPU to the Mat object
      cudaStreamSynchronize(stream);
      terrainMapData.setHeightMap(finalCroppedHeightMap);

      // Tools to visualize
      // Does not work on the OCU so commented it out
      //        Mat inputMat = new Mat();
      //        inputDepthImage.download(inputMat);
      //        Mat transformedMat = new Mat();
      //        transformedInputDepthImage.download(transformedMat);
      //        Mat localMat = new Mat();
      //        localHeightMapImage.download(localMat);
      //        Mat globalMat = new Mat();
      //        globalHeightMapImage.download(globalMat);
      //        PerceptionDebugTools.display("Input Height Map", inputMat, 1);
      //        PerceptionDebugTools.display(" Transfomed Input Height Map", transformedMat, 1);
      //        PerceptionDebugTools.display("Local Height Map", localMat, 1);
      //        PerceptionDebugTools.display("Global Height Map", globalMat, 1);
      //        PerceptionDebugTools.display("Cropped Height Map", finalCroppedHeightMap, 1);
      //        inputDepthImage.download(finalCroppedHeightMap);
      //        Rect roi = new Rect(0, 0, 151, 151);
      //        Mat croppedMat = new Mat(finalCroppedHeightMap, roi);
      //        terrainMapData.setHeightMap(croppedMat);

   }

   public void destroy()
   {
      System.out.println("here destroy");

      heightMapCUDAProgram.close();
      updateKernel.close();
      registerKernel.close();
      croppingKernel.close();

      // At the end we have to destroy the stream to release the memory
      cudart.cudaStreamDestroy(stream);
      stream.close();

      // Clean up each resource
      deallocateFloatPointer(groundToSensorTransformHostPointer, groundToSensorTransformDevicePointer);
      deallocateFloatPointer(sensorToGroundTransformHostPointer, sensorToGroundTransformDevicePointer);
      deallocateFloatPointer(worldToGroundTransformHostPointer, worldToGroundTransformDevicePointer);
      deallocateFloatPointer(groundToWorldTransformHostPointer, groundToWorldTransformDevicePointer);
      deallocateFloatPointer(parametersHostPointer, parametersDevicePointer);

      blockSize.close();
      gridSizeKernel1.close();
      gridSizeKernel2.close();
      gridSizeKernel3.close();

      inputDepthImage.close();
      localHeightMapImage.close();
      globalHeightMapImage.close();
      globalHeightVarianceImage.close();
      terrainCostImage.close();
      contactMapImage.close();
      sensorCroppedHeightMapImage.close();
      sensorCroppedTerrainCostImage.close();
      sensorCroppedContactMapImage.close();
   }

   public int getCenterIndex()
   {
      return centerIndex;
   }

   public void populateParameterBuffers(HeightMapParameters parameters, CameraIntrinsics cameraIntrinsics, Tuple3DReadOnly gridCenter)
   {
      paramsArray = new float[] {(float) parameters.getLocalCellSizeInMeters(),
                                 (float) centerIndex,
                                 (float) cameraIntrinsics.getHeight(),
                                 (float) cameraIntrinsics.getWidth(),
                                 (float) gridCenter.getX(),
                                 (float) gridCenter.getY(),
                                 (float) mode,
                                 (float) cameraIntrinsics.getCx(),
                                 (float) cameraIntrinsics.getCy(),
                                 (float) cameraIntrinsics.getFx(),
                                 (float) cameraIntrinsics.getFy(),
                                 (float) parameters.getGlobalCellSizeInMeters(),
                                 (float) globalCenterIndex,
                                 (float) parameters.getRobotCollisionCylinderRadius(),
                                 gridOffsetX,
                                 (float) parameters.getHeightFilterAlpha(),
                                 (float) localCellsPerAxis,
                                 (float) globalCellsPerAxis,
                                 (float) parameters.getHeightScaleFactor(),
                                 (float) parameters.getMinHeightRegistration(),
                                 (float) parameters.getMaxHeightRegistration(),
                                 (float) parameters.getMinHeightDifference(),
                                 (float) parameters.getMaxHeightDifference(),
                                 (float) parameters.getSearchWindowHeight(),
                                 (float) parameters.getSearchWindowWidth(),
                                 (float) cropCenterIndex,
                                 (float) parameters.getMinClampHeight(),
                                 (float) parameters.getMaxClampHeight(),
                                 (float) parameters.getHeightOffset(),
                                 (float) parameters.getSteppingCosineThreshold(),
                                 (float) parameters.getSteppingContactThreshold(),
                                 (float) parameters.getContactWindowSize(),
                                 (float) parameters.getSpatialAlpha(),
                                 (float) parameters.getSearchSkipSize(),
                                 (float) parameters.getVerticalSearchSize(),
                                 (float) parameters.getVerticalSearchResolution(),
                                 (float) parameters.getFastSearchSize()};

      initialized = true;
   }

   public static HeightMapData packHeightMapData(RapidHeightMapExtractorCUDA heightMapExtractor, HeightMapData heightMapDataToPack)
   {
      Mat heightMapMat = heightMapExtractor.getTerrainMapData().getHeightMap();
      HeightMapData latestHeightMapData = heightMapDataToPack;
      if (latestHeightMapData == null)
      {
         latestHeightMapData = new HeightMapData((float) RapidHeightMapExtractorCUDA.getHeightMapParameters().getGlobalCellSizeInMeters(),
                                                 (float) RapidHeightMapExtractorCUDA.getHeightMapParameters().getGlobalWidthInMeters(),
                                                 heightMapExtractor.getSensorOrigin().getX(),
                                                 heightMapExtractor.getSensorOrigin().getY());
      }
      PerceptionMessageTools.convertToHeightMapData(heightMapMat,
                                                    latestHeightMapData,
                                                    heightMapExtractor.getSensorOrigin(),
                                                    (float) RapidHeightMapExtractorCUDA.getHeightMapParameters().getGlobalWidthInMeters(),
                                                    (float) RapidHeightMapExtractorCUDA.getHeightMapParameters().getGlobalCellSizeInMeters());

      return latestHeightMapData;
   }
}


