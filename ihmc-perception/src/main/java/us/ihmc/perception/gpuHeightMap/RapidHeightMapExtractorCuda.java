package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.LongPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.neural.HeightMapAutoencoder;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import org.bytedeco.opencv.opencv_core.Rect;
import us.ihmc.euclid.tuple3D.Point3D;
import org.bytedeco.opencv.opencv_core.Scalar;

import java.net.URISyntaxException;
import java.nio.ByteBuffer;
import java.nio.file.Path;
import java.util.Objects;

import static org.bytedeco.cuda.global.cudart.*;

import us.ihmc.perception.camera.CameraIntrinsics;

public class RapidHeightMapExtractorCuda
{
   private GpuMat inputDepthImage;
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
   //   private static final boolean computeSteppability = true;
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
   private Pointer groundToSensorTransformDevicePointer;

   private FloatPointer sensorToGroundTransformHostPointer;
   private Pointer sensorToGroundTransformDevicePointer;

   private FloatPointer worldToGroundTransformHostPointer;
   private Pointer worldToGroundTransformDevicePointer;

   private FloatPointer groundToWorldTransformHostPointer;
   private Pointer groundToWorldTransformDevicePointer;

   private FloatPointer parametersHostPointer;
   private Pointer parametersDevicePointer;

   private CUDAKernel updateKernel;
   private CUDAKernel registerKernel;
   private CUDAKernel croppingKernel;

   private dim3 blockSize;
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

   public RapidHeightMapExtractorCuda(ReferenceFrame leftFootSoleFrame, ReferenceFrame rightFootSoleFrame)
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

   private void checkCudaError(int errorCode, String pointerName)
   {
      if (errorCode != cudaSuccess)
      {
         System.err.println("CUDA error on " + pointerName + ": " + cudaGetErrorString(errorCode));
      }
   }

   private void allocateCudaMemory(Pointer devicePointer, long size, String pointerName)
   {
      checkCudaError(cudaMallocAsync(devicePointer, size, stream), pointerName + " allocation");
      if (devicePointer == null)
      {
         System.err.println(pointerName + " allocation failed.");
      }
   }

   private void copyToDeviceMemory(Pointer devicePointer, Pointer hostPointer, String pointerName)
   {
      checkCudaError(cudaMemcpyAsync(devicePointer, hostPointer, hostPointer.sizeof(), cudaMemcpyHostToDevice, stream), pointerName + " copy");
      if (devicePointer == null)
      {
         System.err.println(pointerName + " copy failed.");
      }
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
   //
   //   private void deallocatePointer(PointerPointer<Pointer> devicePointerPointer)
   //   {
   //      if (devicePointerPointer != null)
   //      {
   //         cudaFreeAsync(devicePointerPointer, stream);
   //         System.out.println("Deallocated device pointer pointer.");
   //      }
   //   }

   //   public void nehertest() throws URISyntaxException
   //   {
   //      System.out.println("IN TEST_____________________________________________________________________________________________________");
   //
   //      stream = CUDAStreamManager.getStream();
   //
   //      Path kernelPath = Path.of(Objects.requireNonNull(getClass().getResource("RapidHeightMapExtractor.cu")).toURI());
   //
   //      heightMapCUDAProgram = new CUDAProgram(kernelPath, null);
   //      heightMapCUDAProgram.loadKernel("heightMapUpdateKernel");
   //      heightMapCUDAProgram.loadKernel("heightMapRegistrationKernel");
   //      heightMapCUDAProgram.loadKernel("croppingKernel");
   //
   //      IntPointer sum1 = new IntPointer(1L);
   //      IntPointer deviceSum1 = new IntPointer();
   //      PointerPointer<Pointer> deviceSumPointer1 = new PointerPointer<>(1L);
   //
   //      IntPointer sum2 = new IntPointer(1L);
   //      IntPointer deviceSum2 = new IntPointer();
   //      PointerPointer<Pointer> deviceSumPointer2 = new PointerPointer<>(1L);
   //
   //      IntPointer sum3 = new IntPointer(1L);
   //      IntPointer deviceSum3 = new IntPointer();
   //      PointerPointer<Pointer> deviceSumPointer3 = new PointerPointer<>(1L);
   //
   //      cudaMallocAsync(deviceSum1, sum1.sizeof(), stream);
   //      deviceSumPointer1.put(deviceSum1);
   //
   //      cudaMallocAsync(deviceSum2, sum2.sizeof(), stream);
   //      deviceSumPointer2.put(deviceSum2);
   //
   //      cudaMallocAsync(deviceSum3, sum3.sizeof(), stream);
   //      deviceSumPointer3.put(deviceSum3);
   //
   //      heightMapCUDAProgram.runKernel(stream, "heightMapUpdateKernel", new dim3(1, 1, 1), new dim3(1, 1, 1), 0, deviceSumPointer1);
   //      heightMapCUDAProgram.runKernel(stream, "heightMapRegistrationKernel", new dim3(1, 1, 1), new dim3(1, 1, 1), 0, deviceSumPointer1, deviceSumPointer2);
   //      heightMapCUDAProgram.runKernel(stream, "croppingKernel", new dim3(1, 1, 1), new dim3(1, 1, 1), 0, deviceSumPointer2, deviceSumPointer3);
   //
   //      cudaStreamSynchronize(stream);
   //
   //      cudaMemcpyAsync(sum1, deviceSum1, deviceSum1.sizeof(), cudaMemcpyDefault, stream);
   //      cudaMemcpyAsync(sum2, deviceSum2, deviceSum2.sizeof(), cudaMemcpyDefault, stream);
   //      cudaMemcpyAsync(sum3, deviceSum3, deviceSum3.sizeof(), cudaMemcpyDefault, stream);
   //
   //      System.out.println(sum1.get() + " = 1 --------------------------------------------------------");
   //      System.out.println(sum2.get() + " = 1 --------------------------------------------------------");
   //      System.out.println(sum3.get() + " = 1 --------------------------------------------------------");
   //
   //      cudaFreeAsync(deviceSum1, stream);
   //      cudaFreeAsync(deviceSum2, stream);
   //      cudaFreeAsync(deviceSum3, stream);
   //
   //      heightMapCUDAProgram.destroy();
   //
   //      CUDAStreamManager.releaseStream(stream);
   //   }

   public void initialize() throws URISyntaxException

   {
      stream = new CUstream_st();
      cudart.cudaStreamCreate(stream);

      // Load CUDA kernels
      Path kernelPath = Path.of(Objects.requireNonNull(getClass().getResource("RapidHeightMapExtractor.cu")).toURI());
      heightMapCUDAProgram = new CUDAProgram(kernelPath, null);

      String updateKernelName = "heightMapUpdateKernel";
      String registerKernelName = "heightMapRegistrationKernel";
      String croppingKernelName = "croppingKernel";

      updateKernel = heightMapCUDAProgram.loadKernel(updateKernelName);
      registerKernel = heightMapCUDAProgram.loadKernel(registerKernelName);
      croppingKernel = heightMapCUDAProgram.loadKernel(croppingKernelName);

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

      globalHeightMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);

      globalHeightVarianceImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);

      terrainCostImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);

      contactMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);

      sensorCroppedHeightMapImage = new GpuMat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);

      sensorCroppedTerrainCostImage = new GpuMat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);

      sensorCroppedContactMapImage = new GpuMat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);

      // Initialize transformation pointers
      groundToSensorTransformHostPointer = new FloatPointer(16);
      groundToSensorTransformDevicePointer = new Pointer();

      sensorToGroundTransformHostPointer = new FloatPointer(16);
      sensorToGroundTransformDevicePointer = new Pointer();

      worldToGroundTransformHostPointer = new FloatPointer(16);
      worldToGroundTransformDevicePointer = new Pointer();

      groundToWorldTransformHostPointer = new FloatPointer(16);
      groundToWorldTransformDevicePointer = new Pointer();

      parametersHostPointer = new FloatPointer(37);
      parametersDevicePointer = new Pointer();
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

      localHeightMapImage.put(new Scalar(offset));
      globalHeightMapImage.put(new Scalar(offset));

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

      //      Allocate memory on the GPU for each of the transforms and images
      //      This step involves allocating CUDA memory asynchronously, and it's important to check for allocation errors
      allocateCudaMemory(groundToSensorTransformDevicePointer, groundToSensorTransformArray.length * Float.BYTES, "groundToSensorTransformDevicePointer");
      allocateCudaMemory(sensorToGroundTransformDevicePointer, sensorToGroundTransformArray.length * Float.BYTES, "sensorToGroundTransformDevicePointer");
      allocateCudaMemory(worldToGroundTransformDevicePointer, worldToGroundTransformArray.length * Float.BYTES, "worldToGroundTransformDevicePointer");
      allocateCudaMemory(groundToWorldTransformDevicePointer, groundToWorldTransformArray.length * Float.BYTES, "groundToWorldTransformDevicePointer");
      allocateCudaMemory(parametersDevicePointer, paramsArray.length * Float.BYTES, "parametersDevicePointer");

      cudaStreamSynchronize(stream);

      //Copy the data from host memory to device memory asynchronously
      // This ensures the device has the latest data available for kernel processing
      copyToDeviceMemory(groundToSensorTransformDevicePointer, groundToSensorTransformHostPointer, "groundToSensorTransformDevicePointer");
      copyToDeviceMemory(sensorToGroundTransformDevicePointer, sensorToGroundTransformHostPointer, "sensorToGroundTransformDevicePointer");
      copyToDeviceMemory(worldToGroundTransformDevicePointer, worldToGroundTransformHostPointer, "worldToGroundTransformDevicePointer");
      copyToDeviceMemory(groundToWorldTransformDevicePointer, groundToWorldTransformHostPointer, "groundToWorldTransformDevicePointer");
      copyToDeviceMemory(parametersDevicePointer, parametersHostPointer, "parametersDevicePointer");

      cudaStreamSynchronize(stream);

      //       Execute the CUDA kernels with the provided stream
      //       Each kernel performs a specific task related to the height map update, registration, and cropping

      int blockSizeXY = 32;
      int gridSizeKernel1X = (inputDepthImage.rows() + blockSizeXY - 1) / blockSizeXY;
      int gridSizeKernel1Y = (inputDepthImage.cols() + blockSizeXY - 1) / blockSizeXY;
      int gridSizeKernel2XY = (localCellsPerAxis + blockSizeXY - 1) / blockSizeXY;
      int gridSizeKernel3XY = (globalCellsPerAxis + blockSizeXY - 1) / blockSizeXY;

      blockSize = new dim3(blockSizeXY, blockSizeXY, 1);
      gridSizeKernel1 = new dim3(gridSizeKernel1X, gridSizeKernel1Y, 1);
      gridSizeKernel2 = new dim3(gridSizeKernel2XY, gridSizeKernel2XY, 1);
      gridSizeKernel3 = new dim3(gridSizeKernel3XY, gridSizeKernel3XY, 1);

      System.out.println();

      updateKernel.withPointer(inputDepthImage.data()).withLong(inputDepthImage.step());
      updateKernel.withPointer(localHeightMapImage.data()).withLong(localHeightMapImage.step());
      updateKernel.withPointer(parametersDevicePointer);
      updateKernel.withPointer(sensorToGroundTransformDevicePointer);
      updateKernel.withPointer(groundToSensorTransformDevicePointer);

      registerKernel.withPointer(localHeightMapImage.data()).withLong(localHeightMapImage.step());
      registerKernel.withPointer(globalHeightMapImage.data()).withLong(globalHeightMapImage.step());
      registerKernel.withPointer(parametersDevicePointer);
      registerKernel.withPointer(worldToGroundTransformDevicePointer);
      registerKernel.withPointer(sensorToGroundTransformDevicePointer);

      croppingKernel.withPointer(globalHeightMapImage.data()).withLong(globalHeightMapImage.step());
      croppingKernel.withPointer(sensorCroppedHeightMapImage.data()).withLong(sensorCroppedHeightMapImage.step());
      croppingKernel.withPointer(parametersDevicePointer);

      updateKernel.run(stream, gridSizeKernel1, blockSize, 0);
      registerKernel.run(stream, gridSizeKernel2, blockSize, 0);
      croppingKernel.run(stream, gridSizeKernel3, blockSize, 0);

      //Update the terrain map data with the new results
      terrainMapData.setSensorOrigin(groundToWorldTransform.getTranslationX(), groundToWorldTransform.getTranslationY());

      Mat finalCroppedHeightMap = new Mat();  // Assuming the height map is 201x201
      sensorCroppedHeightMapImage.download(finalCroppedHeightMap);  // Download the image from the GPU to the Mat object

      terrainMapData.setHeightMap(finalCroppedHeightMap);

      //      Rect roi = new Rect(0, 0, 201, 201);
      //       Download the final height map image into a OpenCV Mat object for further processing
      //      Mat croppedMat = new Mat(finalCroppedHeightMap, roi);
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

      //      deallocatePointer(inputDepthImageDevicePointerPointer);
      //      deallocatePointer(localHeightMapImageDevicePointerPointer);
      //      deallocatePointer(globalHeightMapImageDevicePointerPointer);
      //      deallocatePointer(sensorCroppedHeightMapImageDevicePointerPointer);
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

   public static HeightMapData packHeightMapData(RapidHeightMapExtractorCuda heightMapExtractor, HeightMapData heightMapDataToPack)
   {
      Mat heightMapMat = heightMapExtractor.getTerrainMapData().getHeightMap();
      HeightMapData latestHeightMapData = heightMapDataToPack;
      if (latestHeightMapData == null)
      {
         latestHeightMapData = new HeightMapData((float) RapidHeightMapExtractorCuda.getHeightMapParameters().getGlobalCellSizeInMeters(),
                                                 (float) RapidHeightMapExtractorCuda.getHeightMapParameters().getGlobalWidthInMeters(),
                                                 heightMapExtractor.getSensorOrigin().getX(),
                                                 heightMapExtractor.getSensorOrigin().getY());
      }
      PerceptionMessageTools.convertToHeightMapData(heightMapMat,
                                                    latestHeightMapData,
                                                    heightMapExtractor.getSensorOrigin(),
                                                    (float) RapidHeightMapExtractorCuda.getHeightMapParameters().getGlobalWidthInMeters(),
                                                    (float) RapidHeightMapExtractorCuda.getHeightMapParameters().getGlobalCellSizeInMeters());

      return latestHeightMapData;
   }
}


