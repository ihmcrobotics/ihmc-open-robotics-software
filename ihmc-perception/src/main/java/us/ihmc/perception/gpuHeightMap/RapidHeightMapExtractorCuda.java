package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.neural.HeightMapAutoencoder;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
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
import java.util.ArrayList;
import java.util.Objects;

import static org.bytedeco.cuda.global.cudart.*;
import static org.jcodec.common.Assert.assertEquals;

import us.ihmc.perception.camera.CameraIntrinsics;

public class RapidHeightMapExtractorCuda
{

   private GpuMat localHeightMapImage;
   private GpuMat globalHeightMapImage;
   private GpuMat globalHeightVarianceImage;
   private GpuMat terrainCostImage;
   private GpuMat contactMapImage;
   private GpuMat sensorCroppedHeightMapImage;
   private GpuMat sensorCroppedTerrainCostImage;
   private GpuMat sensorCroppedContactMapImage;

   private GpuMat steppabilityImage;
   private GpuMat snapHeightImage;
   private GpuMat snapNormalXImage;
   private GpuMat snapNormalYImage;
   private GpuMat snapNormalZImage;
   private GpuMat snappedAreaFractionImage;
   private GpuMat steppabilityConnectionsImage;

   private final SteppableRegionCalculatorParameters steppableRegionParameters = new SteppableRegionCalculatorParameters();
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
   private CUDAProgram program;
   private CUstream_st stream = CUDAStreamManager.getStream();
   private ByteBuffer snappingParametersBuffer;

   private GpuMat inputDepthImage;
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
   private PointerPointer<Pointer> groundToSensorTransformDevicePointerPointer;

   private FloatPointer sensorToGroundTransformHostPointer;
   private Pointer sensorToGroundTransformDevicePointer;
   private PointerPointer<Pointer> sensorToGroundTransformDevicePointerPointer;

   private FloatPointer worldToGroundTransformHostPointer;
   private Pointer worldToGroundTransformDevicePointer;
   private PointerPointer<Pointer> worldToGroundTransformDevicePointerPointer;

   private FloatPointer groundToWorldTransformHostPointer;
   private Pointer groundToWorldTransformDevicePointer;
   private PointerPointer<Pointer> groundToWorldTransformDevicePointerPointer;

   private FloatPointer parametersHostPointer;
   private Pointer parametersDevicePointer;
   private PointerPointer<Pointer> parametersDevicePointerPointer;

   private Pointer inputDepthImageHostPointer;
   private Pointer inputDepthImageDevicePointer;
   private Pointer inputDepthImageDevicePointerPointer;

   private Pointer localHeightMapImageHostPointer;
   private Pointer localHeightMapImageDevicePointer;
   private Pointer localHeightMapImageDevicePointerPointer;

   private Pointer globalHeightMapImageHostPointer;
   private Pointer globalHeightMapImageDevicePointer;
   private Pointer globalHeightMapImageDevicePointerPointer;

   private Pointer sensorCroppedHeightMapImageHostPointer;
   private Pointer sensorCroppedHeightMapImageDevicePointer;
   private Pointer sensorCroppedHeightMapImageDevicePointerPointer;

   private boolean processing = false;

   private dim3 blocksize = new dim3(16, 16, 1);
   private dim3 gridsize_kernel1 = new dim3(46, 81, 1);
   private dim3 gridsize_kernel2 = new dim3(11, 11, 1);
   private dim3 gridsize_kernel3 = new dim3(95, 95, 1);

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
      //TODO add a cuda manager and initialize it
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

   public void initialize() throws URISyntaxException

   {

      Path kernelPath = Path.of(Objects.requireNonNull(getClass().getResource("RapidHeightMapExtractor.cu")).toURI());
      program = new CUDAProgram(kernelPath, null);
      program.loadKernel("heightMapUpdateKernel");
      program.loadKernel("heightMapRegistrationKernel");
      program.loadKernel("croppingKernel");

      System.out.println("kernel loaded-------------------------------------------------------------------------------------------------------------------");

      recomputeDerivedParameters();

      cropWindowRectangle = new Rect((globalCellsPerAxis - heightMapParameters.getCropWindowSize()) / 2,
                                     (globalCellsPerAxis - heightMapParameters.getCropWindowSize()) / 2,
                                     heightMapParameters.getCropWindowSize(),
                                     heightMapParameters.getCropWindowSize());
      terrainMapData = new TerrainMapData(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());

      if (computeSteppability)
         snappingParametersBuffer = ByteBuffer.allocateDirect(1024);

      terrainMapData = new TerrainMapData(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());
      denoisedHeightMapImage = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);
      steppableRegionAssignmentMat = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);
      steppableRegionRingMat = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);

      localHeightMapImage = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_16UC1);
      localHeightMapImage.put(new Scalar(1));

      globalHeightMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);
      globalHeightMapImage.put(new Scalar(1));

      globalHeightVarianceImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      terrainCostImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      contactMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      sensorCroppedHeightMapImage = new GpuMat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);
      sensorCroppedTerrainCostImage = new GpuMat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);
      sensorCroppedContactMapImage = new GpuMat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);

      createSteppabilityMapImages(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());

      groundToSensorTransformHostPointer = new FloatPointer(16);
      groundToSensorTransformDevicePointer = new Pointer();
      groundToSensorTransformDevicePointerPointer = new PointerPointer<>(1L);

      sensorToGroundTransformHostPointer = new FloatPointer(16);
      sensorToGroundTransformDevicePointer = new Pointer();
      sensorToGroundTransformDevicePointerPointer = new PointerPointer<>(1L);

      worldToGroundTransformHostPointer = new FloatPointer(16);
      worldToGroundTransformDevicePointer = new Pointer();
      worldToGroundTransformDevicePointerPointer = new PointerPointer<>(1L);

      groundToWorldTransformHostPointer = new FloatPointer(16);
      groundToWorldTransformDevicePointer = new Pointer();
      groundToWorldTransformDevicePointerPointer = new PointerPointer<>(1L);

      parametersHostPointer = new FloatPointer(37);
      parametersDevicePointer = new Pointer();
      parametersDevicePointerPointer = new PointerPointer<>(1L);

      inputDepthImageHostPointer = new Pointer();
      inputDepthImageDevicePointer = new Pointer();
      inputDepthImageDevicePointerPointer = new PointerPointer<>(1L);

      localHeightMapImageHostPointer = new Pointer();
      localHeightMapImageDevicePointer = new Pointer();
      localHeightMapImageDevicePointerPointer = new PointerPointer<>(1L);

      globalHeightMapImageHostPointer = new Pointer();
      globalHeightMapImageDevicePointer = new Pointer();
      globalHeightMapImageDevicePointerPointer = new PointerPointer<>(1L);

      sensorCroppedHeightMapImageHostPointer = new Pointer();
      sensorCroppedHeightMapImageDevicePointer = new Pointer();
      sensorCroppedHeightMapImageDevicePointerPointer = new PointerPointer<>(1L);

      System.out.println(localCellsPerAxis + " = local-----------------------------------------------------------------------------------------------------------------");
      System.out.println(globalCellsPerAxis + "  = global-----------------------------------------------------------------------------------------------------------------");
      System.out.println(localHeightMapImage.sizeof() + " = localimage-----------------------------------------------------------------------------------------------------------------");
      System.out.println(globalHeightMapImage.sizeof() + "  = globalimage-----------------------------------------------------------------------------------------------------------------");
   }

   public void createSteppabilityMapImages(int height, int width)
   {
      steppabilityImage = new GpuMat(width, height, opencv_core.CV_8UC1);
      snapHeightImage = new GpuMat(width, height, opencv_core.CV_16UC1);
      snapNormalXImage = new GpuMat(width, height, opencv_core.CV_8UC1);
      snapNormalYImage = new GpuMat(width, height, opencv_core.CV_8UC1);
      snapNormalZImage = new GpuMat(width, height, opencv_core.CV_8UC1);
      snappedAreaFractionImage = new GpuMat(width, height, opencv_core.CV_8UC1);
      steppabilityConnectionsImage = new GpuMat(width, height, opencv_core.CV_8UC1);
   }

   public void reset()
   {
      System.out.println("in reset");

      double thicknessOfTheFoot = 0.02;
      double height = 0.0f;

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

      inputDepthImage = new GpuMat(depthImage.rows(), depthImage.cols(), depthImage.type());
      depthImage.copyTo(inputDepthImage);

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

   //
   public static HeightMapParameters getHeightMapParameters()
   {
      return heightMapParameters;
   }

   private void checkCudaError(int errorCode, String pointerName)
   {
      if (errorCode != cudaSuccess)
      {
         System.err.println("CUDA error on " + pointerName + ": " + cudaGetErrorString(errorCode));
         // Optionally, handle the error further, e.g., clean up, retry, or abort.
      }
   }

   //
   public void update(RigidBodyTransform sensorToWorldTransform, RigidBodyTransform sensorToGroundTransform, RigidBodyTransform groundToWorldTransform)
   {

      currentGroundToWorldTransform.set(groundToWorldTransform);
      currentSensorToWorldTransform.set(sensorToWorldTransform);
      sensorToGroundTransform.getTranslation().setZ(sensorToWorldTransform.getTranslationZ());

      RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
      groundToSensorTransform.invert();
      RigidBodyTransform worldToGroundTransform = new RigidBodyTransform(groundToWorldTransform);
      worldToGroundTransform.invert();

      sensorOrigin.set(sensorToWorldTransform.getTranslation());

      populateParameterBuffers(heightMapParameters, cameraIntrinsics, sensorOrigin);

      groundToSensorTransform.get(groundToSensorTransformArray);
      sensorToGroundTransform.get(sensorToGroundTransformArray);
      worldToGroundTransform.get(worldToGroundTransformArray);
      groundToWorldTransform.get(groundToWorldTransformArray);

      groundToSensorTransformHostPointer.put(groundToSensorTransformArray);
      sensorToGroundTransformHostPointer.put(sensorToGroundTransformArray);
      worldToGroundTransformHostPointer.put(worldToGroundTransformArray);
      groundToWorldTransformHostPointer.put(groundToWorldTransformArray);
      inputDepthImageHostPointer = inputDepthImage.ptr();
      globalHeightMapImageHostPointer = globalHeightMapImage.ptr();
      sensorCroppedHeightMapImageHostPointer = sensorCroppedHeightMapImage.ptr();



      checkCudaError(cudaMallocAsync(groundToSensorTransformDevicePointer, groundToSensorTransformArray.length * Float.BYTES, stream),
                     "groundToSensorTransformDevicePointer");
      checkCudaError(cudaMallocAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformArray.length * Float.BYTES, stream),
                     "sensorToGroundTransformDevicePointer");
      checkCudaError(cudaMallocAsync(worldToGroundTransformDevicePointer, worldToGroundTransformArray.length * Float.BYTES, stream),
                     "worldToGroundTransformDevicePointer");
      checkCudaError(cudaMallocAsync(groundToWorldTransformDevicePointer, groundToWorldTransformArray.length * Float.BYTES, stream),
                     "groundToWorldTransformDevicePointer");
      checkCudaError(cudaMallocAsync(parametersDevicePointer, parametersHostPointer.sizeof(), stream), "parametersDevicePointer");
      checkCudaError(cudaMallocAsync(inputDepthImageDevicePointer, inputDepthImage.sizeof(), stream), "inputDepthImageDevicePointer");
      checkCudaError(cudaMallocAsync(localHeightMapImageDevicePointer, inputDepthImageHostPointer.sizeof(), stream), "localHeightMapImageDevicePointer");
      checkCudaError(cudaMallocAsync(globalHeightMapImageDevicePointer, globalHeightMapImage.sizeof(), stream), "globalHeightMapImageDevicePointer");
      checkCudaError(cudaMallocAsync(sensorCroppedHeightMapImageDevicePointer, sensorCroppedHeightMapImage.sizeof(), stream), "sensorCroppedHeightMapImageDevicePointer");


      checkCudaError(cudaMemcpyAsync(groundToSensorTransformDevicePointer,
                                     groundToSensorTransformHostPointer,
                                     groundToSensorTransformHostPointer.sizeof(),
                                     cudaMemcpyHostToDevice,
                                     stream), "groundToSensorTransformDevicePointer copy");
      if (groundToSensorTransformDevicePointer == null)
      {
         System.err.println("groundToSensorTransformDevicePointer is null after allocation.");
      }

      checkCudaError(cudaMemcpyAsync(sensorToGroundTransformDevicePointer,
                                     sensorToGroundTransformHostPointer,
                                     sensorToGroundTransformHostPointer.sizeof(),
                                     cudaMemcpyHostToDevice,
                                     stream), "sensorToGroundTransformDevicePointer copy");
      if (sensorToGroundTransformDevicePointer == null)
      {
         System.err.println("sensorToGroundTransformDevicePointer is null after allocation.");
      }

      checkCudaError(cudaMemcpyAsync(worldToGroundTransformDevicePointer,
                                     worldToGroundTransformHostPointer,
                                     worldToGroundTransformHostPointer.sizeof(),
                                     cudaMemcpyHostToDevice,
                                     stream), "worldToGroundTransformDevicePointer copy");
      if (worldToGroundTransformDevicePointer == null)
      {
         System.err.println("worldToGroundTransformDevicePointer is null after allocation.");
      }

      checkCudaError(cudaMemcpyAsync(groundToWorldTransformDevicePointer,
                                     groundToWorldTransformHostPointer,
                                     groundToWorldTransformHostPointer.sizeof(),
                                     cudaMemcpyHostToDevice,
                                     stream), "groundToWorldTransformDevicePointer copy");
      if (groundToWorldTransformDevicePointer == null)
      {
         System.err.println("groundToWorldTransformDevicePointer is null after allocation.");
      }
      cudaStreamSynchronize(stream);


      checkCudaError(cudaMemcpyAsync(parametersDevicePointer, parametersHostPointer, parametersHostPointer.sizeof(), cudaMemcpyHostToDevice, stream),
                     "parametersDevicePointer copy");
      if (parametersDevicePointer == null)
      {
         System.err.println("parametersDevicePointer is null after allocation.");
      }
      cudaStreamSynchronize(stream);

      checkCudaError(cudaMemcpyAsync(inputDepthImageDevicePointer, inputDepthImageHostPointer, inputDepthImageHostPointer.sizeof(), cudaMemcpyHostToDevice, stream),
                     "inputDepthImageDevicePointer copy");
      if (inputDepthImageDevicePointer == null)
      {
         System.err.println("inputDepthImageDevicePointer is null after allocation.");
      }
      cudaStreamSynchronize(stream);

      checkCudaError(cudaMemcpyAsync(localHeightMapImageDevicePointer, localHeightMapImageHostPointer, localHeightMapImageDevicePointer.sizeof(), cudaMemcpyHostToDevice, stream),
                     "localHeightMapImageDevicePointer copy");
      if (inputDepthImageDevicePointer == null)
      {
         System.err.println("localHeightMapImageDevicePointer is null after allocation.");
      }
      cudaStreamSynchronize(stream);

      checkCudaError(cudaMemcpyAsync(globalHeightMapImageDevicePointer, globalHeightMapImageHostPointer, globalHeightMapImageDevicePointer.sizeof(), cudaMemcpyHostToDevice, stream),
                     "globalHeightMapImageDevicePointer copy");
      if (inputDepthImageDevicePointer == null)
      {
         System.err.println("globalHeightMapImageDevicePointer is null after allocation.");
      }
      cudaStreamSynchronize(stream);

      checkCudaError(cudaMemcpyAsync(sensorCroppedHeightMapImageDevicePointer, sensorCroppedHeightMapImageHostPointer, sensorCroppedHeightMapImageDevicePointer.sizeof(), cudaMemcpyHostToDevice, stream),
                     "sensorCroppedHeightMapImageDevicePointer copy");
      if (inputDepthImageDevicePointer == null)
      {
         System.err.println("sensorCroppedHeightMapImageDevicePointer is null after allocation.");
      }
      cudaStreamSynchronize(stream);

      groundToSensorTransformDevicePointerPointer.put(groundToSensorTransformDevicePointer);
      sensorToGroundTransformDevicePointerPointer.put(sensorToGroundTransformDevicePointer);
      worldToGroundTransformDevicePointerPointer.put(worldToGroundTransformDevicePointer);
      groundToWorldTransformDevicePointerPointer.put(groundToWorldTransformDevicePointer);
      parametersDevicePointerPointer.put(parametersDevicePointer);
      inputDepthImageDevicePointerPointer.put(inputDepthImageDevicePointer);
      localHeightMapImageDevicePointerPointer.put(localHeightMapImageDevicePointer);
      globalHeightMapImageDevicePointerPointer.put(globalHeightMapImageDevicePointer);
      sensorCroppedHeightMapImageDevicePointerPointer.put(sensorCroppedHeightMapImageDevicePointer);

      program.runKernel(stream, "heightMapUpdateKernel", gridsize_kernel1,
                        blocksize, 0, inputDepthImageDevicePointerPointer, localHeightMapImageDevicePointerPointer, parametersDevicePointerPointer, sensorToGroundTransformDevicePointerPointer, groundToSensorTransformDevicePointerPointer);
      program.runKernel(stream, "heightMapRegistrationKernel", gridsize_kernel2,
                        blocksize, 0, localHeightMapImageDevicePointerPointer, globalHeightMapImageDevicePointerPointer, parametersDevicePointerPointer, worldToGroundTransformDevicePointerPointer, sensorToGroundTransformDevicePointerPointer);
      program.runKernel(stream, "croppingKernel", gridsize_kernel1,
                        blocksize, 0, inputDepthImageDevicePointerPointer, sensorCroppedHeightMapImageDevicePointerPointer, parametersDevicePointerPointer);

      checkCudaError(cudaMemcpyAsync(sensorCroppedHeightMapImageHostPointer,sensorCroppedHeightMapImageDevicePointer, sensorCroppedHeightMapImageDevicePointer.sizeof(), cudaMemcpyDeviceToHost, stream),
                     "inputDepthImageDevicePointer copy");
      if (inputDepthImageDevicePointer == null)
      {
         System.err.println("inputDepthImageDevicePointer is null after allocation.");
      }
      cudaStreamSynchronize(stream);

      terrainMapData.setSensorOrigin(groundToWorldTransform.getTranslationX(), groundToWorldTransform.getTranslationY());

      Mat mat = new Mat(201, 201, opencv_core.CV_32F);
      sensorCroppedHeightMapImage.download(mat);

      terrainMapData.setHeightMap(mat);

   }

   public void destroy()
   {
      System.out.println("here destroy");

      groundToSensorTransformHostPointer.deallocate();
      cudaFreeAsync(groundToSensorTransformDevicePointer,stream);
      cudaFreeAsync(groundToSensorTransformDevicePointerPointer,stream);

      sensorToGroundTransformHostPointer.deallocate();
      cudaFreeAsync(sensorToGroundTransformDevicePointer,stream);
      cudaFreeAsync(sensorToGroundTransformDevicePointerPointer ,stream);

      worldToGroundTransformHostPointer.deallocate();
      cudaFreeAsync(worldToGroundTransformDevicePointer ,stream);
      cudaFreeAsync(worldToGroundTransformDevicePointerPointer,stream);

      groundToWorldTransformHostPointer.deallocate();
      cudaFreeAsync(groundToWorldTransformDevicePointer,stream);
      cudaFreeAsync(groundToWorldTransformDevicePointerPointer,stream);

      parametersHostPointer.deallocate();
      cudaFreeAsync(parametersDevicePointer,stream);
      cudaFreeAsync(parametersDevicePointerPointer,stream);

      inputDepthImageHostPointer.deallocate();
      cudaFreeAsync(inputDepthImageDevicePointer ,stream);
      cudaFreeAsync(inputDepthImageDevicePointerPointer,stream);

      localHeightMapImageHostPointer.deallocate();
      cudaFreeAsync(localHeightMapImageDevicePointer,stream);
      cudaFreeAsync(localHeightMapImageDevicePointerPointer,stream);

      globalHeightMapImageHostPointer.deallocate();
      cudaFreeAsync(globalHeightMapImageDevicePointer,stream);
      cudaFreeAsync(globalHeightMapImageDevicePointerPointer,stream);

      sensorCroppedHeightMapImageHostPointer.deallocate();
      cudaFreeAsync(sensorCroppedHeightMapImageDevicePointer,stream);
      cudaFreeAsync(sensorCroppedHeightMapImageDevicePointerPointer,stream);
   }

   public int getCenterIndex()
   {
      return centerIndex;
   }

   public void populateParameterBuffers(HeightMapParameters parameters, CameraIntrinsics cameraIntrinsics, Tuple3DReadOnly gridCenter)
   {

      parametersHostPointer.put(0, (float) parameters.getLocalCellSizeInMeters());
      parametersHostPointer.put(1, (float) centerIndex);
      parametersHostPointer.put(2, (float) cameraIntrinsics.getHeight());
      parametersHostPointer.put(3, (float) cameraIntrinsics.getWidth());
      parametersHostPointer.put(4, (float) gridCenter.getX());
      parametersHostPointer.put(5, (float) gridCenter.getY());
      parametersHostPointer.put(6, (float) mode);
      parametersHostPointer.put(7, (float) cameraIntrinsics.getCx());
      parametersHostPointer.put(8, (float) cameraIntrinsics.getCy());
      parametersHostPointer.put(9, (float) cameraIntrinsics.getFx());
      parametersHostPointer.put(10, (float) cameraIntrinsics.getFy());
      parametersHostPointer.put(11, (float) parameters.getGlobalCellSizeInMeters());
      parametersHostPointer.put(12, (float) globalCenterIndex);
      parametersHostPointer.put(13, (float) parameters.getRobotCollisionCylinderRadius());
      parametersHostPointer.put(14,  gridOffsetX);
      parametersHostPointer.put(15, (float) parameters.getHeightFilterAlpha());
      parametersHostPointer.put(16, (float) localCellsPerAxis);
      parametersHostPointer.put(17, (float) globalCellsPerAxis);
      parametersHostPointer.put(18, (float) parameters.getHeightScaleFactor());
      parametersHostPointer.put(19, (float) parameters.getMinHeightRegistration());
      parametersHostPointer.put(20, (float) parameters.getMaxHeightRegistration());
      parametersHostPointer.put(21, (float) parameters.getMinHeightDifference());
      parametersHostPointer.put(22, (float) parameters.getMaxHeightDifference());
      parametersHostPointer.put(23, (float) parameters.getSearchWindowHeight());
      parametersHostPointer.put(24, (float) parameters.getSearchWindowWidth());
      parametersHostPointer.put(25, (float) cropCenterIndex);
      parametersHostPointer.put(26, (float) parameters.getMinClampHeight());
      parametersHostPointer.put(27, (float) parameters.getMaxClampHeight());
      parametersHostPointer.put(28, (float) parameters.getHeightOffset());
      parametersHostPointer.put(29, (float) parameters.getSteppingCosineThreshold());
      parametersHostPointer.put(30, (float) parameters.getSteppingContactThreshold());
      parametersHostPointer.put(31, (float) parameters.getContactWindowSize());
      parametersHostPointer.put(32, (float) parameters.getSpatialAlpha());
      parametersHostPointer.put(33, (float) parameters.getSearchSkipSize());
      parametersHostPointer.put(34, (float) parameters.getVerticalSearchSize());
      parametersHostPointer.put(35, (float) parameters.getVerticalSearchResolution());
      parametersHostPointer.put(36, (float) parameters.getFastSearchSize());


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


