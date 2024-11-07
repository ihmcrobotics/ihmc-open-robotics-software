package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jfree.chart.plot.dial.DialPointer;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.neural.HeightMapAutoencoder;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import org.bytedeco.opencv.opencv_core.Rect;
import us.ihmc.euclid.tuple3D.Point3D;

import java.io.IOException;
import java.io.InputStream;
import java.net.URISyntaxException;

import java.nio.ByteBuffer;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

import static org.bytedeco.cuda.global.cudart.*;
import static org.jcodec.common.Assert.assertEquals;

import us.ihmc.perception.camera.CameraIntrinsics;

public class RapidHeightMapExtractorCuda
{
   private int mode = 1; // 0 -> Ouster, 1 -> Realsense
   private float gridOffsetX;
   private int centerIndex;
   private int localCellsPerAxis;
   private int globalCenterIndex;
   private int cropCenterIndex;
   private int globalCellsPerAxis;
   private CameraIntrinsics cameraIntrinsics;

   ArrayList<Integer> parameterArray;
   private TerrainMapData terrainMapData;
   private static HeightMapParameters heightMapParameters = new HeightMapParameters("GPU");
   private final SideDependentList<ReferenceFrame> footSoleFrames = new SideDependentList<>();

   private Rect cropWindowRectangle;
   private static final boolean computeSteppability = true;
   private CUDAProgram program;
   private CUstream_st stream = CUDAStreamManager.getStream();
   private IntPointer a;
   private IntPointer b;
   private IntPointer sum;
   private IntPointer deviceSum;
   private PointerPointer<Pointer> deviceSumPointer;
   private ByteBuffer snappingParametersBuffer;

   private GpuMat inputDepthImage;
   private ByteBuffer parametersBuffer;
   private ByteBuffer groundToSensorTransformBuffer;
   private ByteBuffer sensorToGroundTransformBuffer;
   private ByteBuffer worldToGroundTransformBuffer;
   private ByteBuffer groundToWorldTransformBuffer;
   private ByteBuffer groundPlaneBuffer;

   private BytePointer parametersBufferPointer;
   private BytePointer sensorToGroundTransformBufferPointer;
   private BytePointer groundToSensorTransformBufferPointer;

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

   private GpuMat localHeightMapImage;
   private GpuMat createGlobalHeightMapImage;
   private GpuMat createGlobalHeightVarianceImage;
   private GpuMat createTerrainCostImage;
   private GpuMat createContactMapImage;
   private GpuMat createSensorCroppedHeightMapImage;
   private GpuMat createSensorCroppedTerrainCostImage;
   private GpuMat createSensorCroppedContactMapImage;

   private boolean processing = false;

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
      createGlobalHeightMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);
      createGlobalHeightVarianceImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      createTerrainCostImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      createContactMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      createSensorCroppedHeightMapImage = new GpuMat(heightMapParameters.getCropWindowSize(),
                                                            heightMapParameters.getCropWindowSize(),
                                                            opencv_core.CV_16UC1);
      createSensorCroppedTerrainCostImage = new GpuMat(heightMapParameters.getCropWindowSize(),
                                                              heightMapParameters.getCropWindowSize(),
                                                              opencv_core.CV_8UC1);
      createSensorCroppedContactMapImage = new GpuMat(heightMapParameters.getCropWindowSize(),
                                                             heightMapParameters.getCropWindowSize(),
                                                             opencv_core.CV_8UC1);

      createSteppabilityMapImages(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());

      groundToSensorTransformHostPointer = new FloatPointer(16);
      groundToSensorTransformDevicePointer = new Pointer();
      groundToSensorTransformDevicePointerPointer = new PointerPointer<>();

      sensorToGroundTransformHostPointer = new FloatPointer();
      sensorToGroundTransformDevicePointer = new Pointer();
      sensorToGroundTransformDevicePointerPointer = new PointerPointer<>();

      worldToGroundTransformHostPointer = new FloatPointer();
      worldToGroundTransformDevicePointer = new Pointer();
      worldToGroundTransformDevicePointerPointer = new PointerPointer<>();

      groundToWorldTransformHostPointer = new FloatPointer();
      groundToWorldTransformDevicePointer = new Pointer();
      groundToWorldTransformDevicePointerPointer = new PointerPointer<>();

      parametersHostPointer = new FloatPointer();
      parametersDevicePointer = new Pointer();
      parametersDevicePointerPointer = new PointerPointer<>();

      inputDepthImageHostPointer = new Pointer();
      inputDepthImageDevicePointer = new Pointer();
      inputDepthImageDevicePointerPointer = new Pointer();

      localHeightMapImageHostPointer = new Pointer();
      localHeightMapImageDevicePointer = new Pointer();
      localHeightMapImageDevicePointerPointer = new PointerPointer<>();




   }

   public void createSteppabilityMapImages(int height, int width)
   {
      GpuMat steppabilityImage = new GpuMat(width, height, opencv_core.CV_8UC1);
      GpuMat snapHeightImage = new GpuMat(width, height, opencv_core.CV_16UC1);
      GpuMat snapNormalXImage = new GpuMat(width, height, opencv_core.CV_8UC1);
      GpuMat snapNormalYImage = new GpuMat(width, height, opencv_core.CV_8UC1);
      GpuMat snapNormalZImage = new GpuMat(width, height, opencv_core.CV_8UC1);
      GpuMat snappedAreaFractionImage = new GpuMat(width, height, opencv_core.CV_8UC1);
      GpuMat steppabilityConnectionsImage = new GpuMat(width, height, opencv_core.CV_8UC1);


   }

   public void reset()
   {
      System.out.println("in reset");

      // Copy result from device to host

      //      double thicknessOfTheFoot = 0.02;
      //      double height = 0.0f;
      //
      //      if (footSoleFrames.sides().length == 2)
      //      {
      //         height = Math.min(footSoleFrames.get(RobotSide.LEFT).getTransformToWorldFrame().getTranslationZ(),
      //                           footSoleFrames.get(RobotSide.RIGHT).getTransformToWorldFrame().getTranslationZ()) - thicknessOfTheFoot;
      //      }
      //      int offset = (int) ((height + heightMapParameters.getHeightOffset()) * heightMapParameters.getHeightScaleFactor());
      //      localHeightMapImage.getBytedecoOpenCVMat().put(new Scalar(offset));
      //      globalHeightMapImage.getBytedecoOpenCVMat().put(new Scalar(offset));
      //      CudaHeightMapTools.BytedecoImageToPointer(localHeightMapImage,localHeightMapImagePointer);
      //      CudaHeightMapTools.BytedecoImageToPointer(globalHeightMapImage,globalHeightMapImagePointer);
      //
      //
      //
      //      if (computeSteppability)
      //      {
      //         snapHeightImage.getBytedecoOpenCVMat().put(new Scalar(32768));
      //         CudaHeightMapTools.BytedecoImageToPointer(snapHeightImage,snapHeightImagePointer);
      //      }
      //
      //      sequenceNumber = 0;
      //
   }

   public void create(GpuMat depthImage, int mode)
   {
      this.inputDepthImage = depthImage;
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

   //
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

         RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
         groundToSensorTransform.invert();

         RigidBodyTransform worldToGroundTransform = new RigidBodyTransform(groundToWorldTransform);
         worldToGroundTransform.invert();


         sensorOrigin.set(sensorToWorldTransform.getTranslation());

         populateParameterBuffers(heightMapParameters, cameraIntrinsics, sensorOrigin);

         groundToSensorTransform.get(groundToSensorTransformArray);

         groundToSensorTransformHostPointer.put(groundToSensorTransformArray);

         cudaMallocAsync(groundToSensorTransformDevicePointer, groundToSensorTransformArray.length * Float.BYTES, stream);
         groundToSensorTransformDevicePointerPointer.put(groundToSensorTransformDevicePointer);
         cudaMemcpyAsync(groundToSensorTransformDevicePointer, groundToSensorTransformHostPointer, groundToSensorTransformHostPointer.sizeof(), cudaMemcpyHostToDevice, stream);

         sensorToGroundTransform.get(sensorToGroundTransformArray);
         sensorToGroundTransformHostPointer.put(sensorToGroundTransformArray);
         cudaMallocAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformArray.length * Float.BYTES, stream);
         sensorToGroundTransformDevicePointerPointer.put(sensorToGroundTransformDevicePointer);
         cudaMemcpyAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformHostPointer, sensorToGroundTransformHostPointer.sizeof(), cudaMemcpyHostToDevice, stream);

         worldToGroundTransform.get(worldToGroundTransformArray);
         System.out.println("neherrrrrrrrrrrrrrrrrrrrrr--------------------------------------------");
         System.out.println(worldToGroundTransformArray+ "-----------------------------------------------------------------------------------");
         worldToGroundTransformHostPointer.put(worldToGroundTransformArray);
         cudaMallocAsync(worldToGroundTransformDevicePointer, worldToGroundTransformArray.length * Float.BYTES, stream);
         worldToGroundTransformDevicePointerPointer.put(worldToGroundTransformDevicePointer);
         cudaMemcpyAsync(worldToGroundTransformDevicePointer, worldToGroundTransformHostPointer, worldToGroundTransformHostPointer.sizeof(), cudaMemcpyHostToDevice, stream);

         groundToWorldTransform.get(groundToWorldTransformArray);
         groundToWorldTransformHostPointer.put(groundToWorldTransformArray);
         cudaMallocAsync(groundToWorldTransformDevicePointer, groundToWorldTransformArray.length * Float.BYTES, stream);
         groundToWorldTransformDevicePointerPointer.put(groundToWorldTransformDevicePointer);
         cudaMemcpyAsync(groundToWorldTransformDevicePointer, groundToWorldTransformHostPointer, groundToWorldTransformHostPointer.sizeof(), cudaMemcpyHostToDevice, stream);

         for (int i = 0; i < 39; i++)
         {
            parametersHostPointer.put(parametersBuffer.getFloat(i));
         }


         cudaMallocAsync(parametersDevicePointer, 152, stream);
         parametersDevicePointerPointer.put(parametersDevicePointer);

         inputDepthImageHostPointer = inputDepthImage.data();
         cudaMallocAsync(inputDepthImageDevicePointer, inputDepthImageDevicePointer.sizeof(), stream);
         inputDepthImageDevicePointerPointer.put(inputDepthImageDevicePointer);
         cudaMemcpyAsync(inputDepthImageDevicePointer, inputDepthImageHostPointer, inputDepthImageHostPointer.sizeof(), cudaMemcpyHostToDevice, stream);

         cudaMallocAsync(localHeightMapImageDevicePointer, localHeightMapImageDevicePointer.sizeof(), stream);
         localHeightMapImageDevicePointerPointer.put(localHeightMapImageDevicePointer);
         cudaMemcpyAsync(localHeightMapImageDevicePointer, localHeightMapImageHostPointer, localHeightMapImageHostPointer.sizeof(), cudaMemcpyHostToDevice, stream);

         program.runKernel(stream, "add_vector", new dim3(1, 1, 1), new dim3(inputDepthImageHostPointer.sizeof(), 1, 1), 0, inputDepthImageDevicePointerPointer, localHeightMapImageDevicePointerPointer, parametersDevicePointerPointer,sensorToGroundTransformDevicePointerPointer,groundToSensorTransformDevicePointerPointer);



      }}

      public void destroy () {
      System.out.println("here destroy");
   }

      public void populateParameterBuffers (HeightMapParameters parameters, CameraIntrinsics cameraIntrinsics, Tuple3DReadOnly gridCenter)
      {
         parametersBuffer = ByteBuffer.allocateDirect(152);
         //// Fill parameters buffer
         parametersBuffer.putFloat((float) parameters.getLocalCellSizeInMeters());
         parametersBuffer.putFloat(centerIndex);
         parametersBuffer.putFloat((float) cameraIntrinsics.getHeight());
         parametersBuffer.putFloat((float) cameraIntrinsics.getWidth());
         parametersBuffer.putFloat((float) gridCenter.getX());
         parametersBuffer.putFloat((float) gridCenter.getY());
         parametersBuffer.putFloat((float) mode);
         parametersBuffer.putFloat((float) cameraIntrinsics.getCx());
         parametersBuffer.putFloat((float) cameraIntrinsics.getCy());
         parametersBuffer.putFloat((float) cameraIntrinsics.getFx());
         parametersBuffer.putFloat((float) cameraIntrinsics.getFy());
         parametersBuffer.putFloat((float) parameters.getGlobalCellSizeInMeters());
         parametersBuffer.putFloat((float) globalCenterIndex);
         parametersBuffer.putFloat((float) parameters.getRobotCollisionCylinderRadius());
         parametersBuffer.putFloat(gridOffsetX);
         parametersBuffer.putFloat((float) parameters.getHeightFilterAlpha());
         parametersBuffer.putFloat(localCellsPerAxis);
         parametersBuffer.putFloat(globalCellsPerAxis);
         parametersBuffer.putFloat((float) parameters.getHeightScaleFactor());
         parametersBuffer.putFloat((float) parameters.getMinHeightRegistration());
         parametersBuffer.putFloat((float) parameters.getMaxHeightRegistration());
         parametersBuffer.putFloat((float) parameters.getMinHeightDifference());
         parametersBuffer.putFloat((float) parameters.getMaxHeightDifference());
         parametersBuffer.putFloat((float) parameters.getSearchWindowHeight());
         parametersBuffer.putFloat((float) parameters.getSearchWindowWidth());
         parametersBuffer.putFloat((float) cropCenterIndex);
         parametersBuffer.putFloat((float) parameters.getMinClampHeight());
         parametersBuffer.putFloat((float) parameters.getMaxClampHeight());
         parametersBuffer.putFloat((float) parameters.getHeightOffset());
         parametersBuffer.putFloat((float) parameters.getSteppingCosineThreshold());
         parametersBuffer.putFloat((float) parameters.getSteppingContactThreshold());
         parametersBuffer.putFloat((float) parameters.getContactWindowSize());
         parametersBuffer.putFloat((float) parameters.getSpatialAlpha());
         parametersBuffer.putFloat((float) parameters.getSearchSkipSize());
         parametersBuffer.putFloat((float) parameters.getVerticalSearchSize());
         parametersBuffer.putFloat((float) parameters.getVerticalSearchResolution());
         parametersBuffer.putFloat((float) parameters.getFastSearchSize());
      }
   }

