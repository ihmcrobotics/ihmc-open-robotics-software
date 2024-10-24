package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.PointerPointer;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import org.bytedeco.opencv.opencv_core.Rect;
import us.ihmc.euclid.tuple3D.Point3D;

import java.util.ArrayList;

import static org.bytedeco.cuda.global.cudart.*;
import static org.jcodec.common.Assert.assertEquals;

//package us.ihmc.perception.gpuHeightMap;
//
//import org.bytedeco.hdf5.H5FD_file_image_callbacks_t.Image_malloc_long_int_Pointer;
//import org.bytedeco.javacpp.PointerPointer;
//import org.bytedeco.opencl.global.OpenCL;
//import org.bytedeco.opencv.global.opencv_core;
//import org.bytedeco.opencv.opencv_core.Mat;
//import org.bytedeco.opencv.opencv_core.Rect;
//import org.bytedeco.opencv.opencv_core.Scalar;
//import us.ihmc.euclid.referenceFrame.ReferenceFrame;
//import us.ihmc.euclid.transform.RigidBodyTransform;
//import us.ihmc.euclid.tuple3D.Point3D;
//import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
//import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.camera.CameraIntrinsics;
//import us.ihmc.perception.cuda.CUDAProgram;
//import us.ihmc.perception.cuda.CudaHeightMapTools;
//import us.ihmc.perception.heightMap.TerrainMapData;
//import us.ihmc.perception.opencl.OpenCLFloatBuffer;
//import us.ihmc.perception.opencl.OpenCLFloatParameters;
//import us.ihmc.perception.opencl.OpenCLManager;
//import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
//import us.ihmc.perception.tools.PerceptionMessageTools;
//import us.ihmc.robotics.robotSide.RobotSide;
//import us.ihmc.robotics.robotSide.SideDependentList;
//import us.ihmc.sensorProcessing.heightMap.HeightMapData;
//import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
//import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
//
//import java.util.ArrayList;
//
//import static com.esotericsoftware.jsonbeans.JsonValue.ValueType.array;
//
public class RapidHeightMapExtractorCuda
{
   public int sequenceNumber = 0;
   private int mode = 1; // 0 -> Ouster, 1 -> Realsense
   private float gridOffsetX;
   private int centerIndex;
   private int localCellsPerAxis;
   private int globalCenterIndex;
   private int cropCenterIndex;
   private int globalCellsPerAxis;
      private CameraIntrinsics cameraIntrinsics;
   private final Point3D sensorOrigin = new Point3D();

   //   private static final boolean computeSteppability = true;
   //   private float[] worldToGroundTransformArray = new float[16];
   //   private float[] groundToWorldTransformArray = new float[16];
   //   private float[] groundToSensorTransformArray = new float[16];
   private float[] sensorToGroundTransformArray = new float[16];
   ArrayList<Integer> parameterArray;
   //   ArrayList<Integer> snappingParametersArray;
   //   private PointerPointer worldToGroundTransformPointer;
   //   private PointerPointer sensorToGroundTransformPointer;
   //   private PointerPointer groundToSensorTransformPointer;
   //   private PointerPointer groundToWorldTransformPointer;
   //
   //   private boolean initialized = false;
   //   private boolean modified = true;
   //   private boolean processing = false;
   //   private boolean heightMapDataAvailable = false;
      private TerrainMapData terrainMapData;
   //   private Mat denoisedHeightMapImage;
   //   private Mat steppableRegionAssignmentMat;
   //   private Mat steppableRegionRingMat;
   //   public int sequenceNumber = 0;
   //
   //   private BytedecoImage localHeightMapImage;
   //   private Image_malloc_long_int_Pointer localHeightMapImagePointer;
   //   private BytedecoImage globalHeightMapImage;
   //   private Image_malloc_long_int_Pointer globalHeightMapImagePointer;
   //   private BytedecoImage globalHeightVarianceImage;
   //   private Image_malloc_long_int_Pointer globalHeightVarianceImagePointer;
   //   private BytedecoImage terrainCostImage;
   //   private Image_malloc_long_int_Pointer terrainCostImagePointer;
   //   private BytedecoImage contactMapImage;
   //   private BytedecoImage steppabilityImage;
   //   private BytedecoImage snapHeightImage;
   //   private BytedecoImage snapNormalXImage;
   //   private BytedecoImage snapNormalYImage;
   //   private BytedecoImage snapNormalZImage;
   //   private BytedecoImage steppabilityConnectionsImage;
   //   private BytedecoImage snappedAreaFractionImage;
   //   private BytedecoImage sensorCroppedHeightMapImage;
   //   private BytedecoImage sensorCroppedTerrainCostImage;
   //   private BytedecoImage sensorCroppedContactMapImage;
   //   private Image_malloc_long_int_Pointer contactMapImagePointer;
   //   private Image_malloc_long_int_Pointer steppabilityImagePointer;
   //   private Image_malloc_long_int_Pointer snapHeightImagePointer;
   //   private Image_malloc_long_int_Pointer snapNormalXImagePointer;
   //   private Image_malloc_long_int_Pointer snapNormalYImagePointer;
   //   private Image_malloc_long_int_Pointer snapNormalZImagePointer;
   //   private Image_malloc_long_int_Pointer snappedAreaFractionImagePointer;
   //   private Image_malloc_long_int_Pointer steppabilityConnectionsImagePointer;
   //   private Image_malloc_long_int_Pointer sensorCroppedHeightMapImagePointer;
   //   private Image_malloc_long_int_Pointer sensorCroppedTerrainCostImagePointer;
   //   private Image_malloc_long_int_Pointer sensorCroppedContactMapImagePointer;
   //   private Image_malloc_long_int_Pointer inputDepthImagePointer;
   //
   //   private final SteppableRegionCalculatorParameters steppableRegionParameters = new SteppableRegionCalculatorParameters();
   //
   private static HeightMapParameters heightMapParameters = new HeightMapParameters("GPU");
   //   private final RigidBodyTransform currentSensorToWorldTransform = new RigidBodyTransform();
   //   private final RigidBodyTransform currentGroundToWorldTransform = new RigidBodyTransform();
   //   private final Point3D sensorOrigin = new Point3D();
   //   private final TerrainMapStatistics terrainMapStatistics = new TerrainMapStatistics();
   private BytedecoImage inputDepthImage;
   private Rect cropWindowRectangle;
   //   private HeightMapAutoencoder denoiser;
   private final SideDependentList<ReferenceFrame> footSoleFrames = new SideDependentList<>();

   //   // TODO add a cuda manager
   //
   //
   //
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
   public void initialize()
   {
      recomputeDerivedParameters();
      cropWindowRectangle = new Rect((globalCellsPerAxis - heightMapParameters.getCropWindowSize()) / 2,
                                     (globalCellsPerAxis - heightMapParameters.getCropWindowSize()) / 2,
                                     heightMapParameters.getCropWindowSize(),
                                     heightMapParameters.getCropWindowSize());
      //
      parameterArray = new ArrayList<>();
//      if (computeSteppability)
//         snappingParametersArray = new ArrayList<>();
      //
      ////      CudaHeightMapTools.TransformToPointer(groundToSensorTransformArray,groundToSensorTransformPointer);
      ////      CudaHeightMapTools.TransformToPointer(sensorToGroundTransformArray,sensorToGroundTransformPointer);
      ////      CudaHeightMapTools.TransformToPointer(worldToGroundTransformArray,worldToGroundTransformPointer);
      ////      CudaHeightMapTools.TransformToPointer(groundToSensorTransformArray,groundToSensorTransformPointer);
      //
      //      //TODO groundPlaneBuffer
      //
            terrainMapData = new TerrainMapData(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());
      //      denoisedHeightMapImage = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);
      //      steppableRegionAssignmentMat = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);
      //      steppableRegionRingMat = new Mat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);
      //
      //      localHeightMapImage = new BytedecoImage(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_16UC1);
      ////      CudaHeightMapTools.BytedecoImageToPointer(localHeightMapImage,localHeightMapImagePointer);
      //      globalHeightMapImage = new BytedecoImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);
      ////      CudaHeightMapTools.BytedecoImageToPointer(globalHeightMapImage,globalHeightMapImagePointer);
      //      globalHeightVarianceImage = new BytedecoImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      //      CudaHeightMapTools.BytedecoImageToPointer(globalHeightVarianceImage,globalHeightVarianceImagePointer);
      //      terrainCostImage = new BytedecoImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      //      CudaHeightMapTools.BytedecoImageToPointer(terrainCostImage,terrainCostImagePointer);
      //      contactMapImage = new BytedecoImage(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
      //      CudaHeightMapTools.BytedecoImageToPointer(contactMapImage,contactMapImagePointer);
      //
      //      createSteppabilityMapImages(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());
      //
      //
      //
      //      sensorCroppedHeightMapImage = new BytedecoImage(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);
      //      CudaHeightMapTools.BytedecoImageToPointer(sensorCroppedHeightMapImage,sensorCroppedHeightMapImagePointer);
      //      sensorCroppedTerrainCostImage= new BytedecoImage(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);
      //      CudaHeightMapTools.BytedecoImageToPointer(sensorCroppedTerrainCostImage,sensorCroppedTerrainCostImagePointer);
      //      sensorCroppedContactMapImage= new BytedecoImage(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_8UC1);
      //      CudaHeightMapTools.BytedecoImageToPointer(sensorCroppedContactMapImage,sensorCroppedContactMapImagePointer);
      //
      //      CUDAProgram rapidHeightMapUpdaterProgram = new CUDAProgram("RapidHeightMapExtractor", stringName);
      //      rapidHeightMapUpdaterProgram.loadKernel("heightMapUpdateKernel");
      //      rapidHeightMapUpdaterProgram.loadKernel("heightMapRegistrationKernel");
      //      rapidHeightMapUpdaterProgram.loadKernel("terrainCostKernel");
      //      rapidHeightMapUpdaterProgram.loadKernel("contactMapKernel");
      //      rapidHeightMapUpdaterProgram.loadKernel("croppingKernel");
      //
      //      if (computeSteppability)
      //      {
      //         rapidHeightMapUpdaterProgram.loadKernel("computeSnappedValuesKernel");
      //         rapidHeightMapUpdaterProgram.loadKernel("computeSteppabilityConnectionsKernel");
      //      }
      //
      //
      //   }
      //   public void createSteppabilityMapImages(int height, int width)
      //   {
      //      steppabilityImage = new BytedecoImage(width, height, opencv_core.CV_8UC1);
      //      snapHeightImage = new BytedecoImage(width, height, opencv_core.CV_16UC1);
      //      snapNormalXImage = new BytedecoImage(width, height, opencv_core.CV_8UC1);
      //      snapNormalYImage = new BytedecoImage(width, height, opencv_core.CV_8UC1);
      //      snapNormalZImage = new BytedecoImage(width, height, opencv_core.CV_8UC1);
      //      snappedAreaFractionImage = new BytedecoImage(width, height, opencv_core.CV_8UC1);
      //      steppabilityConnectionsImage = new BytedecoImage(width, height, opencv_core.CV_8UC1);
      //
      //       CudaHeightMapTools.BytedecoImageToPointer(steppabilityImage,steppabilityImagePointer);
      //       //CudaHeightMapTools.BytedecoImageToPointer(snapHeightImage,snapHeightImagePointer);
      //       CudaHeightMapTools.BytedecoImageToPointer(snapNormalXImage,snapNormalXImagePointer);
      //       CudaHeightMapTools.BytedecoImageToPointer(snapNormalYImage,snapNormalYImagePointer);
      //       CudaHeightMapTools.BytedecoImageToPointer(snapNormalZImage,snapNormalZImagePointer);
      //       CudaHeightMapTools.BytedecoImageToPointer(snappedAreaFractionImage,snappedAreaFractionImagePointer);
      //       CudaHeightMapTools.BytedecoImageToPointer(steppabilityConnectionsImage,steppabilityConnectionsImagePointer);
         }
         public void reset()
         {
            System.out.println("in reset");
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


      public void create(BytedecoImage depthImage, int mode)
      {
         this.inputDepthImage = depthImage;
         this.mode = mode;
         System.out.println("here");
//
//         initialize();
//         reset();
      }


      //
      //   //TODO
      //   public void populateParameterBuffers(HeightMapParameters parameters, CameraIntrinsics cameraIntrinsics, Tuple3DReadOnly gridCenter)
      //   {
      //      //// Fill parameters buffer
      //      parametersBuffer.setParameter((float) parameters.getLocalCellSizeInMeters());
      //      parametersBuffer.setParameter(centerIndex);
      //      parametersBuffer.setParameter((float) cameraIntrinsics.getHeight());
      //      parametersBuffer.setParameter((float) cameraIntrinsics.getWidth());
      //      parametersBuffer.setParameter((float) gridCenter.getX());
      //      parametersBuffer.setParameter((float) gridCenter.getY());
      //      parametersBuffer.setParameter((float) mode);
      //      parametersBuffer.setParameter((float) cameraIntrinsics.getCx());
      //      parametersBuffer.setParameter((float) cameraIntrinsics.getCy());
      //      parametersBuffer.setParameter((float) cameraIntrinsics.getFx());
      //      parametersBuffer.setParameter((float) cameraIntrinsics.getFy());
      //      parametersBuffer.setParameter((float) parameters.getGlobalCellSizeInMeters());
      //      parametersBuffer.setParameter((float) globalCenterIndex);
      //      parametersBuffer.setParameter((float) parameters.getRobotCollisionCylinderRadius());
      //      parametersBuffer.setParameter(gridOffsetX);
      //      parametersBuffer.setParameter((float) parameters.getHeightFilterAlpha());
      //      parametersBuffer.setParameter(localCellsPerAxis);
      //      parametersBuffer.setParameter(globalCellsPerAxis);
      //      parametersBuffer.setParameter((float) parameters.getHeightScaleFactor());
      //      parametersBuffer.setParameter((float) parameters.getMinHeightRegistration());
      //      parametersBuffer.setParameter((float) parameters.getMaxHeightRegistration());
      //      parametersBuffer.setParameter((float) parameters.getMinHeightDifference());
      //      parametersBuffer.setParameter((float) parameters.getMaxHeightDifference());
      //      parametersBuffer.setParameter((float) parameters.getSearchWindowHeight());
      //      parametersBuffer.setParameter((float) parameters.getSearchWindowWidth());
      //      parametersBuffer.setParameter((float) cropCenterIndex);
      //      parametersBuffer.setParameter((float) parameters.getMinClampHeight());
      //      parametersBuffer.setParameter((float) parameters.getMaxClampHeight());
      //      parametersBuffer.setParameter((float) parameters.getHeightOffset());
      //      parametersBuffer.setParameter((float) parameters.getSteppingCosineThreshold());
      //      parametersBuffer.setParameter((float) parameters.getSteppingContactThreshold());
      //      parametersBuffer.setParameter((float) parameters.getContactWindowSize());
      //      parametersBuffer.setParameter((float) parameters.getSpatialAlpha());
      //      parametersBuffer.setParameter((float) parameters.getSearchSkipSize());
      //      parametersBuffer.setParameter((float) parameters.getVerticalSearchSize());
      //      parametersBuffer.setParameter((float) parameters.getVerticalSearchResolution());
      //      parametersBuffer.setParameter((float) parameters.getFastSearchSize());
      //
      //      parametersBuffer.writeOpenCLBufferObject(openCLManager);
      //
      //      if (computeSteppability)
      //      {
      //         snappingParametersBuffer.setParameter((float) gridCenter.getX());
      //         snappingParametersBuffer.setParameter((float) gridCenter.getY());
      //         snappingParametersBuffer.setParameter((float) parameters.getGlobalCellSizeInMeters());
      //         snappingParametersBuffer.setParameter(globalCenterIndex);
      //         snappingParametersBuffer.setParameter((float) cropCenterIndex);
      //         snappingParametersBuffer.setParameter((float) parameters.getHeightScaleFactor());
      //         snappingParametersBuffer.setParameter((float) parameters.getHeightOffset());
      //         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getFootLength());
      //         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getFootWidth());
      //         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getDistanceFromCliffTops());
      //         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getDistanceFromCliffBottoms());
      //         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getCliffStartHeightToAvoid());
      //         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getCliffEndHeightToAvoid());
      //         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getMinSupportAreaFraction());
      //         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getMinSnapHeightThreshold());
      //         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getSnapHeightThresholdAtSearchEdge());
      //         snappingParametersBuffer.setParameter((float) this.steppableRegionParameters.getInequalityActivationSlope());
      //
      //         snappingParametersBuffer.writeOpenCLBufferObject(openCLManager);
      //      }
      //
      //      initialized = true;
      //   }
         public TerrainMapData getTerrainMapData()
         {
            return terrainMapData;
         }

      //   public static HeightMapData packHeightMapData(RapidHeightMapExtractor heightMapExtractor, HeightMapData heightMapDataToPack)
      //   {
      //      Mat heightMapMat = heightMapExtractor.getTerrainMapData().getHeightMap();
      //      HeightMapData latestHeightMapData = heightMapDataToPack;
      //      if (latestHeightMapData == null)
      //      {
      //         latestHeightMapData = new HeightMapData((float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(),
      //                                                 (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
      //                                                 heightMapExtractor.getSensorOrigin().getX(),
      //                                                 heightMapExtractor.getSensorOrigin().getY());
      //      }
      //      PerceptionMessageTools.convertToHeightMapData(heightMapMat,
      //                                                    latestHeightMapData,
      //                                                    heightMapExtractor.getSensorOrigin(),
      //                                                    (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
      //                                                    (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters());
      //
      //      return latestHeightMapData;
      //   }
         public Point3D getSensorOrigin()
         {
            return sensorOrigin;
         }
      //
      //   public static HeightMapParameters getHeightMapParameters()
      //   {
      //      return heightMapParameters;
      //   }
      //
         public void update(RigidBodyTransform sensorToWorldTransform, RigidBodyTransform sensorToGroundTransform, RigidBodyTransform groundToWorldTransform)
         {
            System.out.println("in update");
      //      if (!processing)
      //      {
      //         terrainMapStatistics.startTotalTime();
      //
      //         currentGroundToWorldTransform.set(groundToWorldTransform);
      //         currentSensorToWorldTransform.set(sensorToWorldTransform);
      //         sensorToGroundTransform.getTranslation().setZ(sensorToWorldTransform.getTranslationZ());
      //
      //         // Upload input depth image
      //         terrainMapStatistics.startDepthUploadTime();
      //         CudaHeightMapTools.BytedecoImageToPointer(inputDepthImage,inputDepthImagePointer);
      //         terrainMapStatistics.endDepthUploadTime();
      //
      //         terrainMapStatistics.startCPUProcessingTime();
      //         RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
      //         groundToSensorTransform.invert();
      //
      //         RigidBodyTransform worldToGroundTransform = new RigidBodyTransform(groundToWorldTransform);
      //         worldToGroundTransform.invert();
      //
      //         sensorOrigin.set(sensorToWorldTransform.getTranslation());
      //
      //         populateParameterBuffers(heightMapParameters, cameraIntrinsics, sensorOrigin);
      //         // Fill world-to-sensor transform buffer
      //         CudaHeightMapTools.TransformToPointer(groundToSensorTransformArray,groundToSensorTransformPointer);
      //         // Fill sensor-to-world transform buffer
      //         CudaHeightMapTools.TransformToPointer(sensorToGroundTransformArray,sensorToGroundTransformPointer);
      //         // Fill world-to-ground transform buffer
      //         CudaHeightMapTools.TransformToPointer(worldToGroundTransformArray,worldToGroundTransformPointer);
      //         //fill ground-to-world transform buffer
      //         CudaHeightMapTools.TransformToPointer(groundToSensorTransformArray,groundToSensorTransformPointer);
            }
      //
      //
      //      snappedAreaFractionImage.getBytedecoOpenCVMat().data()
      //
      public void destroy()
      {
         System.out.println("here destroy");
      }
   }
