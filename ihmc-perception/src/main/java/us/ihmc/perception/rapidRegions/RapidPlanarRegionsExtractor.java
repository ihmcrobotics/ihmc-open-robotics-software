package us.ihmc.perception.rapidRegions;

import controller_msgs.msg.dds.RobotConfigurationData;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.ejml.data.BMatrixRMaj;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.graphicalSegmentation.GraphicalSegmentationCalculator;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionFilterTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;

import java.util.Comparator;
import java.util.Stack;

public class RapidPlanarRegionsExtractor
{
   private CollidingScanRegionFilter collidingScanRegionFilter;
   private RapidPlanarRegionsCustomizer rapidPlanarRegionsCustomizer;
   private RapidRegionsExtractorParameters parameters;
   private GraphicalSegmentationCalculator graphicalSegmentationCalculator;

   private final Stopwatch wholeAlgorithmDurationStopwatch = new Stopwatch();
   private final Stopwatch gpuDurationStopwatch = new Stopwatch();
   private final Stopwatch depthFirstSearchDurationStopwatch = new Stopwatch();

   private ProjectionModel sensorModel;

   private PatchFeatureGrid currentFeatureGrid;
   private PatchFeatureGrid previousFeatureGrid;
   private BytedecoImage patchGraph;

   private boolean enabled = true;
   private boolean patchSizeChanged = true;
   private boolean modified = true;
   private boolean processing = false;

   private double depthScalar = 1000.0;

   private int imageHeight;
   private int imageWidth;
   private int patchImageHeight;
   private int patchImageWidth;
   private int patchHeight;
   private int patchWidth;
   private int filterPatchImageHeight;
   private int filterPatchImageWidth;

   private final RapidPatchesDebugOutputGenerator debugger = new RapidPatchesDebugOutputGenerator();

   private final RecyclingArrayList<RapidPlanarRegion> rapidPlanarRegions = new RecyclingArrayList<>(RapidPlanarRegion::new);

   private OpenCLManager openCLManager;
   private OpenCLFloatParameters parametersBuffer;
   private _cl_program planarRegionExtractionProgram;
   private _cl_kernel packKernel;
   private _cl_kernel mergeKernel;
   private _cl_kernel copyKernel;

   // TODO: Remove
   //private _cl_kernel sphericalBackProjectionKernel;
   //private _cl_kernel perspectiveBackProjectionKernel;
   private OpenCLFloatBuffer cloudBuffer;

   private final PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private final FramePlanarRegionsList framePlanarRegionsList = new FramePlanarRegionsList();
   private final RapidPlanarRegionIsland tempIsland = new RapidPlanarRegionIsland();

   private FullHumanoidRobotModel fullRobotModel;
   private CollisionBoxProvider collisionBoxProvider;
   private RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();

   private boolean firstRun = true;
   boolean waitIfNecessary = false; // dangerous if true! need a timeout

   public RapidPlanarRegionsExtractor(OpenCLManager openCLManager, CameraIntrinsics cameraIntrinsics)
   {
      this(openCLManager,
           openCLManager.loadProgram("RapidRegionsExtractor"),
           cameraIntrinsics.getHeight(),
           cameraIntrinsics.getWidth(),
           cameraIntrinsics.getFx(),
           cameraIntrinsics.getFy(),
           cameraIntrinsics.getCx(),
           cameraIntrinsics.getCy());
   }

   public RapidPlanarRegionsExtractor(OpenCLManager openCLManager, int imageHeight, int imageWidth, double fx, double fy, double cx, double cy)
   {
      this(openCLManager, openCLManager.loadProgram("RapidRegionsExtractor"), imageHeight, imageWidth, fx, fy, cx, cy);
   }

   public RapidPlanarRegionsExtractor(OpenCLManager openCLManager,
                                      _cl_program program,
                                      int imageHeight,
                                      int imageWidth,
                                      double fx,
                                      double fy,
                                      double cx,
                                      double cy)
   {
      this(openCLManager, program, imageHeight, imageWidth, fx, fy, cx, cy, "");
   }

   /**
    * Creates buffers and kernels for the OpenCL program.
    *
    * @param imageWidth  width of the input depth image
    * @param imageHeight height of the input depth image
    */
   public RapidPlanarRegionsExtractor(OpenCLManager openCLManager,
                                      _cl_program program,
                                      int imageHeight,
                                      int imageWidth,
                                      double fx,
                                      double fy,
                                      double cx,
                                      double cy,
                                      String version)
   {
      this.sensorModel = ProjectionModel.PERSPECTIVE;
      this.openCLManager = openCLManager;
      this.planarRegionExtractionProgram = program;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;

      this.parameters = new RapidRegionsExtractorParameters(version);
      this.parameters.set(RapidRegionsExtractorParameters.focalLengthXPixels, fx);
      this.parameters.set(RapidRegionsExtractorParameters.focalLengthYPixels, fy);
      this.parameters.set(RapidRegionsExtractorParameters.principalOffsetXPixels, cx);
      this.parameters.set(RapidRegionsExtractorParameters.principalOffsetYPixels, cy);

      rapidPlanarRegionsCustomizer = new RapidPlanarRegionsCustomizer();

      //perspectiveBackProjectionKernel = openCLManager.createKernel(planarRegionExtractionProgram, "perspectiveBackProjectionKernel");
      this.create();
   }

   public RapidPlanarRegionsExtractor(OpenCLManager openCLManager, int imageHeight, int imageWidth, ProjectionModel model)
   {
      this(openCLManager, openCLManager.loadProgram("RapidRegionsExtractor"), imageHeight, imageWidth, model);
   }

   public RapidPlanarRegionsExtractor(OpenCLManager openCLManager, _cl_program program, int imageHeight, int imageWidth, ProjectionModel model)
   {
      this.sensorModel = model;
      this.openCLManager = openCLManager;
      this.planarRegionExtractionProgram = program;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;

      if (model == ProjectionModel.SPHERICAL)
      {
         this.parameters = new RapidRegionsExtractorParameters("Spherical");
         rapidPlanarRegionsCustomizer = new RapidPlanarRegionsCustomizer("ForSphericalRapidRegions");
         //sphericalBackProjectionKernel = openCLManager.createKernel(planarRegionExtractionProgram, "sphericalBackProjectionKernel");
      }
      else if (model == ProjectionModel.ORTHOGRAPHIC)
      {
         this.parameters = new RapidRegionsExtractorParameters("Orthographic");
         rapidPlanarRegionsCustomizer = new RapidPlanarRegionsCustomizer();
      }

      create();
   }

   private void create()
   {
      calculateDerivativeParameters();

      LogTools.info("Creating buffers and kernels for OpenCL program.");

      debugger.create(imageHeight, imageWidth);
      parametersBuffer = new OpenCLFloatParameters();
      cloudBuffer = new OpenCLFloatBuffer(imageHeight * imageWidth * 3);

      currentFeatureGrid = new PatchFeatureGrid(openCLManager, patchImageWidth, patchImageHeight);
      previousFeatureGrid = new PatchFeatureGrid(openCLManager, patchImageWidth, patchImageHeight);
      patchGraph = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_8UC1);

      packKernel = openCLManager.createKernel(planarRegionExtractionProgram, "packKernel");
      mergeKernel = openCLManager.createKernel(planarRegionExtractionProgram, "mergeKernel");
      copyKernel = openCLManager.createKernel(planarRegionExtractionProgram, "copyKernel");

      graphicalSegmentationCalculator = new GraphicalSegmentationCalculator(parameters, currentFeatureGrid, patchGraph, debugger);

      LogTools.info("Finished creating buffers and kernels for OpenCL program.");
   }

   public void initializeBodyCollisionFilter(FullHumanoidRobotModel robotModel, CollisionBoxProvider collisionBoxProvider)
   {
      if (robotModel == null)
      {
         LogTools.warn("Cannot initialize body collision filter. Robot model is null.");
         return;
      }

      if (collisionBoxProvider == null)
      {
         LogTools.warn("Cannot initialize body collision filter. Robot collision box provider is null.");
         return;
      }

      this.fullRobotModel = robotModel;
      this.collisionBoxProvider = collisionBoxProvider;


      this.collidingScanRegionFilter = PerceptionFilterTools.createHumanoidShinCollisionFilter(fullRobotModel, this.collisionBoxProvider);
   }

   public void filterFramePlanarRegionsList(FramePlanarRegionsList frameRegionsToFilter)
   {
      if (fullRobotModel == null || collidingScanRegionFilter == null)
         return;

      this.fullRobotModel.updateFrames();
      this.collidingScanRegionFilter.update();

      synchronized (frameRegionsToFilter)
      {
         PerceptionFilterTools.filterCollidingPlanarRegions(frameRegionsToFilter, this.collidingScanRegionFilter);
      }
   }

   public void updateRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      if (robotConfigurationData != null && robotConfigurationData.getJointNameHash() != 0)
      {
         robotConfigurationDataBuffer.update(robotConfigurationData);
         long newestTimestamp = robotConfigurationDataBuffer.getNewestTimestamp();
         long selectedTimestamp = robotConfigurationDataBuffer.updateFullRobotModel(waitIfNecessary, newestTimestamp, this.fullRobotModel, null);
      }
   }

   public void update(BytedecoImage input16UC1DepthImage, ReferenceFrame cameraFrame, FramePlanarRegionsList frameRegions)
   {
      if (!processing && enabled)
      {
         processing = true;
         debugger.clearDebugImage();
         wholeAlgorithmDurationStopwatch.start();

         gpuDurationStopwatch.start();
         computePatchFeatureGrid(input16UC1DepthImage);
         gpuDurationStopwatch.suspend();

         depthFirstSearchDurationStopwatch.start();
         graphicalSegmentationCalculator.findRegions(rapidPlanarRegions);
         graphicalSegmentationCalculator.findBoundariesAndHoles(rapidPlanarRegions);
         depthFirstSearchDurationStopwatch.suspend();

         growRegionBoundaries();
         rapidPlanarRegionsCustomizer.createCustomPlanarRegionsList(rapidPlanarRegions, cameraFrame, frameRegions);

         filterFramePlanarRegionsList(frameRegions);

         wholeAlgorithmDurationStopwatch.suspend();

         debugger.update(input16UC1DepthImage.getBytedecoOpenCVMat(),
                         currentFeatureGrid,
                         patchGraph,
                         cloudBuffer.getBackingDirectFloatBuffer(),
                         cameraFrame.getTransformToWorldFrame());

         modified = true;
      }
   }

   /**
    * Extracts features and generates patch graph from the input depth image on the GPU.
    */
   public void computePatchFeatureGrid(BytedecoImage input16UC1DepthImage)
   {
      calculateDerivativeParameters();

      parametersBuffer.setParameter((float) 0);
      parametersBuffer.setParameter((float) parameters.getMergeAngularThreshold());
      parametersBuffer.setParameter((float) parameters.getMergeOrthogonalThreshold());
      parametersBuffer.setParameter(patchHeight);
      parametersBuffer.setParameter(patchWidth);
      parametersBuffer.setParameter(patchImageHeight);
      parametersBuffer.setParameter(patchImageWidth);
      parametersBuffer.setParameter((float) parameters.getFocalLengthXPixels());
      parametersBuffer.setParameter((float) parameters.getFocalLengthYPixels());
      parametersBuffer.setParameter((float) parameters.getPrincipalOffsetXPixels());
      parametersBuffer.setParameter((float) parameters.getPrincipalOffsetYPixels());
      parametersBuffer.setParameter(parameters.getDeadPixelFilterPatchSize());
      parametersBuffer.setParameter(filterPatchImageHeight);
      parametersBuffer.setParameter(filterPatchImageWidth);
      parametersBuffer.setParameter(imageHeight);
      parametersBuffer.setParameter(imageWidth);
      parametersBuffer.setParameter((float) parameters.getNormalPackRange());
      parametersBuffer.setParameter((float) parameters.getCentroidPackRange());
      parametersBuffer.setParameter((float) parameters.getMergeRange());
      parametersBuffer.setParameter((float) parameters.getMergeDistanceThreshold());

      switch (sensorModel)
      {
         case ORTHOGRAPHIC:
            parametersBuffer.setParameter(0.0f);
            break;
         case PERSPECTIVE:
            parametersBuffer.setParameter(1.0f);
            break;
         case SPHERICAL:
            parametersBuffer.setParameter(2.0f);
            break;
      }

      parametersBuffer.setParameter((float) depthScalar);

      parametersBuffer.writeOpenCLBufferObject(openCLManager);

      if (patchSizeChanged)
      {
         patchSizeChanged = false;
         LogTools.info("Resizing patch image to {}x{}", patchImageWidth, patchImageHeight);

         currentFeatureGrid.resize(patchImageWidth, patchImageHeight);
         previousFeatureGrid.resize(patchImageWidth, patchImageHeight);
         patchGraph.resize(patchImageWidth, patchImageHeight, openCLManager, null);

         graphicalSegmentationCalculator.reshape(patchImageHeight, patchImageWidth);
      }
      if (firstRun)
      {
         LogTools.info("First Run.");
         firstRun = false;
         input16UC1DepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

         currentFeatureGrid.createOpenCLImages();
         previousFeatureGrid.createOpenCLImages();

         patchGraph.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         cloudBuffer.createOpenCLBufferObject(openCLManager);
      }
      else
      {
         //         LogTools.info("Writing to OpenCL Image");
         input16UC1DepthImage.writeOpenCLImage(openCLManager);
         parametersBuffer.writeOpenCLBufferObject(openCLManager);
      }

      //      LogTools.info("Done Writing Input Image");

      _cl_mem inputImage = input16UC1DepthImage.getOpenCLImageObject();

      openCLManager.setKernelArgument(packKernel, 0, inputImage);
      openCLManager.setKernelArgument(packKernel, 1, currentFeatureGrid.getNxImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 2, currentFeatureGrid.getNyImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 3, currentFeatureGrid.getNzImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 4, currentFeatureGrid.getCxImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 5, currentFeatureGrid.getCyImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 6, currentFeatureGrid.getCzImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 7, parametersBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(packKernel, patchImageWidth, patchImageHeight);

      openCLManager.setKernelArgument(mergeKernel, 0, inputImage);
      openCLManager.setKernelArgument(mergeKernel, 1, currentFeatureGrid.getNxImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 2, currentFeatureGrid.getNyImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 3, currentFeatureGrid.getNzImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 4, currentFeatureGrid.getCxImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 5, currentFeatureGrid.getCyImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 6, currentFeatureGrid.getCzImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 7, patchGraph.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 8, parametersBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(mergeKernel, patchImageWidth, patchImageHeight);

      currentFeatureGrid.readOpenCLImages();
      patchGraph.readOpenCLImage(openCLManager);

      // TODO: Remove
      //if (sensorModel == ProjectionModel.SPHERICAL)
      //{
      //   openCLManager.setKernelArgument(sphericalBackProjectionKernel, 0, inputImage);
      //   openCLManager.setKernelArgument(sphericalBackProjectionKernel, 1, cloudBuffer.getOpenCLBufferObject());
      //   openCLManager.setKernelArgument(sphericalBackProjectionKernel, 2, parametersBuffer.getOpenCLBufferObject());
      //   openCLManager.execute2D(sphericalBackProjectionKernel, imageWidth, imageHeight);
      //   cloudBuffer.readOpenCLBufferObject(openCLManager);
      //}
      //
      //if (sensorModel == ProjectionModel.PERSPECTIVE)
      //{
      //   openCLManager.setKernelArgument(perspectiveBackProjectionKernel, 0, inputImage);
      //   openCLManager.setKernelArgument(perspectiveBackProjectionKernel, 1, cloudBuffer.getOpenCLBufferObject());
      //   openCLManager.setKernelArgument(perspectiveBackProjectionKernel, 2, parametersBuffer.getOpenCLBufferObject());
      //   openCLManager.execute2D(perspectiveBackProjectionKernel, imageWidth, imageHeight);
      //   cloudBuffer.readOpenCLBufferObject(openCLManager);
      //}
   }

   public void copyFeatureGridMapUsingOpenCL()
   {
      openCLManager.setKernelArgument(copyKernel, 0, currentFeatureGrid.getNxImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(copyKernel, 1, currentFeatureGrid.getNyImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(copyKernel, 2, currentFeatureGrid.getNzImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(copyKernel, 3, currentFeatureGrid.getCxImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(copyKernel, 4, currentFeatureGrid.getCyImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(copyKernel, 5, currentFeatureGrid.getCzImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(copyKernel, 6, previousFeatureGrid.getNxImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(copyKernel, 7, previousFeatureGrid.getNyImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(copyKernel, 8, previousFeatureGrid.getNzImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(copyKernel, 9, previousFeatureGrid.getCxImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(copyKernel, 10, previousFeatureGrid.getCyImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(copyKernel, 11, previousFeatureGrid.getCzImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(copyKernel, 12, parametersBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(copyKernel, patchImageWidth, patchImageHeight);
   }


   public void growRegionBoundaries()
   {
      rapidPlanarRegions.forEach(planarRegion ->
      {
         if (!planarRegion.getRegionRings().isEmpty())
         {
            RapidRegionRing firstRing = planarRegion.getRegionRings().get(0);
            for (Vector2D boundaryIndex : firstRing.getBoundaryIndices())
            {
               // kernel coordinates is in left-handed frame, so lets flip it to IHMC Z up

               //float vertexX = czImage.getFloat((int) boundaryIndex.getY(), (int) boundaryIndex.getX());
               //float vertexY = -cxImage.getFloat((int) boundaryIndex.getY(), (int) boundaryIndex.getX());
               //float vertexZ = cyImage.getFloat((int) boundaryIndex.getY(), (int) boundaryIndex.getX());

               float vertexX = currentFeatureGrid.getCxImage().getFloat((int) boundaryIndex.getY(), (int) boundaryIndex.getX());
               float vertexY = currentFeatureGrid.getCyImage().getFloat((int) boundaryIndex.getY(), (int) boundaryIndex.getX());
               float vertexZ = currentFeatureGrid.getCzImage().getFloat((int) boundaryIndex.getY(), (int) boundaryIndex.getX());

               Point3D boundaryVertex = planarRegion.getBoundaryVertices().add();
               boundaryVertex.set(vertexX, vertexY, vertexZ);
               boundaryVertex.sub(planarRegion.getCenter());
               boundaryVertex.normalize();
               boundaryVertex.scale(parameters.getRegionGrowthFactor());
               boundaryVertex.add(vertexX, vertexY, vertexZ);
            }
         }
      });
   }



   private void calculateDerivativeParameters()
   {
      patchHeight = parameters.getPatchSize();
      patchWidth = parameters.getPatchSize();
      patchImageHeight = imageHeight / patchHeight;
      patchImageWidth = imageWidth / patchWidth;
      filterPatchImageHeight = imageHeight / parameters.getDeadPixelFilterPatchSize();
      filterPatchImageWidth = imageWidth / parameters.getDeadPixelFilterPatchSize();

      if (debugger.isEnabled())
      {
         LogTools.debug(String.format("Patch Height: %d, Patch Width: %d, Patch Image Height: %d, Patch Image Width: %d, Filter Patch Image Height: %d, Filter Patch Image Width: %d",
                                     patchHeight,
                                     patchWidth,
                                     patchImageHeight,
                                     patchImageWidth,
                                     filterPatchImageHeight,
                                     filterPatchImageWidth));
      }
   }


   public void destroy()
   {
      currentFeatureGrid.destroy();
      patchGraph.destroy(openCLManager);
      openCLManager.destroy();
      // TODO: Destroy the rest
   }

   public RapidPlanarRegionsCustomizer getRapidPlanarRegionsCustomizer()
   {
      return rapidPlanarRegionsCustomizer;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public RapidPatchesDebugOutputGenerator getDebugger()
   {
      return debugger;
   }

   public void setPatchSizeChanged(boolean patchSizeChanged)
   {
      this.patchSizeChanged = patchSizeChanged;
   }

   public int getPatchImageWidth()
   {
      return patchImageWidth;
   }

   public int getPatchImageHeight()
   {
      return patchImageHeight;
   }

   public RecyclingArrayList<RapidPlanarRegion> getRapidPlanarRegions()
   {
      return rapidPlanarRegions;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public RapidRegionsExtractorParameters getParameters()
   {
      return parameters;
   }

   public int getRegionMaxSearchDepth()
   {
      return graphicalSegmentationCalculator.getRegionMaxSearchDepth();
   }

   public int getBoundaryMaxSearchDepth()
   {
      return graphicalSegmentationCalculator.getBoundaryMaxSearchDepth();
   }

   public double getMaxSVDSolveTime()
   {
      return graphicalSegmentationCalculator.getMaxSVDSolveTime();
   }

   public Stopwatch getWholeAlgorithmDurationStopwatch()
   {
      return wholeAlgorithmDurationStopwatch;
   }

   public Stopwatch getGpuDurationStopwatch()
   {
      return gpuDurationStopwatch;
   }

   public Stopwatch getDepthFirstSearchDurationStopwatch()
   {
      return depthFirstSearchDurationStopwatch;
   }

   public PatchFeatureGrid getCurrentFeatureGrid()
   {
      return currentFeatureGrid;
   }

   public PatchFeatureGrid getPreviousFeatureGrid()
   {
      return previousFeatureGrid;
   }

   public boolean isModified()
   {
      return modified;
   }

   public void setModified(boolean modified)
   {
      this.modified = modified;
   }

   public boolean isProcessing()
   {
      return processing;
   }

   public void setProcessing(boolean processing)
   {
      this.processing = processing;
   }

   public boolean getEnabled()
   {
      return enabled;
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

    public void setDepthScalar(double depthScalar)
    {
       this.depthScalar = depthScalar;
    }
}

