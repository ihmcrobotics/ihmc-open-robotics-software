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

   private final Stopwatch wholeAlgorithmDurationStopwatch = new Stopwatch();
   private final Stopwatch gpuDurationStopwatch = new Stopwatch();
   private final Stopwatch depthFirstSearchDurationStopwatch = new Stopwatch();

   private ProjectionModel sensorModel;

   private PatchFeatureGrid currentFeatureGrid;
   private PatchFeatureGrid previousFeatureGrid;

   private BytedecoImage patchGraph;

   private BMatrixRMaj regionVisitedMatrix;
   private BMatrixRMaj boundaryVisitedMatrix;
   private BMatrixRMaj boundaryMatrix;
   private DMatrixRMaj regionMatrix;

   private boolean enabled = true;
   private boolean patchSizeChanged = true;
   private boolean modified = true;
   private boolean processing = false;

   private int numberOfRegionPatches = 0;
   private int regionMaxSearchDepth = 0;
   private int boundaryMaxSearchDepth = 0;
   private int numberOfBoundaryPatchesInWholeImage = 0;
   private double maxSVDSolveTime = Double.NaN;
   private double depthScalar = 1000.0;

   private final int[] adjacentY = {-1, -1, -1, 0, 0, 1, 1, 1};
   private final int[] adjacentX = {-1, 0, 1, -1, 1, -1, 0, 1};

   private int imageHeight;
   private int imageWidth;
   private int patchImageHeight;
   private int patchImageWidth;
   private int patchHeight;
   private int patchWidth;
   private int filterPatchImageHeight;
   private int filterPatchImageWidth;

   private final RapidPatchesDebugOutputGenerator debugger = new RapidPatchesDebugOutputGenerator();
   private final Stack<PatchGraphRecursionBlock> depthFirstSearchStack = new Stack<>();
   private final RecyclingArrayList<RapidPlanarRegion> rapidPlanarRegions = new RecyclingArrayList<>(RapidPlanarRegion::new);
   private final Comparator<RapidRegionRing> boundaryLengthComparator = Comparator.comparingInt(regionRing -> regionRing.getBoundaryIndices().size());

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

      regionVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      regionMatrix = new DMatrixRMaj(patchImageHeight, patchImageWidth);

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
         findRegions();
         findBoundariesAndHoles();
         growRegionBoundaries();
         depthFirstSearchDurationStopwatch.suspend();

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

   public void updateDebugger(BytedecoImage input16UC1DepthImage, ReferenceFrame cameraFrame)
   {
      debugger.update(input16UC1DepthImage.getBytedecoOpenCVMat(),
                      currentFeatureGrid,
                      patchGraph,
                      cloudBuffer.getBackingDirectFloatBuffer(),
                      cameraFrame.getTransformToWorldFrame());
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

         regionVisitedMatrix.reshape(patchImageHeight, patchImageWidth);
         boundaryVisitedMatrix.reshape(patchImageHeight, patchImageWidth);
         boundaryMatrix.reshape(patchImageHeight, patchImageWidth);
         regionMatrix.reshape(patchImageHeight, patchImageWidth);
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

   /**
    * Finds the connected regions in the patch graph using Depth First Search. It uses a heap-allocated stack object instead of process recursion stack.
    */
   public void findRegions()
   {
      int planarRegionIslandIndex = 0;
      regionMaxSearchDepth = 0;
      rapidPlanarRegions.clear();
      regionVisitedMatrix.zero();
      boundaryMatrix.zero();
      regionMatrix.zero();
      maxSVDSolveTime = 0.0;

      for (int row = 0; row < patchImageHeight; row++)
      {
         for (int column = 0; column < patchImageWidth; column++)
         {
            int boundaryConnectionsEncodedAsOnes = patchGraph.getByteAsInteger(row, column);

            if (!regionVisitedMatrix.get(row, column) && checkConnectionThreshold(boundaryConnectionsEncodedAsOnes,
                                                                                  parameters.getConnectionThreshold())) // all ones; fully connected
            {
               numberOfRegionPatches = 0; // also number of patches traversed
               RapidPlanarRegion planarRegion = rapidPlanarRegions.add();
               planarRegion.reset(planarRegionIslandIndex);

               // Push the first call on stack
               depthFirstSearchStack.push(new PatchGraphRecursionBlock(row, column, planarRegionIslandIndex, planarRegion, 1));

               // Loop until a new connected region has been found
               while (!depthFirstSearchStack.empty())
               {
                  depthFirstSearchStack.pop().expandBlock();
               }

               // Create final rapid region if the connected island has enough patches
               //LogTools.info("Min Patch Count: {} | Number of Patches: {} | Island: {}", 20, numberOfRegionPatches, planarRegionIslandIndex);
               if (numberOfRegionPatches >= parameters.getRegionMinPatches())
               {
                  //LogTools.info("Region Found: {}", planarRegionIslandIndex);
                  planarRegionIslandIndex++;
                  planarRegion.update(parameters.getUseSVDNormals(), parameters.getSVDReductionFactor());
                  if (planarRegion.getSVDDuration() > maxSVDSolveTime)
                     maxSVDSolveTime = planarRegion.getSVDDuration();
               }
               else
               {
                  int totalGPURegions = rapidPlanarRegions.size();
                  if (totalGPURegions > 0)
                  {
                     rapidPlanarRegions.remove(rapidPlanarRegions.size() - 1);
                  }
               }
               if (numberOfRegionPatches > regionMaxSearchDepth)
                  regionMaxSearchDepth = numberOfRegionPatches;
            }
         }
      }
   }

   public void findBoundariesAndHoles()
   {
      boundaryVisitedMatrix.zero();
      boundaryMaxSearchDepth = 0;
      rapidPlanarRegions.parallelStream().forEach(planarRegion ->
      {
         int leafPatchIndex = 0;
         int regionRingIndex = 0;
         planarRegion.getRegionsRingsBySize().clear();
         for (Point2D leafPatch : planarRegion.getBorderIndices())
         {
            RapidRegionRing regionRing = planarRegion.getRegionRings().add();
            regionRing.reset();
            regionRing.setIndex(regionRingIndex);
            int numberOfBoundaryPatches = boundaryDepthFirstSearch((int) leafPatch.getY(),
                                                                   (int) leafPatch.getX(),
                                                                   planarRegion.getId(),
                                                                   regionRing,
                                                                   leafPatchIndex,
                                                                   1);
            if (numberOfBoundaryPatches >= parameters.getBoundaryMinPatches())
            {
               //debugger.drawRegionRing(regionRing, patchHeight, patchWidth);

               ++regionRingIndex;
               regionRing.updateConvexPolygon();
               planarRegion.getRegionsRingsBySize().add(regionRing);
            }
            else
            {
               planarRegion.getRegionRings().remove(planarRegion.getRegionRings().size() - 1);
            }
            ++leafPatchIndex;
         }

         // remove holes
         for (RapidRegionRing regionRing : planarRegion.getRegionsRingsBySize())
         {
            planarRegion.getHoleRingsToRemove().clear();
            for (RapidRegionRing otherRegionRing : planarRegion.getRegionRings())
            {
               if (otherRegionRing != regionRing)
               {
                  // We probably only need to check one
                  Vector2D boundaryIndex = otherRegionRing.getBoundaryIndices().get(0);
                  if (regionRing.getConvexPolygon().isPointInside(boundaryIndex.getX(), boundaryIndex.getY()))
                  {
                     planarRegion.getHoleRingsToRemove().add(otherRegionRing);
                  }
               }
            }
            for (RapidRegionRing regionRingToRemove : planarRegion.getHoleRingsToRemove())
            {
               planarRegion.getRegionRings().remove(regionRingToRemove);
            }
         }

         planarRegion.getRegionRings().sort(boundaryLengthComparator);
      });
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

   private int boundaryDepthFirstSearch(int row, int column, int planarRegionId, RapidRegionRing regionRing, int leafPatchIndex, int searchDepth)
   {
      if (boundaryVisitedMatrix.get(row, column) || searchDepth > parameters.getBoundarySearchDepthLimit())
         return 0;

      if (searchDepth > boundaryMaxSearchDepth)
         boundaryMaxSearchDepth = searchDepth;

      ++numberOfBoundaryPatchesInWholeImage;
      boundaryVisitedMatrix.set(row, column, true);
      regionRing.getBoundaryIndices().add().set(column, row);

      int numberOfBoundaryPatches = 1;
      for (int i = 0; i < 8; i++)
      {
         if (row + adjacentY[i] < patchImageHeight - 1 && row + adjacentY[i] > 1 && column + adjacentX[i] < patchImageWidth - 1 && column + adjacentX[i] > 1
             && boundaryMatrix.get(row + adjacentY[i], column + adjacentX[i]) && planarRegionId == regionMatrix.get(row + adjacentY[i], column + adjacentX[i]))
         {
            numberOfBoundaryPatches += boundaryDepthFirstSearch(row + adjacentY[i],
                                                                column + adjacentX[i],
                                                                planarRegionId,
                                                                regionRing,
                                                                leafPatchIndex,
                                                                searchDepth + 1);
         }
      }
      return numberOfBoundaryPatches;
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
         LogTools.info(String.format("Patch Height: %d, Patch Width: %d, Patch Image Height: %d, Patch Image Width: %d, Filter Patch Image Height: %d, Filter Patch Image Width: %d",
                                     patchHeight,
                                     patchWidth,
                                     patchImageHeight,
                                     patchImageWidth,
                                     filterPatchImageHeight,
                                     filterPatchImageWidth));
      }
   }

   public boolean checkConnectionNonZero(int nodeConnection)
   {
      return Integer.bitCount(nodeConnection) > 0;
   }

   public boolean checkConnectionFull(int nodeConnection)
   {
      return nodeConnection == 255;
   }

   public boolean checkConnectionThreshold(int nodeConnection, int threshold)
   {
      return Integer.bitCount(nodeConnection) > threshold;
   }

   public boolean checkConnectionDirectional(int nodeConnection, int neighbor)
   {
      int mask = 1 << neighbor;
      return (nodeConnection & mask) != 0;
   }

   public class PatchGraphRecursionBlock
   {
      private final int row;
      private final int column;
      private final int planarRegionIslandIndex;
      private final RapidPlanarRegion planarRegion;
      private final int searchDepth;

      public PatchGraphRecursionBlock(int row, int column, int planarRegionIslandIndex, RapidPlanarRegion planarRegion, int searchDepth)
      {
         this.row = row;
         this.column = column;
         this.planarRegionIslandIndex = planarRegionIslandIndex;
         this.planarRegion = planarRegion;
         this.searchDepth = searchDepth;
      }

      public void expandBlock()
      {
         if (regionVisitedMatrix.get(row, column) || searchDepth > parameters.getInternalSearchDepthLimit())
            return;

         LogTools.debug("Expanding block at row: {}, column: {}, searchDepth: {}", row, column, searchDepth);

         if (searchDepth > regionMaxSearchDepth)
            regionMaxSearchDepth = searchDepth;

         ++numberOfRegionPatches;
         regionVisitedMatrix.set(row, column, true);
         regionMatrix.set(row, column, planarRegionIslandIndex);

         float nx = currentFeatureGrid.getNxImage().getFloat(row, column);
         float ny = currentFeatureGrid.getNyImage().getFloat(row, column);
         float nz = currentFeatureGrid.getNzImage().getFloat(row, column);
         float cx = currentFeatureGrid.getCxImage().getFloat(row, column);
         float cy = currentFeatureGrid.getCyImage().getFloat(row, column);
         float cz = currentFeatureGrid.getCzImage().getFloat(row, column);

         planarRegion.addRegionPatch(row, column, nx, ny, nz, cx, cy, cz);

         debugger.drawInternalNode(planarRegionIslandIndex, column, row, patchHeight, patchWidth);

         int count = 0;
         for (int i = 0; i < 8; i++)
         {
            if (row + adjacentY[i] < patchImageHeight - 1 && row + adjacentY[i] > 1 && column + adjacentX[i] < patchImageWidth - 1 && column + adjacentX[i] > 1)
            {
               int boundaryConnectionsEncodedAsOnes = patchGraph.getByteAsInteger((row + adjacentY[i]), (column + adjacentX[i]));
               if (checkConnectionThreshold(boundaryConnectionsEncodedAsOnes, parameters.getConnectionThreshold())) // all ones; fully connected
               {
                  ++count;
                  depthFirstSearchStack.push(new PatchGraphRecursionBlock(row + adjacentY[i],
                                                                          column + adjacentX[i],
                                                                          planarRegionIslandIndex,
                                                                          planarRegion,
                                                                          searchDepth + 1));
               }
            }
         }
         if (count != 8)
         {
            boundaryMatrix.set(row, column, true);
            Point2D boundaryPoint = planarRegion.getBorderIndices().add();

            if (boundaryPoint != null)
               boundaryPoint.set(column, row);
            //debugger.drawBoundaryNode(planarRegionIslandIndex, column, row, patchHeight, patchWidth);
         }
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

   public int getNumberOfBoundaryPatchesInWholeImage()
   {
      return numberOfBoundaryPatchesInWholeImage;
   }

   public RecyclingArrayList<RapidPlanarRegion> getRapidPlanarRegions()
   {
      return rapidPlanarRegions;
   }

   public int getPatchWidth()
   {
      return patchWidth;
   }

   public int getPatchHeight()
   {
      return patchHeight;
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
      return regionMaxSearchDepth;
   }

   public int getBoundaryMaxSearchDepth()
   {
      return boundaryMaxSearchDepth;
   }

   public double getMaxSVDSolveTime()
   {
      return maxSVDSolveTime;
   }

   public BytedecoImage getPatchGraph()
   {
      return patchGraph;
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

