package us.ihmc.avatar.gpuPlanarRegions;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.ejml.data.BMatrixRMaj;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;

import java.util.Comparator;
import java.util.Stack;

import static us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters.*;

public class RapidPlanarRegionsExtractor
{
   private BytedecoImage inputU16DepthImage;

   private enum SensorModel
   {
      SPHERICAL, PERSPECTIVE
   }

   private final GPUPlanarRegionExtractionParameters parameters;

   private SensorModel sensorModel;

   private BytedecoImage nxImage;
   private BytedecoImage nyImage;
   private BytedecoImage nzImage;
   private BytedecoImage cxImage;
   private BytedecoImage cyImage;
   private BytedecoImage czImage;
   private BytedecoImage patchGraph;

   private BMatrixRMaj regionVisitedMatrix;
   private BMatrixRMaj boundaryVisitedMatrix;
   private BMatrixRMaj boundaryMatrix;
   private DMatrixRMaj regionMatrix;

   private boolean patchSizeChanged = true;

   private int numberOfRegionPatches = 0;
   private int regionMaxSearchDepth = 0;
   private int boundaryMaxSearchDepth = 0;
   private int numberOfBoundaryPatchesInWholeImage = 0;
   private double maxSVDSolveTime = Double.NaN;

   private final int[] adjacentY = {-1, 0, 1, 1, 1, 0, -1, -1};
   private final int[] adjacentX = {-1, -1, -1, 0, 1, 1, 1, 0};

   private int imageHeight;
   private int imageWidth;
   private int patchImageHeight;
   private int patchImageWidth;
   private int patchHeight;
   private int patchWidth;
   private int filterPatchImageHeight;
   private int filterPatchImageWidth;

   private final Stack<PatchGraphRecursionBlock> depthFirstSearchStack = new Stack<>();
   private final RecyclingArrayList<GPUPlanarRegion> gpuPlanarRegions = new RecyclingArrayList<>(GPUPlanarRegion::new);
   private final Comparator<GPURegionRing> boundaryLengthComparator = Comparator.comparingInt(regionRing -> regionRing.getBoundaryIndices().size());

   private OpenCLManager openCLManager;
   private OpenCLFloatBuffer parametersBuffer;
   private _cl_program planarRegionExtractionProgram;
   private _cl_kernel packKernel;
   private _cl_kernel mergeKernel;

   private final PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private final PlanarRegionsListWithPose planarRegionsListWithPose = new PlanarRegionsListWithPose();
   private final GPUPlanarRegionIsland tempIsland = new GPUPlanarRegionIsland();
   private boolean firstRun = true;

   public RapidPlanarRegionsExtractor(GPUPlanarRegionExtractionParameters parameters)
   {
      this.parameters = parameters;
   }

   /**
    * Creates buffers and kernels for the OpenCL program.
    *
    * @param imageWidth  width of the input depth image
    * @param imageHeight height of the input depth image
    */
   public void create(OpenCLManager openCLManager, BytedecoImage depthImage, int imageWidth, int imageHeight, double fx, double fy, double cx, double cy)
   {
      this.sensorModel = SensorModel.PERSPECTIVE;
      this.openCLManager = openCLManager;
      this.inputU16DepthImage = depthImage;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;

      parameters.set(focalLengthXPixels, fx);
      parameters.set(focalLengthYPixels, fy);
      parameters.set(principalOffsetXPixels, cx);
      parameters.set(principalOffsetYPixels, cy);

      parametersBuffer = new OpenCLFloatBuffer(16);
      calculateDerivativeParameters();

      nxImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      nyImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      nzImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      cxImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      cyImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      czImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);

      patchGraph = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_8UC1);

      planarRegionExtractionProgram = openCLManager.loadProgram("RapidRegionsExtractor");
      packKernel = openCLManager.createKernel(planarRegionExtractionProgram, "packKernel");
      mergeKernel = openCLManager.createKernel(planarRegionExtractionProgram, "mergeKernel");

      regionVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      regionMatrix = new DMatrixRMaj(patchImageHeight, patchImageWidth);
   }

   public void create(OpenCLManager openCLManager, BytedecoImage depthImage, int imageWidth, int imageHeight)
   {
      this.sensorModel = SensorModel.SPHERICAL;
      this.openCLManager = openCLManager;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.inputU16DepthImage = depthImage;

      parametersBuffer = new OpenCLFloatBuffer(16);
      calculateDerivativeParameters();

      nxImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      nyImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      nzImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      cxImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      cyImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      czImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      patchGraph = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_8UC1);

      openCLManager.create();
      planarRegionExtractionProgram = openCLManager.loadProgram("RapidRegionsExtractor");
      packKernel = openCLManager.createKernel(planarRegionExtractionProgram, "packKernel");
      mergeKernel = openCLManager.createKernel(planarRegionExtractionProgram, "mergeKernel");

      regionVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      regionMatrix = new DMatrixRMaj(patchImageHeight, patchImageWidth);
   }

   public void update()
   {
      LogTools.info("++++++++++++++ [START] Calling Update ++++++++++++++++");
      extractPatchGraphUsingOpenCL();
      findRegions();
      findBoundariesAndHoles();
      growRegionBoundaries();
      LogTools.info(" --------------------- [DONE] Calling Update -------------------------- ");
   }

   /**
    * Extracts features and generates patch graph from the input depth image on the GPU.
    */
   public void extractPatchGraphUsingOpenCL()
   {
      LogTools.info("++++++++++++++ [START] Generating Patch Graph ++++++++++++++++");
      calculateDerivativeParameters();

      // Flip so the Y+ goes up instead of down.
      opencv_core.flip(inputU16DepthImage.getBytedecoOpenCVMat(), inputU16DepthImage.getBytedecoOpenCVMat(), BytedecoOpenCVTools.FLIP_Y);

      parametersBuffer.getBytedecoFloatBufferPointer().put(0, (float) parameters.getFilterDisparityThreshold());
      parametersBuffer.getBytedecoFloatBufferPointer().put(1, (float) parameters.getMergeAngularThreshold());
      parametersBuffer.getBytedecoFloatBufferPointer().put(2, (float) parameters.getMergeDistanceThreshold());
      parametersBuffer.getBytedecoFloatBufferPointer().put(3, patchHeight);
      parametersBuffer.getBytedecoFloatBufferPointer().put(4, patchWidth);
      parametersBuffer.getBytedecoFloatBufferPointer().put(5, patchImageHeight);
      parametersBuffer.getBytedecoFloatBufferPointer().put(6, patchImageWidth);
      parametersBuffer.getBytedecoFloatBufferPointer().put(7, (float) parameters.getFocalLengthXPixels());
      parametersBuffer.getBytedecoFloatBufferPointer().put(8, (float) parameters.getFocalLengthYPixels());
      parametersBuffer.getBytedecoFloatBufferPointer().put(9, (float) parameters.getPrincipalOffsetXPixels());
      parametersBuffer.getBytedecoFloatBufferPointer().put(10, (float) parameters.getPrincipalOffsetYPixels());
      parametersBuffer.getBytedecoFloatBufferPointer().put(11, parameters.getDeadPixelFilterPatchSize());
      parametersBuffer.getBytedecoFloatBufferPointer().put(12, filterPatchImageHeight);
      parametersBuffer.getBytedecoFloatBufferPointer().put(13, filterPatchImageWidth);
      parametersBuffer.getBytedecoFloatBufferPointer().put(14, imageHeight);
      parametersBuffer.getBytedecoFloatBufferPointer().put(15, imageWidth);

      if (patchSizeChanged)
      {
         patchSizeChanged = false;
         LogTools.info("Resizing patch image to {}x{}", patchImageWidth, patchImageHeight);

         nxImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         nyImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         nzImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         cxImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         cyImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         czImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         patchGraph.resize(patchImageWidth, patchImageHeight, openCLManager, null);

         regionVisitedMatrix.reshape(patchImageHeight, patchImageWidth);
         boundaryVisitedMatrix.reshape(patchImageHeight, patchImageWidth);
         boundaryMatrix.reshape(patchImageHeight, patchImageWidth);
         regionMatrix.reshape(patchImageHeight, patchImageWidth);
      }
      if (firstRun)
      {
         firstRun = false;
         inputU16DepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);

         nxImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         nyImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         nzImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         cxImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         cyImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         czImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         patchGraph.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         parametersBuffer.createOpenCLBufferObject(openCLManager);
      }
      else
      {
         inputU16DepthImage.writeOpenCLImage(openCLManager);
         parametersBuffer.writeOpenCLBufferObject(openCLManager);
      }

      _cl_mem inputImage = inputU16DepthImage.getOpenCLImageObject();

      openCLManager.setKernelArgument(packKernel, 0, inputImage);
      openCLManager.setKernelArgument(packKernel, 1, nxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 2, nyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 3, nzImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 4, cxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 5, cyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 6, czImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 7, parametersBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(packKernel, patchImageHeight, patchImageWidth);

      openCLManager.setKernelArgument(mergeKernel, 0, nxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 1, nyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 2, nzImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 3, cxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 4, cyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 5, czImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 6, patchGraph.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 7, parametersBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(mergeKernel, patchImageHeight, patchImageWidth);

      openCLManager.enqueueReadImage(nxImage.getOpenCLImageObject(), patchImageWidth, patchImageHeight, nxImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(nyImage.getOpenCLImageObject(), patchImageWidth, patchImageHeight, nyImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(nzImage.getOpenCLImageObject(), patchImageWidth, patchImageHeight, nzImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(cxImage.getOpenCLImageObject(), patchImageWidth, patchImageHeight, cxImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(cyImage.getOpenCLImageObject(), patchImageWidth, patchImageHeight, cyImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(czImage.getOpenCLImageObject(), patchImageWidth, patchImageHeight, czImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(patchGraph.getOpenCLImageObject(), patchImageWidth, patchImageHeight, patchGraph.getBytedecoByteBufferPointer());

      openCLManager.finish();

      LogTools.info(" --------------------- [DONE] Generating Patch Graph -------------------------- ");
   }

   /**
    * Finds the connected regions in the patch graph using Depth First Search. It uses a heap-allocated stack object instead of process recursion stack.
    */
   public void findRegions()
   {
      LogTools.info("++++++++++++++ [START] Finding Regions ++++++++++++++++");
      int planarRegionIslandIndex = 0;
      regionMaxSearchDepth = 0;
      gpuPlanarRegions.clear();
      regionVisitedMatrix.zero();
      boundaryMatrix.zero();
      regionMatrix.zero();
      maxSVDSolveTime = 0.0;

      for (int row = 0; row < patchImageHeight; row++)
      {
         for (int column = 0; column < patchImageWidth; column++)
         {
            int boundaryConnectionsEncodedAsOnes = Byte.toUnsignedInt(patchGraph.getBytedecoOpenCVMat().ptr(row, column).get());
            if (!regionVisitedMatrix.get(row, column) && boundaryConnectionsEncodedAsOnes == 255) // all ones; fully connected
            {
               numberOfRegionPatches = 0; // also number of patches traversed
               GPUPlanarRegion planarRegion = gpuPlanarRegions.add();
               planarRegion.reset(planarRegionIslandIndex);

               depthFirstSearchStack.push(new PatchGraphRecursionBlock(row, column, planarRegionIslandIndex, planarRegion, 1));
               while (!depthFirstSearchStack.empty())
               {
                  depthFirstSearchStack.pop().expandBlock();
               }
               if (numberOfRegionPatches >= parameters.getRegionMinPatches())
               {
                  ++planarRegionIslandIndex;
                  planarRegion.update(parameters.getUseSVDNormals(), parameters.getSVDReductionFactor());
                  if (planarRegion.getSVDDuration() > maxSVDSolveTime)
                     maxSVDSolveTime = planarRegion.getSVDDuration();

                  tempIsland.planarRegion = planarRegion;
                  tempIsland.planarRegionIslandIndex = planarRegionIslandIndex;
               }
               else
               {
                  gpuPlanarRegions.remove(gpuPlanarRegions.size() - 1);
               }
               if (numberOfRegionPatches > regionMaxSearchDepth)
                  regionMaxSearchDepth = numberOfRegionPatches;
            }
         }
      }
      LogTools.info("Total GPUPlanarRegions Found: {}", gpuPlanarRegions.size());
      LogTools.info(" --------------------- [DONE] Generating Patch Graph -------------------------- ");
   }

   public void findBoundariesAndHoles()
   {
      LogTools.info("++++++++++++++ [START] Finding Boundaries and Holes ++++++++++++++++");
      boundaryVisitedMatrix.zero();
      boundaryMaxSearchDepth = 0;
      gpuPlanarRegions.parallelStream().forEach(planarRegion ->
      {
         int leafPatchIndex = 0;
         int regionRingIndex = 0;
         planarRegion.getRegionsRingsBySize().clear();
         for (Point2D leafPatch : planarRegion.getBorderIndices())
         {
            GPURegionRing regionRing = planarRegion.getRegionRings().add();
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
         for (GPURegionRing regionRing : planarRegion.getRegionsRingsBySize())
         {
            planarRegion.getHoleRingsToRemove().clear();
            for (GPURegionRing otherRegionRing : planarRegion.getRegionRings())
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
            for (GPURegionRing regionRingToRemove : planarRegion.getHoleRingsToRemove())
            {
               planarRegion.getRegionRings().remove(regionRingToRemove);
            }
         }

         planarRegion.getRegionRings().sort(boundaryLengthComparator);
      });
      LogTools.info(" --------------------- [DONE] Finding Boundaries and Holes -------------------------- ");
   }

   public void growRegionBoundaries()
   {
      LogTools.info("++++++++++++++ [START] Growing Region Boundaries ++++++++++++++++");
      gpuPlanarRegions.forEach(planarRegion ->
                               {
                                  if (!planarRegion.getRegionRings().isEmpty())
                                  {
                                     GPURegionRing firstRing = planarRegion.getRegionRings().get(0);
                                     for (Vector2D boundaryIndex : firstRing.getBoundaryIndices())
                                     {
                                        // kernel coordinates is in left-handed frame, so lets flip it to IHMC Z up
                                        float vertexX = czImage.getBytedecoOpenCVMat().ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX()).getFloat();
                                        float vertexY = -cxImage.getBytedecoOpenCVMat().ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX()).getFloat();
                                        float vertexZ = cyImage.getBytedecoOpenCVMat().ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX()).getFloat();

                                        Vector3D boundaryVertex = planarRegion.getBoundaryVertices().add();
                                        boundaryVertex.set(vertexX, vertexY, vertexZ);
                                        boundaryVertex.sub(planarRegion.getCenter());
                                        boundaryVertex.normalize();
                                        boundaryVertex.scale(parameters.getRegionGrowthFactor());
                                        boundaryVertex.add(vertexX, vertexY, vertexZ);
                                     }
                                  }
                               });
      LogTools.info(" --------------------- [DONE] Growing Region Boundaries -------------------------- ");
   }

   private int boundaryDepthFirstSearch(int row, int column, int planarRegionId, GPURegionRing regionRing, int leafPatchIndex, int searchDepth)
   {
      if (boundaryVisitedMatrix.get(row, column) || searchDepth > parameters.getSearchDepthLimit())
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
      int previousPatchHeight = patchHeight;
      int previousPatchWidth = patchWidth;
      int previousPatchImageHeight = patchImageHeight;
      int previousPatchImageWidth = patchImageWidth;
      int previousFilterPatchImageHeight = filterPatchImageHeight;
      int previousFilterPatchImageWidth = filterPatchImageWidth;

      patchHeight = parameters.getPatchSize();
      patchWidth = parameters.getPatchSize();
      patchImageHeight = imageHeight / patchHeight;
      patchImageWidth = imageWidth / patchWidth;
      filterPatchImageHeight = imageHeight / parameters.getDeadPixelFilterPatchSize();
      filterPatchImageWidth = imageWidth / parameters.getDeadPixelFilterPatchSize();

      int newPatchHeight = patchHeight;
      int newPatchWidth = patchWidth;
      int newPatchImageHeight = patchImageHeight;
      int newPatchImageWidth = patchImageWidth;
      int newFilterPatchImageHeight = filterPatchImageHeight;
      int newFilterPatchImageWidth = filterPatchImageWidth;

      boolean changed = previousPatchHeight != newPatchHeight;
      changed |= previousPatchWidth != newPatchWidth;
      changed |= previousPatchImageHeight != newPatchImageHeight;
      changed |= previousPatchImageWidth != newPatchImageWidth;
      changed |= previousFilterPatchImageHeight != newFilterPatchImageHeight;
      changed |= previousFilterPatchImageWidth != newFilterPatchImageWidth;

      if (changed)
      {
         LogTools.info("Updated patch sizes:");
         LogTools.info("newPatchHeight: {} -> {}", previousPatchHeight, newPatchHeight);
         LogTools.info("newPatchWidth: {} -> {}", previousPatchWidth, newPatchWidth);
         LogTools.info("newPatchImageHeight: {} -> {}", previousPatchImageHeight, newPatchImageHeight);
         LogTools.info("newPatchImageWidth: {} -> {}", previousPatchImageWidth, newPatchImageWidth);
         LogTools.info("newFilterPatchImageHeight: {} -> {}", previousFilterPatchImageHeight, newFilterPatchImageHeight);
         LogTools.info("newFilterPatchImageWidth: {} -> {}", previousFilterPatchImageWidth, newFilterPatchImageWidth);
      }
   }

   public void destroy()
   {
      openCLManager.destroy();
      // TODO: Destroy the rest
   }

   public void setPatchSizeChanged(boolean patchSizeChanged)
   {
      this.patchSizeChanged = patchSizeChanged;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
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

   public RecyclingArrayList<GPUPlanarRegion> getGPUPlanarRegions()
   {
      return gpuPlanarRegions;
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

   public GPUPlanarRegionExtractionParameters getParameters()
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

   public class PatchGraphRecursionBlock
   {
      private final int row;
      private final int column;
      private final int planarRegionIslandIndex;
      private final GPUPlanarRegion planarRegion;
      private final int searchDepth;

      public PatchGraphRecursionBlock(int row, int column, int planarRegionIslandIndex, GPUPlanarRegion planarRegion, int searchDepth)
      {
         this.row = row;
         this.column = column;
         this.planarRegionIslandIndex = planarRegionIslandIndex;
         this.planarRegion = planarRegion;
         this.searchDepth = searchDepth;
      }

      public void expandBlock()
      {
         if (regionVisitedMatrix.get(row, column) || searchDepth > parameters.getSearchDepthLimit())
            return;

         if (searchDepth > regionMaxSearchDepth)
            regionMaxSearchDepth = searchDepth;

         ++numberOfRegionPatches;
         regionVisitedMatrix.set(row, column, true);
         regionMatrix.set(row, column, planarRegionIslandIndex);
         // kernel coordinates is in left-handed frame, so lets flip it to IHMC Z up
         float ny = -nxImage.getFloat(column, row);
         float nz = nyImage.getFloat(column, row);
         float nx = nzImage.getFloat(column, row);
         float cy = -cxImage.getFloat(column, row);
         float cz = cyImage.getFloat(column, row);
         float cx = czImage.getFloat(column, row);
         planarRegion.addRegionPatch(row, column, nx, ny, nz, cx, cy, cz);

         int count = 0;
         for (int i = 0; i < 8; i++)
         {
            if (row + adjacentY[i] < patchImageHeight - 1 && row + adjacentY[i] > 1 && column + adjacentX[i] < patchImageWidth - 1 && column + adjacentX[i] > 1)
            {
               int boundaryConnectionsEncodedAsOnes = patchGraph.getByteAsInteger((column + adjacentX[i]), (row + adjacentY[i]));
               if (boundaryConnectionsEncodedAsOnes == 255) // all ones; fully connected
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
            planarRegion.getBorderIndices().add().set(column, row);
         }
      }
   }
}

