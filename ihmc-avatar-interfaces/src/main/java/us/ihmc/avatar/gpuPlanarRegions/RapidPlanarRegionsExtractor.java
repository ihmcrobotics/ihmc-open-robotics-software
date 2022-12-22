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

import java.nio.ByteBuffer;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Stack;
import java.util.function.Consumer;

import static us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters.*;

public class RapidPlanarRegionsExtractor
{
   private enum FeatureGridType
   {
      PATCH_NORMAL_X, PATCH_NORMAL_Y, PATCH_NORMAL_Z, PATCH_CENTER_X, PATCH_CENTER_Y, PATCH_CENTER_Z
   }

   private final FeatureGridType[] featureGridTypes = {FeatureGridType.PATCH_NORMAL_X,
                                                       FeatureGridType.PATCH_NORMAL_Y,
                                                       FeatureGridType.PATCH_NORMAL_Z,
                                                       FeatureGridType.PATCH_CENTER_X,
                                                       FeatureGridType.PATCH_CENTER_Y,
                                                       FeatureGridType.PATCH_CENTER_Z};

   private final GPUPlanarRegionExtractionParameters parameters;

   private BytedecoImage inputU16DepthImage;

   HashMap<FeatureGridType, BytedecoImage> gridImages = new HashMap<>();
   private BytedecoImage patchGraph;

   private BMatrixRMaj regionVisitedMatrix;
   private BMatrixRMaj boundaryVisitedMatrix;
   private BMatrixRMaj boundaryMatrix;
   private DMatrixRMaj regionMatrix;

   private boolean patchSizeChanged = false;

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
   private final Stack<GPUPlanarRegionExtractionDepthFirstSearchQuery> depthFirstSearchStack = new Stack<>();

   private final RecyclingArrayList<GPUPlanarRegion> gpuPlanarRegions = new RecyclingArrayList<>(GPUPlanarRegion::new);
   private final Comparator<GPURegionRing> boundaryLengthComparator = Comparator.comparingInt(regionRing -> regionRing.getBoundaryIndices().size());

   private final OpenCLManager openCLManager = new OpenCLManager();
   private OpenCLFloatBuffer parametersBuffer;
   private _cl_program planarRegionExtractionProgram;
   private _cl_kernel filterKernel;
   private _cl_kernel packKernel;
   private _cl_kernel mergeKernel;

   private final PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private final PlanarRegionsListWithPose planarRegionsListWithPose = new PlanarRegionsListWithPose();
   private final RigidBodyTransform sensorToWorldFrameTransform = new RigidBodyTransform();
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
   public void create(int imageWidth, int imageHeight, ByteBuffer sourceDepthByteBufferOfFloats, double fx, double fy, double cx, double cy)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;

      parameters.set(focalLengthXPixels, fx);
      parameters.set(focalLengthYPixels, fy);
      parameters.set(principalOffsetXPixels, cx);
      parameters.set(principalOffsetYPixels, cy);

      parametersBuffer = new OpenCLFloatBuffer(16);
      calculateDerivativeParameters();
      inputU16DepthImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);

      for (FeatureGridType type : featureGridTypes)
      {
         gridImages.put(type, new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1));
      }

      patchGraph = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_8UC1);

      openCLManager.create();
      planarRegionExtractionProgram = openCLManager.loadProgram("PlanarRegionExtraction");
      packKernel = openCLManager.createKernel(planarRegionExtractionProgram, "packKernel");
      mergeKernel = openCLManager.createKernel(planarRegionExtractionProgram, "mergeKernel");

      regionVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      regionMatrix = new DMatrixRMaj(patchImageHeight, patchImageWidth);
   }

   /**
    * Extracts features and generates patch graph from the input depth image on the GPU.
    *
    */
   public void extractPatchGraphOnGPU()
   {
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

         for (FeatureGridType type : featureGridTypes)
         {
            gridImages.get(type).resize(patchImageWidth, patchImageHeight, openCLManager, null);
         }
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

         for (FeatureGridType type : featureGridTypes)
         {
            gridImages.get(type).createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         }
         patchGraph.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         parametersBuffer.createOpenCLBufferObject(openCLManager);
      }
      else
      {
         inputU16DepthImage.writeOpenCLImage(openCLManager);
         parametersBuffer.writeOpenCLBufferObject(openCLManager);
      }

      _cl_mem inputImage = inputU16DepthImage.getOpenCLImageObject();

      int argIndex = 0;
      openCLManager.setKernelArgument(packKernel, argIndex++, inputImage);
      for (FeatureGridType type : featureGridTypes)
      {
         openCLManager.setKernelArgument(packKernel, argIndex++, gridImages.get(type).getOpenCLImageObject());
      }
      openCLManager.setKernelArgument(packKernel, argIndex++, parametersBuffer.getOpenCLBufferObject());

      argIndex = 0;
      for (FeatureGridType type : featureGridTypes)
      {
         openCLManager.setKernelArgument(mergeKernel, argIndex++, gridImages.get(type).getOpenCLImageObject());
      }
      openCLManager.setKernelArgument(mergeKernel, argIndex++, patchGraph.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, argIndex++, parametersBuffer.getOpenCLBufferObject());

      openCLManager.execute2D(packKernel, patchImageHeight, patchImageWidth);
      openCLManager.execute2D(mergeKernel, patchImageHeight, patchImageWidth);

      for (FeatureGridType type : featureGridTypes)
      {
         openCLManager.enqueueReadImage(gridImages.get(type).getOpenCLImageObject(),
                                        patchImageWidth,
                                        patchImageHeight,
                                        gridImages.get(type).getBytedecoByteBufferPointer());
      }
      openCLManager.enqueueReadImage(patchGraph.getOpenCLImageObject(), patchImageWidth, patchImageHeight, patchGraph.getBytedecoByteBufferPointer());

      openCLManager.finish();
   }

   /**
    *
    *
    * @param forDrawingDebugPanel Consumer for drawing the debug panel.
    */
   public void findRegions(Consumer<GPUPlanarRegionIsland> forDrawingDebugPanel)
   {
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
               // We use a stack object instead of using the JVM's stack because it's very tall and would require special
               // settings and breaks profiling. It would crash YourKit.
               depthFirstSearchStack.push(new GPUPlanarRegionExtractionDepthFirstSearchQuery(row, column, planarRegionIslandIndex, planarRegion, 1));
               while (!depthFirstSearchStack.empty())
               {
                  depthFirstSearchStack.pop().performQuery();
               }
               if (numberOfRegionPatches >= parameters.getRegionMinPatches())
               {
                  ++planarRegionIslandIndex;
                  planarRegion.update(parameters.getUseSVDNormals(), parameters.getSVDReductionFactor());
                  if (planarRegion.getSVDDuration() > maxSVDSolveTime)
                     maxSVDSolveTime = planarRegion.getSVDDuration();

                  tempIsland.planarRegion = planarRegion;
                  tempIsland.planarRegionIslandIndex = planarRegionIslandIndex;
                  if (forDrawingDebugPanel != null)
                     forDrawingDebugPanel.accept(tempIsland);
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
   }

   public class GPUPlanarRegionExtractionDepthFirstSearchQuery
   {
      private final int row;
      private final int column;
      private final int planarRegionIslandIndex;
      private final GPUPlanarRegion planarRegion;
      private final int searchDepth;

      public GPUPlanarRegionExtractionDepthFirstSearchQuery(int row, int column, int planarRegionIslandIndex, GPUPlanarRegion planarRegion, int searchDepth)
      {
         this.row = row;
         this.column = column;
         this.planarRegionIslandIndex = planarRegionIslandIndex;
         this.planarRegion = planarRegion;
         this.searchDepth = searchDepth;
      }

      public void performQuery()
      {
         if (regionVisitedMatrix.get(row, column) || searchDepth > parameters.getSearchDepthLimit())
            return;

         if (searchDepth > regionMaxSearchDepth)
            regionMaxSearchDepth = searchDepth;

         ++numberOfRegionPatches;
         regionVisitedMatrix.set(row, column, true);
         regionMatrix.set(row, column, planarRegionIslandIndex);
         // kernel coordinates is in left-handed frame, so lets flip it to IHMC Z up
         float ny = -gridImages.get(FeatureGridType.PATCH_NORMAL_X).getFloat(column, row);
         float nz = gridImages.get(FeatureGridType.PATCH_NORMAL_Y).getFloat(column, row);
         float nx = gridImages.get(FeatureGridType.PATCH_NORMAL_Z).getFloat(column, row);
         float cy = -gridImages.get(FeatureGridType.PATCH_CENTER_X).getFloat(column, row);
         float cz = gridImages.get(FeatureGridType.PATCH_CENTER_Y).getFloat(column, row);
         float cx = gridImages.get(FeatureGridType.PATCH_CENTER_Z).getFloat(column, row);
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
                  depthFirstSearchStack.push(new GPUPlanarRegionExtractionDepthFirstSearchQuery(row + adjacentY[i],
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

   public void findBoundariesAndHoles(Consumer<GPURegionRing> forDrawingDebugPanel)
   {
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
                                                         if (forDrawingDebugPanel != null)
                                                            forDrawingDebugPanel.accept(regionRing);
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

   public void growRegionBoundaries()
   {
      gpuPlanarRegions.forEach(planarRegion ->
                               {
                                  if (!planarRegion.getRegionRings().isEmpty())
                                  {
                                     GPURegionRing firstRing = planarRegion.getRegionRings().get(0);
                                     for (Vector2D boundaryIndex : firstRing.getBoundaryIndices())
                                     {
                                        // kernel coordinates is in left-handed frame, so lets flip it to IHMC Z up
                                        float vertexX = gridImages.get(FeatureGridType.PATCH_CENTER_Z)
                                                                  .getBytedecoOpenCVMat()
                                                                  .ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX())
                                                                  .getFloat();
                                        float vertexY = -gridImages.get(FeatureGridType.PATCH_CENTER_X)
                                                                   .getBytedecoOpenCVMat()
                                                                   .ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX())
                                                                   .getFloat();
                                        float vertexZ = gridImages.get(FeatureGridType.PATCH_CENTER_Y)
                                                                  .getBytedecoOpenCVMat()
                                                                  .ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX())
                                                                  .getFloat();

                                        Vector3D boundaryVertex = planarRegion.getBoundaryVertices().add();
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

   public PlanarRegionsListWithPose getPlanarRegionsListWithPose()
   {
      return planarRegionsListWithPose;
   }

   public int getPatchImageWidth()
   {
      return patchImageWidth;
   }

   public int getPatchImageHeight()
   {
      return patchImageHeight;
   }

   public OpenCLManager getOpenCLManager()
   {
      return openCLManager;
   }

   public int getNumberOfBoundaryPatchesInWholeImage()
   {
      return numberOfBoundaryPatchesInWholeImage;
   }

   public BytedecoImage getNxImage()
   {
      return gridImages.get(FeatureGridType.PATCH_NORMAL_X);
   }

   public BytedecoImage getNyImage()
   {
      return gridImages.get(FeatureGridType.PATCH_NORMAL_Y);
   }

   public BytedecoImage getNzImage()
   {
      return gridImages.get(FeatureGridType.PATCH_NORMAL_Z);
   }

   public BytedecoImage getCxImage()
   {
      return gridImages.get(FeatureGridType.PATCH_CENTER_X);
   }

   public BytedecoImage getCyImage()
   {
      return gridImages.get(FeatureGridType.PATCH_CENTER_Y);
   }

   public BytedecoImage getCzImage()
   {
      return gridImages.get(FeatureGridType.PATCH_CENTER_Z);
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
}

