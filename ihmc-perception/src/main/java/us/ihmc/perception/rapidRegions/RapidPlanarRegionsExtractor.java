package us.ihmc.perception.rapidRegions;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.ejml.data.BMatrixRMaj;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;

import java.net.URL;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.Comparator;
import java.util.List;
import java.util.Stack;

import static org.bytedeco.cuda.global.cudart.cudaFreeAsync;
import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

public class RapidPlanarRegionsExtractor
{
   private static final int BLOCK_SIZE_XY = 8;

   private final Stopwatch wholeAlgorithmDurationStopwatch = new Stopwatch();
   private final Stopwatch gpuDurationStopwatch = new Stopwatch();
   private final Stopwatch depthFirstSearchDurationStopwatch = new Stopwatch();

   private final RapidPlanarRegionsCustomizer rapidPlanarRegionsCustomizer;
   private final RapidRegionsExtractorParameters parameters;
   private final CameraIntrinsics cameraIntrinsics;
   private final CUstream_st stream;
   private final CUDAProgram rapidRegionsProgram;
   private final dim3 blockSize;

   private final CUDAKernel packKernel;
   private final CUDAKernel mergeKernel;

   private static final int NUM_GPU_PARAMETERS = 14;
   private final FloatPointer parametersHostPointer = new FloatPointer(NUM_GPU_PARAMETERS);
   private final FloatPointer parametersDevicePointer = new FloatPointer();

   /* Patch features - estimated patch normals and centroids */
   private GpuMat patchNormalsXDevice;
   private GpuMat patchNormalsYDevice;
   private GpuMat patchNormalsZDevice;
   private GpuMat patchCentroidsXDevice;
   private GpuMat patchCentroidsYDevice;
   private GpuMat patchCentroidsZDevice;
   private GpuMat patchConnectionsDevice;

   private final Mat patchNormalsXHost = new Mat();
   private final Mat patchNormalsYHost = new Mat();
   private final Mat patchNormalsZHost = new Mat();
   private final Mat patchCentroidsXHost = new Mat();
   private final Mat patchCentroidsYHost = new Mat();
   private final Mat patchCentroidsZHost = new Mat();
   private final Mat patchConnectionsHost = new Mat();

   // Computed patch features
   private FloatBuffer normalsXBuffer;
   private FloatBuffer normalsYBuffer;
   private FloatBuffer normalsZBuffer;
   private FloatBuffer centroidsXBuffer;
   private FloatBuffer centroidsYBuffer;
   private FloatBuffer centroidsZBuffer;
   private ShortBuffer connectionsBuffer;

   // Region finding
   private final BMatrixRMaj regionVisitedMatrix;
   private final BMatrixRMaj boundaryVisitedMatrix;
   private final BMatrixRMaj boundaryMatrix;
   private final DMatrixRMaj regionMatrix;

   private int numberOfRegionPatches = 0;
   private int regionMaxSearchDepth = 0;
   private int boundaryMaxSearchDepth = 0;
   private int numberOfBoundaryPatchesInWholeImage = 0;
   private double maxSVDSolveTime = Double.NaN;

   private final int[] adjacentY = {-1, -1, -1, 0, 0, 1, 1, 1};
   private final int[] adjacentX = {-1, 0, 1, -1, 1, -1, 0, 1};

   private int patchImageHeight;
   private int patchImageWidth;
   private int patchSize;

   private final RapidPatchesDebugOutputGenerator debugger = new RapidPatchesDebugOutputGenerator();
   private final Stack<PatchGraphRecursionBlock> depthFirstSearchStack = new Stack<>();
   private final RecyclingArrayList<RapidPlanarRegion> rapidPlanarRegions = new RecyclingArrayList<>(RapidPlanarRegion::new);
   private final Comparator<RapidRegionRing> boundaryLengthComparator = Comparator.comparingInt(regionRing -> regionRing.getBoundaryIndices().size());

   public RapidPlanarRegionsExtractor(CameraIntrinsics cameraIntrinsics)
   {
      this(cameraIntrinsics, "");
   }

   public RapidPlanarRegionsExtractor(CameraIntrinsics cameraIntrinsics, String version)
   {
      this.parameters = new RapidRegionsExtractorParameters(version);
      this.rapidPlanarRegionsCustomizer = new RapidPlanarRegionsCustomizer(version);
      this.cameraIntrinsics = cameraIntrinsics;

      calculateDerivativeParameters();
      stream = CUDAStreamManager.getStream();
      debugger.create(cameraIntrinsics.getHeight(), cameraIntrinsics.getWidth());

      URL utilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/Utils.cu");
      URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
      URL perceptionUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/PerceptionUtils.cu");
      URL kernelPath = getClass().getResource("RapidRegionsExtractor.cu");

      blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      try
      {
         rapidRegionsProgram = new CUDAProgram(kernelPath, utilsHeaderPath, mathUtilsHeaderPath, perceptionUtilsHeaderPath);

         packKernel = rapidRegionsProgram.loadKernel("packKernel");
         mergeKernel = rapidRegionsProgram.loadKernel("mergeKernel");
         createPatchMats();

         regionVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
         boundaryVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
         boundaryMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
         regionMatrix = new DMatrixRMaj(patchImageHeight, patchImageWidth);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   private void createPatchMats()
   {
      if (patchNormalsXDevice != null)
      {
         patchNormalsXDevice.close();
         patchNormalsYDevice.close();
         patchNormalsZDevice.close();
         patchCentroidsXDevice.close();
         patchCentroidsYDevice.close();
         patchCentroidsZDevice.close();
         patchConnectionsDevice.close();
      }

      patchNormalsXDevice = new GpuMat(patchImageHeight, patchImageWidth, opencv_core.CV_32FC1);
      patchNormalsYDevice = new GpuMat(patchImageHeight, patchImageWidth, opencv_core.CV_32FC1);
      patchNormalsZDevice = new GpuMat(patchImageHeight, patchImageWidth, opencv_core.CV_32FC1);
      patchCentroidsXDevice = new GpuMat(patchImageHeight, patchImageWidth, opencv_core.CV_32FC1);
      patchCentroidsYDevice = new GpuMat(patchImageHeight, patchImageWidth, opencv_core.CV_32FC1);
      patchCentroidsZDevice = new GpuMat(patchImageHeight, patchImageWidth, opencv_core.CV_32FC1);
      patchConnectionsDevice = new GpuMat(patchImageHeight, patchImageWidth, opencv_core.CV_16UC1);
   }

   public void update(GpuMat latestDepthImageGPU, ReferenceFrame cameraFrame, FramePlanarRegionsList frameRegions)
   {
      int error;

      if (parameters.getPatchSize() != patchSize)
      {
         createPatchMats();
      }

      debugger.clearDebugImage();
      wholeAlgorithmDurationStopwatch.start();
      gpuDurationStopwatch.start();

      // Populate parameter buffers
      float[] parametersArray = populateParameterArray(parameters, cameraIntrinsics);
      parametersHostPointer.put(parametersArray);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      int gridDimX = (patchImageWidth + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int gridDimY = (patchImageHeight + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 gridSize = new dim3(gridDimX, gridDimY, 1);

      // Run pack kernel, compute surface normal and centroid locally
      packKernel.withPointer(latestDepthImageGPU.data());
      packKernel.withPointer(patchNormalsXDevice.data());
      packKernel.withPointer(patchNormalsYDevice.data());
      packKernel.withPointer(patchNormalsZDevice.data());
      packKernel.withPointer(patchCentroidsXDevice.data());
      packKernel.withPointer(patchCentroidsYDevice.data());
      packKernel.withPointer(patchCentroidsZDevice.data());
      packKernel.withLong(latestDepthImageGPU.step());
      packKernel.withLong(patchNormalsXDevice.step());
      packKernel.withPointer(parametersDevicePointer);

      packKernel.run(stream, gridSize, blockSize, 0);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      mergeKernel.withPointer(patchNormalsXDevice.data());
      mergeKernel.withPointer(patchNormalsYDevice.data());
      mergeKernel.withPointer(patchNormalsZDevice.data());
      mergeKernel.withPointer(patchCentroidsXDevice.data());
      mergeKernel.withPointer(patchCentroidsYDevice.data());
      mergeKernel.withPointer(patchCentroidsZDevice.data());
      mergeKernel.withPointer(patchConnectionsDevice.data());
      mergeKernel.withLong(patchNormalsXDevice.step());
      mergeKernel.withLong(patchConnectionsDevice.step());
      mergeKernel.withPointer(parametersDevicePointer);
      mergeKernel.run(stream, gridSize, blockSize, 0);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      patchNormalsXDevice.download(patchNormalsXHost);
      patchNormalsYDevice.download(patchNormalsYHost);
      patchNormalsZDevice.download(patchNormalsZHost);
      patchCentroidsXDevice.download(patchCentroidsXHost);
      patchCentroidsYDevice.download(patchCentroidsYHost);
      patchCentroidsZDevice.download(patchCentroidsZHost);
      patchConnectionsDevice.download(patchConnectionsHost);

      normalsXBuffer = patchNormalsXHost.createBuffer();
      normalsYBuffer = patchNormalsYHost.createBuffer();
      normalsZBuffer = patchNormalsZHost.createBuffer();

      centroidsXBuffer = patchCentroidsXHost.createBuffer();
      centroidsYBuffer = patchCentroidsYHost.createBuffer();
      centroidsZBuffer = patchCentroidsZHost.createBuffer();

      connectionsBuffer = patchConnectionsHost.createBuffer();
      gpuDurationStopwatch.suspend();

      depthFirstSearchDurationStopwatch.start();
      findRegions();
      findBoundariesAndHoles();
      growRegionBoundaries();
      depthFirstSearchDurationStopwatch.suspend();

      rapidPlanarRegionsCustomizer.createCustomPlanarRegionsList(rapidPlanarRegions, cameraFrame, frameRegions);

      wholeAlgorithmDurationStopwatch.suspend();
//      debugger.update(depthImageCPU,
//                      currentFeatureGrid,
//                      patchGraph,
//                      cloudBuffer.getBackingDirectFloatBuffer(),
//                      cameraFrame.getTransformToWorldFrame());
   }

   /* for testing */ Vector3D[] getNormals()
   {
      Vector3D[] normals = new Vector3D[patchImageWidth * patchImageHeight];

      for (int i = 0; i < patchImageWidth; i++)
      {
         for (int j = 0; j < patchImageHeight; j++)
         {
            int index = i + patchImageWidth * j;
            normals[index] = new Vector3D(normalsXBuffer.get(index), normalsYBuffer.get(index), normalsZBuffer.get(index));
         }
      }

      return normals;
   }

   /* for testing */ Point3D[] getCentroids()
   {
      Point3D[] centroids = new Point3D[patchImageWidth * patchImageHeight];

      for (int i = 0; i < patchImageWidth; i++)
      {
         for (int j = 0; j < patchImageHeight; j++)
         {
            int index = i + patchImageWidth * j;
            centroids[index] = new Point3D(centroidsXBuffer.get(index), centroidsYBuffer.get(index), centroidsZBuffer.get(index));
         }
      }

      return centroids;
   }

   /**
    * Finds the connected regions in the patch graph using Depth First Search. It uses a heap-allocated stack object instead of process recursion stack.
    */
   private void findRegions()
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
            int index = globalIndex(column, row);
            int boundaryConnectionsEncodedAsOnes = connectionsBuffer.get(index);

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

   private int globalIndex(int column, int row)
   {
      return column + patchImageWidth * row;
   }

   private void findBoundariesAndHoles()
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
                                                           debugger.drawRegionRing(regionRing, parameters.getPatchSize());

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

   private void growRegionBoundaries()
   {
      rapidPlanarRegions.forEach(planarRegion ->
                                 {
                                    if (!planarRegion.getRegionRings().isEmpty())
                                    {
                                       RapidRegionRing firstRing = planarRegion.getRegionRings().get(0);
                                       for (Vector2D boundaryIndex : firstRing.getBoundaryIndices())
                                       {
                                          int index = globalIndex((int) boundaryIndex.getX(), (int) boundaryIndex.getY());
                                          float vertexX = centroidsXBuffer.get(index);
                                          float vertexY = centroidsYBuffer.get(index);
                                          float vertexZ = centroidsZBuffer.get(index);

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

   private static boolean checkConnectionThreshold(int nodeConnection, int threshold)
   {
      return Integer.bitCount(nodeConnection) > threshold;
   }

   private class PatchGraphRecursionBlock
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

         int index = globalIndex(column, row);
         float nx = normalsXBuffer.get(index);
         float ny = normalsYBuffer.get(index);
         float nz = normalsZBuffer.get(index);
         float cx = centroidsXBuffer.get(index);
         float cy = centroidsYBuffer.get(index);
         float cz = centroidsZBuffer.get(index);

         planarRegion.addRegionPatch(row, column, nx, ny, nz, cx, cy, cz);

         int count = 0;
         for (int i = 0; i < adjacentY.length; i++)
         {
            if (row + adjacentY[i] < patchImageHeight - 1 && row + adjacentY[i] > 1 && column + adjacentX[i] < patchImageWidth - 1 && column + adjacentX[i] > 1)
            {
               int indexNeighbor = globalIndex(column + adjacentX[i], row + adjacentY[i]);
               int boundaryConnectionsEncodedAsOnes = connectionsBuffer.get(indexNeighbor);
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
         }
      }
   }

   private void calculateDerivativeParameters()
   {
      patchSize = parameters.getPatchSize();
      patchImageHeight = cameraIntrinsics.getHeight() / patchSize;
      patchImageWidth = cameraIntrinsics.getWidth() / patchSize;
   }

   public float[] populateParameterArray(RapidRegionsExtractorParameters parameters, CameraIntrinsics cameraIntrinsics)
   {
      calculateDerivativeParameters();

      return new float[] {(float) cameraIntrinsics.getHeight(),
                          (float) cameraIntrinsics.getWidth(),
                          (float) patchImageHeight,
                          (float) patchImageWidth,
                          patchSize,
                          (float) cameraIntrinsics.getFx(),
                          (float) cameraIntrinsics.getFy(),
                          (float) cameraIntrinsics.getCx(),
                          (float) cameraIntrinsics.getCy(),
                          (float) parameters.getNormalPackRange(),
                          (float) parameters.getMergeRange(),
                          (float) parameters.getMergeAngularThreshold(),
                          (float) parameters.getMergeOrthogonalThreshold(),
                          (float) parameters.getMergeDistanceThreshold()};
   }

   public RapidRegionsExtractorParameters getRapidRegionsExtractorParameters()
   {
      return parameters;
   }

   public ConcaveHullFactoryParameters getConcaveHullFactoryParameters()
   {
      return rapidPlanarRegionsCustomizer.getConcaveHullFactoryParameters();
   }

   public PolygonizerParameters getPolygonizerParameters()
   {
      return rapidPlanarRegionsCustomizer.getPolygonizerParameters();
   }

   public RapidPlanarRegionsCustomizer getRapidPlanarRegionsCustomizer()
   {
      return rapidPlanarRegionsCustomizer;
   }

   public int getPatchImageHeight()
   {
      return patchImageHeight;
   }

   public int getPatchImageWidth()
   {
      return patchImageWidth;
   }

   public CameraIntrinsics getCameraIntrinsics()
   {
      return cameraIntrinsics;
   }

   public List<RapidPlanarRegion> getRapidPlanarRegions()
   {
      return rapidPlanarRegions;
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

   public double getMaxSVDSolveTime()
   {
      return maxSVDSolveTime;
   }

   public RapidPatchesDebugOutputGenerator getDebugger()
   {
      return debugger;
   }

   public void destroy()
   {
      rapidRegionsProgram.close();
      blockSize.close();

      packKernel.close();
      mergeKernel.close();

      patchNormalsXDevice.close();
      patchNormalsYDevice.close();
      patchNormalsZDevice.close();
      patchCentroidsXDevice.close();
      patchCentroidsYDevice.close();
      patchCentroidsZDevice.close();
      patchConnectionsDevice.close();

      parametersHostPointer.close();
      parametersDevicePointer.close();
      cudaFreeAsync(parametersDevicePointer, stream);

      CUDAStreamManager.releaseStream(stream);
   }
}
