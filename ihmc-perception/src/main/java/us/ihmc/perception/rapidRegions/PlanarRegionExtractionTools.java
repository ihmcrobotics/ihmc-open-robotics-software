package us.ihmc.perception.rapidRegions;

import org.bytedeco.opencv.opencv_core.Mat;
import org.ejml.data.BMatrixRMaj;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.LinearTransform3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.geometry.*;
import us.ihmc.perception.segmentationTools.PolygonizerParameters;
import us.ihmc.perception.segmentationTools.PolygonizerTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class PlanarRegionExtractionTools
{
   private static final boolean useParallelStreams = true;
   private static final int directions = 8;

   private final BMatrixRMaj regionVisitedMatrix = new BMatrixRMaj(0, 0);
   private final BMatrixRMaj boundaryVisitedMatrix = new BMatrixRMaj(0, 0);
   private final BMatrixRMaj boundaryMatrix = new BMatrixRMaj(0, 0);
   private final DMatrixRMaj regionIdMatrix = new DMatrixRMaj(0, 0);
   private final int[] adjacentY = {-1, 0, 1, 1, 1, 0, -1, -1};
   private final int[] adjacentX = {-1, -1, -1, 0, 1, 1, 1, 0};

   private int regionMaxSearchDepth;
   private int numberOfRegionPatches = 0;
   private int boundaryMaxSearchDepth = 0;
   private double maxSVDSolveTime = Double.NaN;


   private int numberOfBoundaryPatchesInWholeImage = 0;

   private ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters("ForGPURegions");
   private PolygonizerParameters polygonizerParameters = new PolygonizerParameters("ForGPURegions");
   private GPUPlanarRegionExtractionParameters parameters;

   private final PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private final Comparator<GPURegionRing> boundaryIndexComparator = Comparator.comparingInt(regionRing -> regionRing.getBoundaryIndices().size());

   private final GPUPlanarRegionIsland tempIsland = new GPUPlanarRegionIsland();

   private final RecyclingArrayList<GPUPlanarRegion> gpuPlanarRegions = new RecyclingArrayList<>(GPUPlanarRegion::new);

   public void createRegionsFromConnectedCells(PlanarRegionExtractionInputData data, Consumer<GPUPlanarRegionIsland> forDrawingDebugPanel)
   {
      int planarRegionIslandIndex = 0;
      regionMaxSearchDepth = 0;
      numberOfBoundaryPatchesInWholeImage = 0;
      gpuPlanarRegions.clear();

      regionVisitedMatrix.reshape(data.imageHeight, data.imageWidth);
      boundaryMatrix.reshape(data.imageHeight, data.imageWidth);
      regionIdMatrix.reshape(data.imageHeight, data.imageWidth);

      regionVisitedMatrix.zero();
      boundaryMatrix.zero();
      regionIdMatrix.zero();


      maxSVDSolveTime = 0.0;
      for (int row = 0; row < data.imageHeight; row++)
      {
         for (int column = 0; column < data.imageWidth; column++)
         {
            int boundaryConnectionsEncodedAsOnes = Byte.toUnsignedInt(data.graphImage.ptr(row, column).get());
            if (!regionVisitedMatrix.get(row, column) && boundaryConnectionsEncodedAsOnes == 255) // all ones; fully connected
            {
               numberOfRegionPatches = 0; // also number of patches traversed
               GPUPlanarRegion planarRegion = gpuPlanarRegions.add();
               planarRegion.reset(planarRegionIslandIndex);
               connectCellToNeighborsInRegion(data, row, column, planarRegionIslandIndex, planarRegion, 1);

               if (numberOfRegionPatches >= parameters.getRegionMinPatches())
               {
                  ++planarRegionIslandIndex;
                  planarRegion.update(parameters.getUseSVDNormals(), parameters.getSVDReductionFactor());
                  if (planarRegion.getSVDDuration() > maxSVDSolveTime)
                     maxSVDSolveTime = planarRegion.getSVDDuration();

                  if (forDrawingDebugPanel != null)
                  {
                     tempIsland.planarRegion = planarRegion;
                     tempIsland.planarRegionIslandIndex = planarRegionIslandIndex;
                     forDrawingDebugPanel.accept(tempIsland);
                  }
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

   private void connectCellToNeighborsInRegion(PlanarRegionExtractionInputData data,
                                               int row,
                                               int column,
                                               int planarRegionIslandIndex,
                                               GPUPlanarRegion planarRegionToPack,
                                               int searchDepth)
   {
      if (regionVisitedMatrix.get(row, column) || searchDepth > parameters.getSearchDepthLimit())
         return;

      if (searchDepth > regionMaxSearchDepth)
         regionMaxSearchDepth = searchDepth;

      ++numberOfRegionPatches;
      regionVisitedMatrix.set(row, column, true);
      regionIdMatrix.set(row, column, planarRegionIslandIndex);

      // TODO skip if the count is too low

      // kernel coordinates is in left-handed frame, so lets flip it to IHMC Z up
      float nx, ny, nz;
      float cx, cy, cz;
      if (data.isDataInImageFrame)
      {
         ny = -data.normalXImage.ptr(row, column).getFloat();
         nz = data.normalYImage.ptr(row, column).getFloat();
         nx = data.normalZImage.ptr(row, column).getFloat();
         cy = -data.centroidXImage.ptr(row, column).getFloat();
         cz = data.centroidYImage.ptr(row, column).getFloat();
         cx = data.centroidZImage.ptr(row, column).getFloat();
      }
      else
      {
         nx = data.normalXImage.ptr(row, column).getFloat();
         ny = data.normalYImage.ptr(row, column).getFloat();
         nz = data.normalZImage.ptr(row, column).getFloat();
         cx = data.centroidXImage.ptr(row, column).getFloat();
         cy = data.centroidYImage.ptr(row, column).getFloat();
         cz = data.centroidZImage.ptr(row, column).getFloat();
      }
      planarRegionToPack.addRegionPatch(row, column, nx, ny, nz, cx, cy, cz);

      int count = 0;
      for (int i = 0; i < directions; i++)
      {
         if (isNeighborInBounds(row, column, i, data.imageHeight, data.imageWidth))
         {
            int boundaryConnectionsEncodedAsOnes = Byte.toUnsignedInt(data.graphImage.ptr((row + adjacentY[i]), (column + adjacentX[i])).get());
            if (boundaryConnectionsEncodedAsOnes == 255) // all ones; fully connected
            {
               count++;
               connectCellToNeighborsInRegion(data, row + adjacentY[i], column + adjacentX[i], planarRegionIslandIndex, planarRegionToPack, searchDepth + 1);
            }
         }
      }
      if (count != directions)
      {
         // this cell is not fully connected, meaning it's not on an edge
         boundaryMatrix.set(row, column, true);
         planarRegionToPack.getBorderIndices().add().set(column, row);
      }
   }

   private boolean isNeighborInBounds(int row, int column, int direction, int imageHeight, int imageWidth)
   {
      return row + adjacentY[direction] < imageHeight - 1 && row + adjacentY[direction] > 1 && column + adjacentX[direction] < imageWidth - 1
             && column + adjacentX[direction] > 1;
   }

   private boolean isNeighborSameRegion(int row, int column, int direction, int currentId)
   {
      return currentId == regionIdMatrix.get(row + adjacentY[direction], column + adjacentX[direction]);
   }

   public void findBoundariesAndHoles(PlanarRegionExtractionInputData data, Consumer<GPURegionRing> forDrawingDebugPanel)
   {
      boundaryVisitedMatrix.zero();
      boundaryMaxSearchDepth = 0;
      Stream<GPUPlanarRegion> stream = useParallelStreams ? gpuPlanarRegions.parallelStream() : gpuPlanarRegions.stream();
      stream.forEach(planarRegion -> connectAllBoundaryRingsInRegion(data, planarRegion, forDrawingDebugPanel));
   }

   private void connectAllBoundaryRingsInRegion(PlanarRegionExtractionInputData data,
                                                GPUPlanarRegion planarRegion,
                                                Consumer<GPURegionRing> forDrawingDebugPanel)
   {
      int regionRingIndex = 0;
      // recurse through all the cells along the border
      for (Point2D leafPatch : planarRegion.getBorderIndices())
      {
         GPURegionRing regionRing = planarRegion.getRegionRings().add();
         regionRing.reset();
         regionRing.setIndex(regionRingIndex);
         // the x and y values of the point already indices, so casting to int loses no data.
         int numberOfBoundaryPatches = boundaryDepthFirstSearch(data, (int) leafPatch.getY(), (int) leafPatch.getX(), planarRegion.getId(), regionRing, 1);
         if (numberOfBoundaryPatches >= parameters.getBoundaryMinPatches())
         {
            if (forDrawingDebugPanel != null)
               forDrawingDebugPanel.accept(regionRing);
            ++regionRingIndex;
         }
         else
         {
            planarRegion.getRegionRings().remove(planarRegion.getRegionRings().size() - 1);
         }
      }
      planarRegion.getRegionRings().sort(boundaryIndexComparator);
   }

   private int boundaryDepthFirstSearch(PlanarRegionExtractionInputData data,
                                        int row,
                                        int column,
                                        int planarRegionId,
                                        GPURegionRing regionRing,
                                        int searchDepth)
   {
      if (boundaryVisitedMatrix.get(row, column) || searchDepth > parameters.getSearchDepthLimit())
         return 0;

      if (searchDepth > boundaryMaxSearchDepth)
         boundaryMaxSearchDepth = searchDepth;

      ++numberOfBoundaryPatchesInWholeImage;
      boundaryVisitedMatrix.set(row, column, true);
      regionRing.getBoundaryIndices().add().set(column, row);

      int numberOfBoundaryPatches = 1;
      for (int i = 0; i < directions; i++)
      {
         boolean isNeighorOnTheSameBoundary = isNeighborInBounds(row, column, i, data.imageHeight, data.imageWidth);
         isNeighorOnTheSameBoundary &= isNeighborSameRegion(row, column, i, planarRegionId);
         isNeighorOnTheSameBoundary &= boundaryMatrix.get(row + adjacentY[i], column + adjacentX[i]); // check that the neighbor is also a boundary cell

         if (isNeighorOnTheSameBoundary)
         {
            numberOfBoundaryPatches += boundaryDepthFirstSearch(data, row + adjacentY[i], column + adjacentX[i], planarRegionId, regionRing, searchDepth + 1);
         }
      }
      return numberOfBoundaryPatches;
   }

   public void growRegionBoundaries(PlanarRegionExtractionInputData data)
   {
      // this should be optional.
      Stream<GPUPlanarRegion> stream = useParallelStreams ? gpuPlanarRegions.parallelStream() : gpuPlanarRegions.stream();
      stream.forEach(planarRegion -> growRegionBoundary(data, planarRegion));
   }

   private void growRegionBoundary(PlanarRegionExtractionInputData data, GPUPlanarRegion planarRegion)
   {
      // this should be optional.
      if (!planarRegion.getRegionRings().isEmpty())
      {
         GPURegionRing firstRing = planarRegion.getRegionRings().get(0);
         for (Vector2D boundaryIndex : firstRing.getBoundaryIndices())
         {
            float vertexX, vertexY, vertexZ;
            if (data.isDataInImageFrame)
            {
               // kernel coordinates are in left-handed frame, so lets flip it to IHMC Z up
               vertexX = data.centroidZImage.ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX()).getFloat();
               vertexY = -data.centroidXImage.ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX()).getFloat();
               vertexZ = data.centroidYImage.ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX()).getFloat();
            }
            else
            {
               vertexX = data.centroidXImage.ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX()).getFloat();
               vertexY = data.centroidYImage.ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX()).getFloat();
               vertexZ = data.centroidZImage.ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX()).getFloat();
            }

            Vector3D boundaryVertex = planarRegion.getBoundaryVertices().add();
            boundaryVertex.set(vertexX, vertexY, vertexZ);
            boundaryVertex.sub(planarRegion.getCenter());
            boundaryVertex.normalize();
            boundaryVertex.scale(parameters.getRegionGrowthFactor());
            boundaryVertex.add(vertexX, vertexY, vertexZ);
         }
      }
   }

   public void computePlanarRegions(ReferenceFrame cameraFrame)
   {
      List<List<PlanarRegion>> listOfListsOfRegions = gpuPlanarRegions.parallelStream()
                                                                      .filter(gpuPlanarRegion -> gpuPlanarRegion.getBoundaryVertices().size()
                                                                                                 >= polygonizerParameters.getMinNumberOfNodes())
                                                                      .map(gpuPlanarRegion ->
                                                                           {
                                                                              List<PlanarRegion> planarRegions = new ArrayList<>();
                                                                              try
                                                                              {
                                                                                 // Going through LinearTransform3D first prevents NotARotationMatrix exceptions.
                                                                                 LinearTransform3D linearTransform3D = new LinearTransform3D(EuclidGeometryTools.axisAngleFromZUpToVector3D(
                                                                                       gpuPlanarRegion.getNormal()));
                                                                                 linearTransform3D.normalize();
                                                                                 FrameQuaternion orientation = new FrameQuaternion();
                                                                                 orientation.setIncludingFrame(cameraFrame,
                                                                                                               linearTransform3D.getAsQuaternion());
                                                                                 orientation.changeFrame(ReferenceFrame.getWorldFrame());

                                                                                 // First compute the set of concave hulls for this region
                                                                                 FramePoint3D origin = new FramePoint3D(cameraFrame,
                                                                                                                        gpuPlanarRegion.getCenter());
                                                                                 origin.changeFrame(ReferenceFrame.getWorldFrame());

                                                                                 List<Point2D> pointCloudInPlane = gpuPlanarRegion.getBoundaryVertices()
                                                                                                                                  .stream()
                                                                                                                                  .map(boundaryVertex ->
                                                                                                                                       {
                                                                                                                                          FramePoint3D framePoint3D = new FramePoint3D(
                                                                                                                                                cameraFrame,
                                                                                                                                                boundaryVertex);
                                                                                                                                          framePoint3D.changeFrame(
                                                                                                                                                ReferenceFrame.getWorldFrame());
                                                                                                                                          return PolygonizerTools.toPointInPlane(
                                                                                                                                                framePoint3D,
                                                                                                                                                origin,
                                                                                                                                                orientation);
                                                                                                                                       })
                                                                                                                                  .filter(point2D ->
                                                                                                                                                Double.isFinite(
                                                                                                                                                      point2D.getX())
                                                                                                                                                && Double.isFinite(
                                                                                                                                                      point2D.getY()))
                                                                                                                                  .collect(Collectors.toList());
                                                                                 List<LineSegment2D> intersections = new ArrayList<>();
                                                                                 //                     = intersections.stream()
                                                                                 //                  .map(intersection -> PolygonizerTools.toLineSegmentInPlane(lineSegmentInWorld, origin, orientation))
                                                                                 //                  .collect(Collectors.toList());
                                                                                 ConcaveHullCollection concaveHullCollection = SimpleConcaveHullFactory.createConcaveHullCollection(
                                                                                       pointCloudInPlane,
                                                                                       intersections,
                                                                                       concaveHullFactoryParameters);

                                                                                 // Apply some simple filtering to reduce the number of vertices and hopefully the number of convex polygons.
                                                                                 double shallowAngleThreshold = polygonizerParameters.getShallowAngleThreshold();
                                                                                 double peakAngleThreshold = polygonizerParameters.getPeakAngleThreshold();
                                                                                 double lengthThreshold = polygonizerParameters.getLengthThreshold();

                                                                                 ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(
                                                                                       shallowAngleThreshold,
                                                                                       peakAngleThreshold,
                                                                                       concaveHullCollection);
                                                                                 ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold,
                                                                                                                                      concaveHullCollection);
                                                                                 if (polygonizerParameters.getCutNarrowPassage())
                                                                                    concaveHullCollection = ConcaveHullPruningFilteringTools.concaveHullNarrowPassageCutter(
                                                                                          lengthThreshold,
                                                                                          concaveHullCollection);

                                                                                 int hullCounter = 0;
                                                                                 int regionId = gpuPlanarRegion.getId();

                                                                                 for (ConcaveHull concaveHull : concaveHullCollection)
                                                                                 {
                                                                                    if (concaveHull.isEmpty())
                                                                                       continue;

                                                                                    // Decompose the concave hulls into convex polygons
                                                                                    double depthThreshold = polygonizerParameters.getDepthThreshold();
                                                                                    List<ConvexPolygon2D> decomposedPolygons = new ArrayList<>();
                                                                                    ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHull,
                                                                                                                                               depthThreshold,
                                                                                                                                               decomposedPolygons);

                                                                                    // Pack the data in PlanarRegion
                                                                                    FramePose3D regionPose = new FramePose3D();
                                                                                    regionPose.setIncludingFrame(ReferenceFrame.getWorldFrame(),
                                                                                                                 origin,
                                                                                                                 orientation);
                                                                                    RigidBodyTransform tempTransform = new RigidBodyTransform();
                                                                                    regionPose.get(tempTransform);
                                                                                    PlanarRegion planarRegion = new PlanarRegion(tempTransform,
                                                                                                                                 concaveHull.getConcaveHullVertices(),
                                                                                                                                 decomposedPolygons);
                                                                                    planarRegion.setRegionId(regionId);
                                                                                    planarRegions.add(planarRegion);

                                                                                    hullCounter++;
                                                                                    regionId = 31 * regionId + hullCounter;
                                                                                 }
                                                                              }
                                                                              catch (RuntimeException e)
                                                                              {
                                                                                 e.printStackTrace();
                                                                              }
                                                                              return planarRegions;
                                                                           })
                                                                      .collect(Collectors.toList());
      planarRegionsList.clear();
      for (List<PlanarRegion> planarRegions : listOfListsOfRegions)
      {
         planarRegionsList.addPlanarRegions(planarRegions);
      }
   }

   public static class PlanarRegionExtractionInputData
   {
      public Mat normalXImage;
      public Mat normalYImage;
      public Mat normalZImage;
      public Mat centroidXImage;
      public Mat centroidYImage;
      public Mat centroidZImage;
      public Mat graphImage;
      public boolean isDataInImageFrame;

      public int imageHeight;
      public int imageWidth;
   }
}
