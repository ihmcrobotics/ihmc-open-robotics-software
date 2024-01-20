package us.ihmc.perception.steppableRegions;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.steppableRegions.data.SteppableBorderRing;
import us.ihmc.perception.steppableRegions.data.SteppableCell;
import us.ihmc.perception.steppableRegions.data.SteppableRegionDataHolder;
import us.ihmc.perception.steppableRegions.data.SteppableRegionsEnvironmentModel;
import us.ihmc.robotEnvironmentAwareness.geometry.*;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

public class SteppableRegionsCalculator
{
   private static final int maxRecursionDepth = 500;

   public static SteppableRegionsEnvironmentModel createEnvironmentByMergingCellsIntoRegions(BytedecoImage steppability,
                                                                                             BytedecoImage snappedHeight,
                                                                                             BytedecoImage snappedNormalX,
                                                                                             BytedecoImage snappedNormalY,
                                                                                             BytedecoImage snappedNormalZ,
                                                                                             BytedecoImage connections,
                                                                                             SteppableRegionCalculatorParametersReadOnly parameters,
                                                                                             HeightMapData heightMapData)
   {
      return createEnvironmentByMergingCellsIntoRegions(steppability,
                                                        snappedHeight,
                                                        snappedNormalX,
                                                        snappedNormalY,
                                                        snappedNormalZ,
                                                        connections,
                                                        parameters,
                                                        heightMapData.getGridCenter().getX(),
                                                        heightMapData.getGridCenter().getY(),
                                                        heightMapData.getGridResolutionXY(),
                                                        heightMapData.getCenterIndex());
   }

   public static SteppableRegionsEnvironmentModel createEnvironmentByMergingCellsIntoRegions(BytedecoImage steppability,
                                                                                             BytedecoImage snappedHeight,
                                                                                             BytedecoImage snappedNormalX,
                                                                                             BytedecoImage snappedNormalY,
                                                                                             BytedecoImage snappedNormalZ,
                                                                                             BytedecoImage connections,
                                                                                             SteppableRegionCalculatorParametersReadOnly parameters,
                                                                                             double gridCenterX,
                                                                                             double gridCenterY,
                                                                                             double gridResolutionXY,
                                                                                             int centerIndex)
   {
      SteppableRegionsEnvironmentModel environmentModel = createUnsortedSteppableRegionEnvironment(steppability,
                                                                                                   snappedHeight,
                                                                                                   snappedNormalX,
                                                                                                   snappedNormalY,
                                                                                                   snappedNormalZ,
                                                                                                   connections);

      if (steppability.getImageHeight() != steppability.getImageWidth())
         throw new RuntimeException("The input steppability should be square");

      while (environmentModel.hasUnexpandedBorderCells())
      {
         SteppableCell unexpandedCell = environmentModel.getNextUnexpandedBorderCell();
         if (!unexpandedCell.cellHasBeenAssigned())
         {
            // If this cell hasn't been assigned, we're expanding in a new region. Sometimes, when the depth limit of the search is reached, the cell may have
            // already been assigned, but never got expanded.
            SteppableRegionDataHolder region = environmentModel.createNewSteppableRegion();
            region.addCell(unexpandedCell, gridCenterX, gridCenterY, gridResolutionXY, centerIndex);

            SteppableBorderRing borderRing = region.createNewBorderRing();
            borderRing.addCell(unexpandedCell);
         }

         recursivelyAddBorderNeighbors(unexpandedCell, environmentModel,
                                       maxRecursionDepth, 0, gridCenterX, gridCenterY, gridResolutionXY, centerIndex);
      }

      while (environmentModel.hasUnexpandedInteriorCells())
      {
         // Start assuming we're expanding in a new region
         SteppableCell unexpandedCell = environmentModel.getNextUnexpandedCell();
         if (unexpandedCell == null)
            break;

         if (!unexpandedCell.cellHasBeenAssigned())
         {
            SteppableRegionDataHolder region = environmentModel.createNewSteppableRegion();
            region.addCell(unexpandedCell, gridCenterX, gridCenterY, gridResolutionXY, centerIndex);
         }

         recursivelyAddNeighbors(unexpandedCell, environmentModel, maxRecursionDepth, 0, gridCenterX, gridCenterY, gridResolutionXY, centerIndex);
      }

      environmentModel.getRegions().removeIf(region -> region.getCells().size() < parameters.getMinCellsInARegion());

      return environmentModel;
   }

   public static SteppableRegionsList createSteppableRegions(ConcaveHullFactoryParameters concaveHullFactoryParameters,
                                                             PolygonizerParameters polygonizerParameters,
                                                             SteppableRegionCalculatorParametersReadOnly steppableRegionCalculatorParameters,
                                                             SteppableRegionsEnvironmentModel environmentModel,
                                                             HeightMapData heightMapData,
                                                             double footYaw)
   {
      return createSteppableRegions(concaveHullFactoryParameters,
                                    polygonizerParameters,
                                    steppableRegionCalculatorParameters,
                                    environmentModel,
                                    heightMapData.getGridCenter().getX(),
                                    heightMapData.getGridCenter().getY(),
                                    heightMapData.getGridSizeXY(),
                                    heightMapData.getGridResolutionXY(),
                                    heightMapData.getCenterIndex(),
                                    footYaw);
   }

   public static SteppableRegionsList createSteppableRegions(ConcaveHullFactoryParameters concaveHullFactoryParameters,
                                                             PolygonizerParameters polygonizerParameters,
                                                             SteppableRegionCalculatorParametersReadOnly steppableRegionCalculatorParameters,
                                                             SteppableRegionsEnvironmentModel environmentModel,
                                                             double gridCenterX,
                                                             double gridCenterY,
                                                             double gridSizeXY,
                                                             double gridResolutionXY,
                                                             int centerIndex,
                                                             double footYaw)
   {
      List<SteppableRegion> listToReturn = new ArrayList<>();
      environmentModel.getRegions()
                      .stream()
                      .map(region -> createSteppableRegions(concaveHullFactoryParameters,
                                                            polygonizerParameters,
                                                            steppableRegionCalculatorParameters,
                                                            region,
                                                            gridCenterX,
                                                            gridCenterY,
                                                            gridSizeXY,
                                                            gridResolutionXY,
                                                            centerIndex,
                                                            footYaw))
                      .forEach(listToReturn::addAll);
      for (int i = 0; i < listToReturn.size(); i++)
         listToReturn.get(i).setRegionId(i);

      return new SteppableRegionsList(footYaw, listToReturn);
   }

   private static List<SteppableRegion> createSteppableRegions(ConcaveHullFactoryParameters concaveHullFactoryParameters,
                                                               PolygonizerParameters polygonizerParameters,
                                                               SteppableRegionCalculatorParametersReadOnly steppableRegionCalculatorParameters,
                                                               SteppableRegionDataHolder regionDataHolder,
                                                               double gridCenterX,
                                                               double gridCenterY,
                                                               double gridSizeXY,
                                                               double gridResolutionXY,
                                                               int centerIndex,
                                                               double footYaw)
   {
      if (regionDataHolder.getMemberCells().size() < steppableRegionCalculatorParameters.getMinCellsInARegion())
         return new ArrayList<>();

      List<Point2D> pointsInWorld = new ArrayList<>();
      List<Point2D> outerRing = getOuterRingPoints(regionDataHolder,
                                                   gridCenterX,
                                                   gridCenterY,
                                                   gridResolutionXY,
                                                   centerIndex,
                                                   steppableRegionCalculatorParameters.getFractionOfCellToExpandSmallRegions());
      if (outerRing != null)
         pointsInWorld.addAll(outerRing);

      List<Point2D> interiorPoints = getInteriorPoints(regionDataHolder, steppableRegionCalculatorParameters.getMaxInteriorPointsToInclude(), new Random());
      if (interiorPoints != null)
         pointsInWorld.addAll(interiorPoints);

      Point2DReadOnly centroid = regionDataHolder.getCentroidInWorld();
      Pose3D pose = new Pose3D(new Point3D(centroid), new AxisAngle());

      List<Point2D> pointCloudInRegion = pointsInWorld.parallelStream().map(point -> toPointInPlane(point, centroid)).toList();

      ConcaveHullCollection concaveHullCollection = SimpleConcaveHullFactory.createConcaveHullCollection(pointCloudInRegion, concaveHullFactoryParameters);

      // Apply some simple filtering to reduce the number of vertices and hopefully the number of convex polygons.
      double shallowAngleThreshold = polygonizerParameters.getShallowAngleThreshold();
      double peakAngleThreshold = polygonizerParameters.getPeakAngleThreshold();
      double lengthThreshold = polygonizerParameters.getLengthThreshold();

      ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullCollection);
      ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHullCollection);
      //      if (polygonizerParameters.getCutNarrowPassage())
      //         concaveHullCollection = ConcaveHullPruningFilteringTools.concaveHullNarrowPassageCutter(lengthThreshold, concaveHullCollection);

      return createSteppableRegions(pose,
                                    concaveHullCollection,
                                    gridCenterX,
                                    gridCenterY,
                                    gridSizeXY,
                                    gridResolutionXY,
                                    centerIndex,
                                    regionDataHolder,
                                    footYaw);
   }

   private static Point2D toPointInPlane(Point2DReadOnly pointInWorld, Point2DReadOnly centroid)
   {
      Point2D point = new Point2D(pointInWorld);
      point.sub(centroid);

      return point;
   }

   private static List<Point2D> getOuterRingPoints(SteppableRegionDataHolder regionDataHolder,
                                                   double gridCenterX,
                                                   double gridCenterY,
                                                   double gridResolutionXY,
                                                   int centerIndex,
                                                   double inflationFraction)
   {
      if (regionDataHolder.getBorderRings().isEmpty())
         return null;

      List<SteppableBorderRing> ringList = new ArrayList<>(regionDataHolder.getBorderRings());
      ringList.sort(Comparator.comparingInt(SteppableBorderRing::size));

      double inflationSize = inflationFraction * gridResolutionXY / 2.0;

      SteppableBorderRing longestRing = ringList.get(ringList.size() - 1);
      List<Point2D> points = new ArrayList<>();
      for (SteppableCell cell : longestRing)
      {
         Point2D point = convertCellToPoint(cell, gridCenterX, gridCenterY, gridResolutionXY, centerIndex);

         if (longestRing.size() > 100)
         {
            points.add(point);
         }
         else
         {
            points.add(new Point2D(point.getX() + inflationSize, point.getY() + inflationSize));
            points.add(new Point2D(point.getX() + inflationSize, point.getY() - inflationSize));
            points.add(new Point2D(point.getX() - inflationSize, point.getY() - inflationSize));
            points.add(new Point2D(point.getX() - inflationSize, point.getY() + inflationSize));
         }
      }
      return points;
   }

   private static List<Point2D> getInteriorPoints(SteppableRegionDataHolder regionDataHolder, int cellsToSample, Random random)
   {
      if (regionDataHolder.getMemberCells().isEmpty())
         return null;

      List<Point2DReadOnly> memberPoints = new ArrayList<>(regionDataHolder.getMemberPoints());
      List<Point2D> points = new ArrayList<>();

      for (int i = 0; i < Math.min(memberPoints.size(), cellsToSample); i++)
      {
         Point2DReadOnly cell = memberPoints.remove(RandomNumbers.nextInt(random, 0, memberPoints.size() - 1));
         points.add(new Point2D(cell));
      }
      return points;
   }

   public static List<SteppableRegion> createSteppableRegions(RigidBodyTransformReadOnly transformToWorld,
                                                              ConcaveHullCollection concaveHullCollection,
                                                              double gridCenterX,
                                                              double gridCenterY,
                                                              double gridSizeXY,
                                                              double gridResolutionXY,
                                                              int centerIndex,
                                                              SteppableRegionDataHolder regionDataHolder,
                                                              double footYaw)
   {
      List<SteppableRegion> regions = concaveHullCollection.getConcaveHulls()
                                                           .parallelStream()
                                                           .map(hull -> createSteppableRegion(transformToWorld, hull, footYaw))
                                                           .toList();
      HeightMapData regionHeightMap = createHeightMapForRegion(regionDataHolder, gridCenterX, gridCenterY, gridSizeXY, gridResolutionXY, centerIndex);
      regions.forEach(region -> region.setLocalHeightMap(regionHeightMap));
      return regions;
   }

   public static SteppableRegion createSteppableRegion(RigidBodyTransformReadOnly transformToWorld, ConcaveHull concaveHull, double footYaw)
   {
      return new SteppableRegion(transformToWorld, concaveHull.getConcaveHullVertices(), footYaw);
   }

   public static HeightMapData createHeightMapForRegion(SteppableRegionDataHolder regionDataHolder,
                                                        double gridCenterX,
                                                        double gridCenterY,
                                                        double gridSizeXY,
                                                        double gridResolutionXY,
                                                        int centerIndex)
   {
      HeightMapData regionHeightMap = new HeightMapData(gridResolutionXY, gridSizeXY, gridCenterX, gridCenterY);

      for (SteppableCell cell : regionDataHolder.getCells())
      {
         int key = HeightMapTools.indicesToKey(cell.getXIndex(), cell.getYIndex(), centerIndex);
         regionHeightMap.setHeightAt(key, cell.getZ(), cell.getNormal());
      }

      return regionHeightMap;
   }

   private static SteppableRegionsEnvironmentModel createUnsortedSteppableRegionEnvironment(BytedecoImage steppability,
                                                                                            BytedecoImage snappedHeight,
                                                                                            BytedecoImage snappedNormalX,
                                                                                            BytedecoImage snappedNormalY,
                                                                                            BytedecoImage snappedNormalZ,
                                                                                            BytedecoImage connections)
   {
      int cellsPerSide = steppability.getImageHeight();
      SteppableRegionsEnvironmentModel steppableRegionsToConvert = new SteppableRegionsEnvironmentModel(cellsPerSide);

      for (int x = 0; x < cellsPerSide; x++)
      {
         for (int y = 0; y < cellsPerSide; y++)
         {
            // this cell is steppable. Also remember the image x-y is switched
            if (steppability.getByteAsInteger(x, y) == SnapResult.VALID.ordinal())
            {
               boolean isBorderCell;
               int connection = connections.getByteAsInteger(x, y);

               if (Integer.bitCount(connection) != 8) // 8 bits is fully connected
                  isBorderCell = true;
               else
                  isBorderCell = false;

               double z = snappedHeight.getFloat(x, y);
               Vector3D normal = new Vector3D(snappedNormalX.getFloat(x, y), snappedNormalY.getFloat(x, y), snappedNormalZ.getFloat(x, y));
               SteppableCell cell = new SteppableCell(x, y, z, normal, cellsPerSide, isBorderCell);
               steppableRegionsToConvert.addUnexpandedSteppableCell(cell);
            }
         }
      }

      for (int x = 0; x < cellsPerSide; x++)
      {
         for (int y = 0; y < cellsPerSide; y++)
         {
            SteppableCell cell = steppableRegionsToConvert.getCellAt(x, y);
            if (cell != null)
               collectConnectedCellNeighbors(cell, steppableRegionsToConvert, connections);
         }
      }

      return steppableRegionsToConvert;
   }

   private static void recursivelyAddNeighbors(SteppableCell cellToExpand,
                                               SteppableRegionsEnvironmentModel environmentModel,
                                               int maxDepth,
                                               int currentDepth,
                                               double gridCenterX,
                                               double gridCenterY,
                                               double gridResolutionXY,
                                               int centerIndex)
   {
      if (!cellToExpand.cellHasBeenAssigned())
         throw new RuntimeException("Should only be expanding assigned cells.");

      environmentModel.markCellAsExpanded(cellToExpand);

      for (SteppableCell neighbor : cellToExpand.getValidNeighbors())
      {
         if (neighbor.cellHasBeenAssigned())
         {
            // The neighboring cell that we're connected to has already been assigned to another region. Since these regions are connected, we should merge them.
            SteppableRegionDataHolder neighborRegion = neighbor.getRegion();
            if (cellToExpand.getRegion().mergeRegion(neighborRegion))
            {
               // If the regions we're successfully merged, we need to remove the other region from the environment.
               // The only reason it wouldn't be successful is if they're the same region.
               environmentModel.getRegions().remove(neighborRegion);
            }
         }
         else
         {
            // the cell has not been assigned already, so we can directly add it to the region contained by the parent cell.
            cellToExpand.getRegion().addCell(neighbor, gridCenterX, gridCenterY, gridResolutionXY, centerIndex);
         }

         if (!neighbor.cellHasBeenExpanded() && currentDepth < maxDepth)
         {
            if (!cellToExpand.cellHasBeenAssigned())
               throw new RuntimeException("Somehow the assignment operation failed.");

            recursivelyAddNeighbors(neighbor,
                                    environmentModel,
                                    maxDepth,
                                    currentDepth + 1,
                                    gridCenterX,
                                    gridCenterY,
                                    gridResolutionXY,
                                    centerIndex);
         }
      }
   }

   private static void recursivelyAddBorderNeighbors(SteppableCell cellToExpand,
                                                     SteppableRegionsEnvironmentModel environmentModel,
                                                     int maxDepth,
                                                     int currentDepth,
                                                     double gridCenterX,
                                                     double gridCenterY,
                                                     double gridResolutionXY,
                                                     int centerIndex)
   {
      if (!cellToExpand.cellHasBeenAssigned())
         throw new RuntimeException("Should only be expanding assigned cells.");

      environmentModel.markCellAsExpanded(cellToExpand);

      for (SteppableCell neighbor : cellToExpand.getValidNeighbors())
      {
         if (neighbor.cellHasBeenAssigned())
         {
            SteppableRegionDataHolder neighborRegion = neighbor.getRegion();
            if (neighbor.isBorderCell())
            {
               // If the neighbor has already been assigned, and is contained as part of a border ring, those rings should be merged together.
               SteppableBorderRing neighborRing = neighbor.getBorderRing();
               if (cellToExpand.getBorderRing().mergeRing(neighborRing))
               {
                  // the only reason that the merge ring operation should fail is if the rings were already the same, so we wouldn't want it to remove.
                  neighborRegion.removeBorderRing(neighborRing);
               }
            }

            // The neighbor is connected, so the regions are connected, and should be merged together.
            if (cellToExpand.getRegion().mergeRegion(neighborRegion))
               environmentModel.removeRegion(neighborRegion);
         }
         else
         {
            // the neighboring cell has not been assigned already. This means we should add it to the current region.
            cellToExpand.getRegion().addCell(neighbor, gridCenterX, gridCenterY, gridResolutionXY, centerIndex);
            if (neighbor.isBorderCell())
            {
               // If the neighboring cell is a border cell, it is part of the border ring.
               cellToExpand.getBorderRing().addCell(neighbor);
               neighbor.setBorderRing(cellToExpand.getBorderRing());
            }
         }

         if (neighbor.isBorderCell() && !neighbor.cellHasBeenExpanded() && currentDepth < maxDepth)
         {
            recursivelyAddBorderNeighbors(neighbor,
                                          environmentModel,
                                          maxDepth,
                                          currentDepth + 1,
                                          gridCenterX,
                                          gridCenterY,
                                          gridResolutionXY,
                                          centerIndex);
         }
      }
   }

   /**
    * This collects a list of all the cells that are in a circle around {@param cell}, as long as they are contained within the "environment". It does not
    * check for connection.
    */
   private static void collectConnectedCellNeighbors(SteppableCell cell, SteppableRegionsEnvironmentModel environmentModel, BytedecoImage connections)
   {
      List<SteppableCell> cellNeighborsToPack = cell.getValidNeighbors();
      int cellsPerSide = environmentModel.getCellsPerSide();

      int boundaryConnectionsEncodedAsOnes = connections.getByteAsInteger(cell.getXIndex(), cell.getYIndex());

      int neighborId = 0;
      for (int x_offset = -1; x_offset <= 1; x_offset++)
      {
         for (int y_offset = -1; y_offset <= 1; y_offset++)
         {
            // we don't want to increment here, because this doesn't count, since it is the current row
            if (x_offset == 0 && y_offset == 0)
               continue;

            int neighborX = cell.getXIndex() + x_offset;
            int neighborY = cell.getYIndex() + y_offset;

            if (neighborX < 0 || neighborY < 0 || neighborX >= cellsPerSide || neighborY >= cellsPerSide)
            {
               // increment the neighbor id for the next time through.
               neighborId++;
               continue;
            }

            SteppableCell neighbor = environmentModel.getCellAt(neighborX, neighborY);
            if (neighbor != null && isConnected(neighborId, boundaryConnectionsEncodedAsOnes))
            {
               cellNeighborsToPack.add(neighbor);
            }

            neighborId++;
         }
      }
   }

   public static Point2D convertCellToPoint(SteppableCell cell, double gridCenterX, double gridCenterY, double gridResolutionXY, int centerIndex)
   {
      double x = HeightMapTools.indexToCoordinate(cell.getXIndex(), gridCenterX, gridResolutionXY, centerIndex);
      double y = HeightMapTools.indexToCoordinate(cell.getYIndex(), gridCenterY, gridResolutionXY, centerIndex);

      return new Point2D(x, y);
   }

   private static boolean isConnected(int counter, int connectionValue)
   {
      int mask = (1 << counter);
      int maskedValue = mask & connectionValue;
      return maskedValue > 0;
   }
}
