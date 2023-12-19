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
         throw new RuntimeException("Should be square");

      int maxDepth = 500;
      while (environmentModel.hasUnexpandedBorderCells())
      {
         // Start assuming we're expanding in a new region
         SteppableCell unexpandedCell = environmentModel.getNextUnexpandedBorderCell();
         if (!unexpandedCell.cellHasBeenAssigned())
         {
            SteppableRegionDataHolder region = environmentModel.createNewSteppableRegion();
            region.addCell(unexpandedCell, gridCenterX, gridCenterY, gridResolutionXY, centerIndex);

            SteppableBorderRing borderRing = region.createNewBorderRing();
            borderRing.addCell(unexpandedCell);
         }
         else
         {
            throw new RuntimeException("Should never reach this place");
         }

         recursivelyAddBorderNeighbors(unexpandedCell, connections, environmentModel, maxDepth, 0, gridCenterX, gridCenterY, gridResolutionXY, centerIndex);
      }

      while (environmentModel.hasUnexpandedInteriorCells())
      {
         // Start assuming we're expanding in a new region
         SteppableCell unexpandedCell = environmentModel.getNextUnexpandedInteriorCell();
         if (!unexpandedCell.cellHasBeenAssigned())
         {
            SteppableRegionDataHolder region = environmentModel.createNewSteppableRegion();
            region.addCell(unexpandedCell, gridCenterX, gridCenterY, gridResolutionXY, centerIndex);
         }

         recursivelyAddNeighbors(unexpandedCell, connections, environmentModel, maxDepth, 0, gridCenterX, gridCenterY, gridResolutionXY, centerIndex);
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
      List<SteppableRegion> listToReturn = new ArrayList<>();
      environmentModel.getRegions()
                      .stream()
                      .map(region -> createSteppableRegions(concaveHullFactoryParameters,
                                                            polygonizerParameters,
                                                            steppableRegionCalculatorParameters,
                                                            region,
                                                            heightMapData.getGridCenter().getX(),
                                                            heightMapData.getGridCenter().getY(),
                                                            heightMapData.getGridSizeXY(),
                                                            heightMapData.getGridResolutionXY(),
                                                            heightMapData.getCenterIndex(),
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
                                                   gridSizeXY,
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
            if (x == 0 && y == 0)
               continue;

            int column = cellsPerSide - x - 1;
            int row = cellsPerSide - y - 1;
            // this cell is steppable. Also remember the image x-y is switched
            if (steppability.getInt(column, row) == 0)
            {
               boolean isBorderCell = connections.getInt(column, row) != 255;

               double z = snappedHeight.getFloat(column, row);
               Vector3D normal = new Vector3D(snappedNormalX.getFloat(column, row), snappedNormalY.getFloat(column, row), snappedNormalZ.getFloat(column, row));
               SteppableCell cell = new SteppableCell(x, y, z, normal, cellsPerSide, isBorderCell);
               steppableRegionsToConvert.addUnexpandedSteppableCell(cell);
            }
         }
      }

      return steppableRegionsToConvert;
   }

   private static void recursivelyAddNeighbors(SteppableCell cellToExpand,
                                               BytedecoImage connections,
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

      for (SteppableCell neighbor : collectConnectedCellNeighbors(cellToExpand, environmentModel, connections))
      {

         if (neighbor.cellHasBeenAssigned())
         {
            SteppableRegionDataHolder neighborRegion = neighbor.getRegion();
            if (cellToExpand.getRegion().mergeRegion(neighborRegion))
               environmentModel.getRegions().remove(neighborRegion);
         }
         else
         {
            // the cell has not been assigned
            cellToExpand.getRegion().addCell(neighbor, gridCenterX, gridCenterY, gridResolutionXY, centerIndex);
         }

         if (!neighbor.cellHasBeenExpanded() && currentDepth < maxDepth && cellToExpand.cellHasBeenAssigned())
         {
            recursivelyAddNeighbors(neighbor,
                                    connections,
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
                                                     BytedecoImage connections,
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

      for (SteppableCell neighbor : collectConnectedCellNeighbors(cellToExpand, environmentModel, connections))
      {
         if (neighbor.cellHasBeenAssigned())
         {
            SteppableRegionDataHolder neighborRegion = neighbor.getRegion();
            if (neighbor.isBorderCell())
            {
               SteppableBorderRing neighborRing = neighbor.getBorderRing();
               if (cellToExpand.getBorderRing().mergeRing(neighborRing))
                  neighborRegion.removeBorderRing(neighborRing);
            }

            if (cellToExpand.getRegion().mergeRegion(neighborRegion))
               environmentModel.removeRegion(neighborRegion);
         }
         else
         {
            // the cell has not been assigned
            cellToExpand.getRegion().addCell(neighbor, gridCenterX, gridCenterY, gridResolutionXY, centerIndex);
            if (neighbor.isBorderCell())
            {
               cellToExpand.getBorderRing().addCell(neighbor);
               neighbor.setBorderRing(cellToExpand.getBorderRing());
            }
         }

         if (neighbor.isBorderCell() && !neighbor.cellHasBeenExpanded() && currentDepth < maxDepth && cellToExpand.cellHasBeenAssigned())
            recursivelyAddBorderNeighbors(neighbor,
                                          connections,
                                          environmentModel,
                                          maxDepth,
                                          currentDepth + 1,
                                          gridCenterX,
                                          gridCenterY,
                                          gridResolutionXY,
                                          centerIndex);
      }
   }

   private static List<SteppableCell> collectConnectedCellNeighbors(SteppableCell cell,
                                                                    SteppableRegionsEnvironmentModel environmentModel,
                                                                    BytedecoImage connections)
   {
      int cellsPerSide = environmentModel.getCellsPerSide();
      int row = cellsPerSide - cell.getXIndex() - 1;
      int col = cellsPerSide - cell.getYIndex() - 1;

      int boundaryConnectionsEncodedAsOnes = connections.getInt(row, col);
      List<NeighborCell> neighbors = collectCellNeighbors(cell, environmentModel);

      return neighbors.stream().filter(neighbor -> isConnected(neighbor.neighborIndex(), boundaryConnectionsEncodedAsOnes)).map(NeighborCell::cell).toList();
   }

   private static List<NeighborCell> collectCellNeighbors(SteppableCell cell, SteppableRegionsEnvironmentModel environmentModel)
   {
      List<NeighborCell> cellNeighbors = new ArrayList<>();
      int cellsPerSide = environmentModel.getCellsPerSide();

      int counter = 0;
      for (int x_offset = -1; x_offset <= 1; x_offset++)
      {
         for (int y_offset = -1; y_offset <= 1; y_offset++)
         {
            if (x_offset == 0 && y_offset == 0)
               continue;

            // These are switched because of the difference between coordinates in the height map and image frame
            int neighborX = cell.getXIndex() - y_offset;
            int neighborY = cell.getYIndex() - x_offset;

            if (neighborX < 0 || neighborY < 0 || neighborX >= cellsPerSide || neighborY >= cellsPerSide)
            {
               counter++;
               continue;
            }

            SteppableCell neighbor = environmentModel.getCellAt(neighborX, neighborY);
            if (neighbor == null)
            {
               counter++;
               continue;
            }

            cellNeighbors.add(new NeighborCell(counter, neighbor));

            counter++;
         }
      }

      return cellNeighbors;
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

   private record NeighborCell(int neighborIndex, SteppableCell cell)
   {
   }
}
