package us.ihmc.perception.steppableRegions;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.perception.steppableRegions.data.SteppableBorderRing;
import us.ihmc.perception.steppableRegions.data.SteppableCell;
import us.ihmc.perception.steppableRegions.data.SteppableRegionDataHolder;
import us.ihmc.perception.steppableRegions.data.SteppableRegionsEnvironmentModel;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.robotEnvironmentAwareness.geometry.*;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
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
            region.addCell(unexpandedCell, heightMapData);

            SteppableBorderRing borderRing = region.createNewBorderRing();
            borderRing.addCell(unexpandedCell);
         }
         else
         {
            throw new RuntimeException("Should never reach this place");
         }

         recursivelyAddBorderNeighbors(unexpandedCell, connections, environmentModel, maxDepth, 0, heightMapData);
      }

      while (environmentModel.hasUnexpandedInteriorCells())
      {
         // Start assuming we're expanding in a new region
         SteppableCell unexpandedCell = environmentModel.getNextUnexpandedInteriorCell();
         if (!unexpandedCell.cellHasBeenAssigned())
         {
            SteppableRegionDataHolder region = environmentModel.createNewSteppableRegion();
            region.addCell(unexpandedCell, heightMapData);
         }

         recursivelyAddNeighbors(unexpandedCell, connections, environmentModel, maxDepth, 0, heightMapData);
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
                                                            heightMapData,
                                                            footYaw))
                      .forEach(listToReturn::addAll);
      for (int i = 0; i < listToReturn.size(); i++)
         listToReturn.get(i).setRegionId(i);

      return new SteppableRegionsList(footYaw, listToReturn);
   }

   public static List<SteppableRegion> createSteppableRegions(ConcaveHullFactoryParameters concaveHullFactoryParameters,
                                                              PolygonizerParameters polygonizerParameters,
                                                              SteppableRegionCalculatorParametersReadOnly steppableRegionCalculatorParameters,
                                                              SteppableRegionDataHolder regionDataHolder,
                                                              HeightMapData heightMapData,
                                                              double footYaw)
   {
      if (regionDataHolder.getMemberCells().size() < steppableRegionCalculatorParameters.getMinCellsInARegion())
         return new ArrayList<>();

      List<Point3D> pointsInWorld = new ArrayList<>();
      List<Point3D> outerRing = getOuterRingPoints(regionDataHolder,
                                                   heightMapData,
                                                   steppableRegionCalculatorParameters.getFractionOfCellToExpandSmallRegions());
      if (outerRing != null)
         pointsInWorld.addAll(outerRing);

      List<Point3D> interiorPoints = getInteriorPoints(regionDataHolder,
                                                       steppableRegionCalculatorParameters.getMaxInteriorPointsToInclude(),
                                                       new Random());
      if (interiorPoints != null)
         pointsInWorld.addAll(interiorPoints);

      Point3DReadOnly centroid = regionDataHolder.getCentroidInWorld();
      Vector3DReadOnly normal = regionDataHolder.getNormalInWorld();
      AxisAngle orientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(normal);
      orientation.invert();
      Pose3D pose = new Pose3D(centroid, orientation);

      List<Point2D> pointCloudInRegion = pointsInWorld.parallelStream().map(point -> PolygonizerTools.toPointInPlane(point, centroid, orientation)).toList();

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
                                    heightMapData,
                                    regionDataHolder,
                                    footYaw);
   }

   private static List<Point3D> getOuterRingPoints(SteppableRegionDataHolder regionDataHolder, HeightMapData heightMapData, double inflationFraction)
   {
      if (regionDataHolder.getBorderRings().size() == 0)
         return null;

      List<SteppableBorderRing> ringList = new ArrayList<>(regionDataHolder.getBorderRings());
      ringList.sort(Comparator.comparingInt(SteppableBorderRing::size));

      double inflationSize = inflationFraction * heightMapData.getGridResolutionXY() / 2.0;

      SteppableBorderRing longestRing = ringList.get(ringList.size() - 1);
      List<Point3D> points = new ArrayList<>();
      for (SteppableCell cell : longestRing)
      {
         Point3D point = convertCellToPoint(cell, heightMapData);

         if (longestRing.size() > 100)
         {
            points.add(point);
         }
         else
         {
            points.add(new Point3D(point.getX() + inflationSize, point.getY() + inflationSize, point.getZ()));
            points.add(new Point3D(point.getX() + inflationSize, point.getY() - inflationSize, point.getZ()));
            points.add(new Point3D(point.getX() - inflationSize, point.getY() - inflationSize, point.getZ()));
            points.add(new Point3D(point.getX() - inflationSize, point.getY() + inflationSize, point.getZ()));
         }
      }
      return points;
   }

   private static List<Point3D> getInteriorPoints(SteppableRegionDataHolder regionDataHolder, int cellsToSample, Random random)
   {
      if (regionDataHolder.getMemberCells().size() == 0)
         return null;

      List<Point3DReadOnly> memberPoints = new ArrayList<>(regionDataHolder.getMemberPoints());
      List<Point3D> points = new ArrayList<>();

      for (int i = 0; i < Math.min(memberPoints.size(), cellsToSample); i++)
      {
         Point3DReadOnly cell = memberPoints.remove(RandomNumbers.nextInt(random, 0, memberPoints.size() - 1));
         points.add(new Point3D(cell));
      }
      return points;
   }

   public static List<SteppableRegion> createSteppableRegions(RigidBodyTransformReadOnly transformToWorld,
                                                              ConcaveHullCollection concaveHullCollection,
                                                              HeightMapData heightMapData,
                                                              SteppableRegionDataHolder regionDataHolder,
                                                              double footYaw)
   {
      List<SteppableRegion> regions = concaveHullCollection.getConcaveHulls()
                                                           .parallelStream()
                                                           .map(hull -> createSteppableRegion(transformToWorld, hull, footYaw))
                                                           .toList();
      HeightMapData regionHeightMap = createHeightMapForRegion(regionDataHolder, heightMapData);
      regions.forEach(region -> region.setLocalHeightMap(regionHeightMap));
      return regions;
   }

   public static SteppableRegion createSteppableRegion(RigidBodyTransformReadOnly transformToWorld,
                                                       ConcaveHull concaveHull,
                                                       double footYaw)
   {
      return new SteppableRegion(transformToWorld, concaveHull.getConcaveHullVertices(), footYaw);
   }

   public static HeightMapData createHeightMapForRegion(SteppableRegionDataHolder regionDataHolder, HeightMapData heightMapData)
   {
      double resolution = heightMapData.getGridResolutionXY();

      HeightMapData regionHeightMap = new HeightMapData(resolution,
                                                        heightMapData.getGridSizeXY(),
                                                        heightMapData.getGridCenter().getX(),
                                                        heightMapData.getGridCenter().getY());

      for (SteppableCell cell : regionDataHolder.getCells())
      {
         int key = HeightMapTools.indicesToKey(cell.getX(), cell.getY(), heightMapData.getCenterIndex());
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
                                               HeightMapData heightMapData)
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
            cellToExpand.getRegion().addCell(neighbor, heightMapData);
         }

         if (!neighbor.cellHasBeenExpanded() && currentDepth < maxDepth && cellToExpand.cellHasBeenAssigned())
            recursivelyAddNeighbors(neighbor, connections, environmentModel, maxDepth, currentDepth + 1, heightMapData);
      }
   }

   private static void recursivelyAddBorderNeighbors(SteppableCell cellToExpand,
                                                     BytedecoImage connections,
                                                     SteppableRegionsEnvironmentModel environmentModel,
                                                     int maxDepth,
                                                     int currentDepth,
                                                     HeightMapData heightMapData)
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
            cellToExpand.getRegion().addCell(neighbor, heightMapData);
            if (neighbor.isBorderCell())
            {
               cellToExpand.getBorderRing().addCell(neighbor);
               neighbor.setBorderRing(cellToExpand.getBorderRing());
            }
         }

         if (neighbor.isBorderCell() && !neighbor.cellHasBeenExpanded() && currentDepth < maxDepth && cellToExpand.cellHasBeenAssigned())
            recursivelyAddBorderNeighbors(neighbor, connections, environmentModel, maxDepth, currentDepth + 1, heightMapData);
      }
   }

   private static List<SteppableCell> collectConnectedCellNeighbors(SteppableCell cell,
                                                                    SteppableRegionsEnvironmentModel environmentModel,
                                                                    BytedecoImage connections)
   {
      int cellsPerSide = environmentModel.getCellsPerSide();
      int row = cellsPerSide - cell.getX() - 1;
      int col = cellsPerSide - cell.getY() - 1;

      int boundaryConnectionsEncodedAsOnes = connections.getInt(row, col);
      List<NeighborCell> neighbors = collectCellNeighbors(cell, environmentModel);

      return neighbors.stream()
                      .filter(neighbor -> isConnected(neighbor.getNeighborIndex(), boundaryConnectionsEncodedAsOnes))
                      .map(NeighborCell::getCell)
                      .toList();
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
            int neighborX = cell.getX() - y_offset;
            int neighborY = cell.getY() - x_offset;

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

   public static Point3D convertCellToPoint(SteppableCell cell, HeightMapData heightMapData)
   {
      double x = HeightMapTools.indexToCoordinate(cell.getX(),
                                                  heightMapData.getGridCenter().getX(),
                                                  heightMapData.getGridResolutionXY(),
                                                  heightMapData.getCenterIndex());
      double y = HeightMapTools.indexToCoordinate(cell.getY(),
                                                  heightMapData.getGridCenter().getY(),
                                                  heightMapData.getGridResolutionXY(),
                                                  heightMapData.getCenterIndex());
      double height = heightMapData.getHeightAt(cell.getX(), cell.getY());

      return new Point3D(x, y, height);
   }

   private static boolean isConnected(int counter, int connectionValue)
   {
      int mask = (1 << counter);
      int maskedValue = mask & connectionValue;
      return maskedValue > 0;
   }

   private static class NeighborCell
   {
      private final int neighborIndex;
      private final SteppableCell cell;

      public NeighborCell(int neighborIndex, SteppableCell cell)
      {
         this.neighborIndex = neighborIndex;
         this.cell = cell;
      }

      public int getNeighborIndex()
      {
         return neighborIndex;
      }

      public SteppableCell getCell()
      {
         return cell;
      }
   }
}
