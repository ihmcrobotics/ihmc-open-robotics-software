package us.ihmc.ihmcPerception.steppableRegions;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.robotEnvironmentAwareness.geometry.*;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.*;
import java.util.stream.Collectors;

public class SteppableRegionsCalculator
{
   public static SteppableRegionsEnvironmentModel mergeCellsIntoSteppableRegionEnvironment(BytedecoImage steppability, BytedecoImage connections)
   {
      SteppableRegionsEnvironmentModel environmentModel = createUnsortedSteppableRegionEnvironment(steppability);

      if (steppability.getImageHeight() != steppability.getImageWidth())
         throw new RuntimeException("Should be square");

      while (environmentModel.hasUnassignedCells())
         recursivelyAddNeighbors(environmentModel.getNextUnassignedCell(), steppability, connections, environmentModel);

      return environmentModel;
   }

   public static List<SteppableRegion> createSteppableRegions(SteppableRegionsEnvironmentModel environmentModel, HeightMapData heightMapData)
   {
      List<SteppableRegion> listToReturn = new ArrayList<>();
      environmentModel.steppableRegions.parallelStream().map(region -> createSteppableRegions(region, heightMapData)).forEach(listToReturn::addAll);

      return listToReturn;
   }

   public static List<SteppableRegion> createSteppableRegions(SteppableRegionDataHolder regionDataHolder, HeightMapData heightMapData)
   {
      Collection<Point3D> pointCloudInWorld = regionDataHolder.getCellPointCloud(heightMapData);
      Point3DReadOnly centroid = regionDataHolder.getCentroidInWorld();
      Vector3DReadOnly normal = regionDataHolder.getNormalInWorld();
      AxisAngle orientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(normal);

      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      List<Point2D> pointCloudInRegion = pointCloudInWorld.parallelStream().map(pointInWorld ->
                                                                                {
                                                                                   return PolygonizerTools.toPointInPlane(pointInWorld, centroid, orientation);
                                                                                }).toList();

      ConcaveHullCollection concaveHullCollection = SimpleConcaveHullFactory.createConcaveHullCollection(pointCloudInRegion, parameters);

      // Apply some simple filtering to reduce the number of vertices and hopefully the number of convex polygons.
      double shallowAngleThreshold = polygonizerParameters.getShallowAngleThreshold();
      double peakAngleThreshold = polygonizerParameters.getPeakAngleThreshold();
      double lengthThreshold = polygonizerParameters.getLengthThreshold();

      ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullCollection);
      ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHullCollection);
      if (polygonizerParameters.getCutNarrowPassage())
         concaveHullCollection = ConcaveHullPruningFilteringTools.concaveHullNarrowPassageCutter(lengthThreshold, concaveHullCollection);

      return createSteppableRegions(centroid, orientation, concaveHullCollection);
   }

   public static List<SteppableRegion> createSteppableRegions(Point3DReadOnly origin,
                                                              Orientation3DReadOnly orientation,
                                                              ConcaveHullCollection concaveHullCollection)
   {
      return concaveHullCollection.getConcaveHulls().parallelStream().map(hull -> createSteppableRegion(origin, orientation, hull)).toList();
   }

   public static SteppableRegion createSteppableRegion(Point3DReadOnly origin,
                                                       Orientation3DReadOnly orientation,
                                                       ConcaveHull concaveHull)
   {
      return new SteppableRegion(origin, orientation, concaveHull.getConcaveHullVertices());
   }

   private static SteppableRegionsEnvironmentModel createUnsortedSteppableRegionEnvironment(BytedecoImage steppability)
   {
      int cellsPerSide = steppability.getImageHeight();
      SteppableRegionsEnvironmentModel steppableRegionsToConvert = new SteppableRegionsEnvironmentModel(cellsPerSide);

      for (int x = 0; x < cellsPerSide; x++)
      {
         for (int y = 0; y < cellsPerSide; y++)
         {
            // this cell is steppable. Also remember the image x-y is switched
            if (steppability.getInt(y, x) == 1)
            {
               SteppableCell cell = new SteppableCell(x, y, cellsPerSide);
               steppableRegionsToConvert.addUnassignedSteppableCell(cell);
            }
         }
      }

      return steppableRegionsToConvert;
   }

   private static void recursivelyAddNeighbors(SteppableCell cellToExpand,
                                               BytedecoImage steppability,
                                               BytedecoImage connections,
                                               SteppableRegionsEnvironmentModel environmentModel)
   {
      // If we don't already have a region, create a region and add this cell to it
      if (!cellToExpand.hasRegion())
      {
         SteppableRegionDataHolder region = environmentModel.createNewSteppableRegion();
         region.addCell(cellToExpand);
      }
      environmentModel.markCellAsAssigned(cellToExpand);

      int boundaryConnectionsEncodedAsOnes = connections.getInt(cellToExpand.x, cellToExpand.y);

      int cellsPerSide = steppability.getImageHeight();
      for (int i = 0; i < 4; i++)
      {
         int neighborX = cellToExpand.x;
         int neighborY = cellToExpand.y;

         // These are switched from the open cl code, since the coordinates are different
         if (i == 0)
            neighborY++;
         else if (i == 1)
            neighborX++;
         else if (i == 2)
            neighborY--;
         else
            neighborX--;

         if (neighborX < 0 || neighborY < 0 || neighborX >= cellsPerSide || neighborY >= cellsPerSide)
            continue;

         if (isConnected(i, boundaryConnectionsEncodedAsOnes))
         {
            SteppableCell neighbor = environmentModel.getCellAt(neighborX, neighborY);
            if (neighbor.cellHasBeenExpand())
            {
               cellToExpand.getRegion().mergeRegion(neighbor.getRegion());
            }
            else
            {
               // the cell has not been expanded
               cellToExpand.getRegion().addCell(neighbor);
               recursivelyAddNeighbors(neighbor, steppability, connections, environmentModel);
            }
         }
      }
   }

   private static boolean isConnected(int edgeNumber, int connectionValue)
   {
      int mask = (1 << edgeNumber);
      int maskedValue = mask & connectionValue;
      return maskedValue > 0;
   }

   private static Point3D getAsPoint(HeightMapData heightMapData, SteppableCell steppableCell)
   {
      double x = HeightMapTools.indexToCoordinate(steppableCell.x,
                                                  heightMapData.getGridCenter().getX(),
                                                  heightMapData.getGridResolutionXY(),
                                                  heightMapData.getCenterIndex());
      double y = HeightMapTools.indexToCoordinate(steppableCell.y,
                                                  heightMapData.getGridCenter().getY(),
                                                  heightMapData.getGridResolutionXY(),
                                                  heightMapData.getCenterIndex());
      return new Point3D(x, y, heightMapData.getHeightAt(steppableCell.x, steppableCell.y));
   }

   public static class SteppableRegionsEnvironmentModel
   {
      private final Stack<SteppableCell> unassignedCellsInTheEnvironment = new Stack<>();
      private final SteppableCell[][] steppableCellsGrid;

      private final List<SteppableRegionDataHolder> steppableRegions = new ArrayList<>();

      public SteppableRegionsEnvironmentModel(int cellsPerSide)
      {
         steppableCellsGrid = new SteppableCell[cellsPerSide][];
         for (int i = 0; i < cellsPerSide; i++)
            steppableCellsGrid[i] = new SteppableCell[cellsPerSide];
      }

      public SteppableCell[][] getSteppableCellsGrid()
      {
         return steppableCellsGrid;
      }

      public SteppableCell getCellAt(int x, int y)
      {
         return steppableCellsGrid[x][y];
      }

      public boolean hasUnassignedCells()
      {
         return !unassignedCellsInTheEnvironment.isEmpty();
      }

      public SteppableCell getNextUnassignedCell()
      {
         return unassignedCellsInTheEnvironment.remove(0);
      }

      public void markCellAsAssigned(SteppableCell cell)
      {
         unassignedCellsInTheEnvironment.remove(cell);
      }

      public void addUnassignedSteppableCell(SteppableCell cell)
      {
         unassignedCellsInTheEnvironment.add(cell);
         steppableCellsGrid[cell.x][cell.y] = cell;
      }

      public SteppableRegionDataHolder createNewSteppableRegion()
      {
         SteppableRegionDataHolder steppableRegion = new SteppableRegionDataHolder();
         steppableRegions.add(steppableRegion);

         return steppableRegion;
      }
   }

   private static class SteppableRegionDataHolder
   {
      private boolean isCellPointCloudUpToDate = false;
      private boolean isCentroidUpToDate = false;
      private boolean isNormalUpToDate = false;

      private final HashSet<SteppableCell> memberCells = new HashSet<>();

      private List<Point3D> pointCloud ;
      private final Point3D regionCentroidTotal = new Point3D();
      private final Point3D regionCentroid = new Point3D();
      private final Vector3D regionNormal = new Vector3D();

      public void addCell(SteppableCell cell)
      {
         markChanged();
         memberCells.add(cell);
         cell.setRegion(this);
      }

      public void mergeRegion(SteppableRegionDataHolder other)
      {
         other.memberCells.forEach(this::addCell);
      }

      public Collection<Point3D> getCellPointCloud(HeightMapData heightMapData)
      {
         if (!isCellPointCloudUpToDate)
            updatePointCloud(heightMapData);
         return pointCloud;
      }

      public Point3DReadOnly getCentroidInWorld()
      {
         if (!isCentroidUpToDate)
            updateCentroid();
         return regionCentroid;
      }

      public Vector3DReadOnly getNormalInWorld()
      {
         if (!isNormalUpToDate)
            updateNormal();
         return regionNormal;
      }

      public Collection<SteppableCell> getCells()
      {
         return memberCells;
      }

      private void markChanged()
      {
         isCellPointCloudUpToDate = false;
         isCentroidUpToDate = false;
         isNormalUpToDate = false;
      }

      private void updatePointCloud(HeightMapData heightMapData)
      {
         pointCloud = memberCells.parallelStream().map(cell -> getAsPoint(heightMapData, cell)).collect(Collectors.toList());
         isCellPointCloudUpToDate = true;
      }

      private void updateCentroid()
      {
         regionCentroid.setAndScale(1.0 / memberCells.size(), regionCentroidTotal);
         isCentroidUpToDate = true;
      }

      private void updateNormal()
      {
         // TODO run a least squares fit here
         regionNormal.set(0.0, 0.0, 1.0);
         isNormalUpToDate = false;
      }
   }

   private static class SteppableCell
   {
      private final int x;
      private final int y;

      private final int hashCode;

      private SteppableRegionDataHolder region;

      public SteppableCell(int x, int y, int cellsPerSide)
      {
         this.x = x;
         this.y = y;

         hashCode = x * cellsPerSide + y;
      }

      public boolean hasRegion()
      {
         return getRegion() == null;
      }

      public SteppableRegionDataHolder getRegion()
      {
         return region;
      }

      public void setRegion(SteppableRegionDataHolder region)
      {
         this.region = region;
      }

      public boolean cellHasBeenExpand()
      {
         return region != null;
      }

      @Override
      public int hashCode()
      {
         return hashCode;
      }

      @Override
      public boolean equals(Object obj)
      {
         if (obj instanceof SteppableCell other)
         {
            return hashCode == other.hashCode();
         }

         return false;
      }
   }
}
