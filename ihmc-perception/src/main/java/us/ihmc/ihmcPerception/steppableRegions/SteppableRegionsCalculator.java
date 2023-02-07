package us.ihmc.ihmcPerception.steppableRegions;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.robotEnvironmentAwareness.geometry.*;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ListViewTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.*;
import java.util.stream.Collectors;

public class SteppableRegionsCalculator
{
   public static SteppableRegionsEnvironmentModel mergeCellsIntoSteppableRegionEnvironment(BytedecoImage steppability,
                                                                                           BytedecoImage snappedHeight,
                                                                                           BytedecoImage connections)
   {
      SteppableRegionsEnvironmentModel environmentModel = createUnsortedSteppableRegionEnvironment(steppability, snappedHeight, connections);

      if (steppability.getImageHeight() != steppability.getImageWidth())
         throw new RuntimeException("Should be square");

      int maxDepth = 100;
      while (environmentModel.hasUnexpandedBorderCells())
      {
         // Start assuming we're expanding in a new region
         SteppableCell unexpandedCell = environmentModel.getNextUnexpandedBorderCell();
         if (!unexpandedCell.cellHasBeenAssigned())
         {
            SteppableRegionDataHolder region = environmentModel.createNewSteppableRegion();
            region.addCell(unexpandedCell);
         }
         if (!unexpandedCell.hasBorderRingNumber())
         {
            int ringNumber = unexpandedCell.getRegion().createNewBorderRing();
            unexpandedCell.setBorderRingNumber(ringNumber);
            unexpandedCell.getBorderRing().add(unexpandedCell);
         }

         recursivelyAddBorderNeighbors(unexpandedCell, steppability, connections, environmentModel, maxDepth, 0);
      }

      while (environmentModel.hasUnexpandedInteriorCells())
      {
         // Start assuming we're expanding in a new region
         SteppableCell unexpandedCell = environmentModel.getNextUnexpandedInteriorCell();
         if (!unexpandedCell.cellHasBeenAssigned())
         {
            SteppableRegionDataHolder region = environmentModel.createNewSteppableRegion();
            region.addCell(unexpandedCell);
         }

         recursivelyAddNeighbors(unexpandedCell, steppability, connections, environmentModel, maxDepth, 0);
      }

      return environmentModel;
   }

   public static List<SteppableRegion> createSteppableRegions(ConcaveHullFactoryParameters concaveHullFactoryParameters,
                                                              PolygonizerParameters polygonizerParameters,
                                                              SteppableRegionsEnvironmentModel environmentModel,
                                                              HeightMapData heightMapData)
   {
      List<SteppableRegion> listToReturn = new ArrayList<>();
      environmentModel.getRegions()
                      .stream()
                      .map(region -> createSteppableRegions(concaveHullFactoryParameters, polygonizerParameters, region, heightMapData))
                      .forEach(listToReturn::addAll);
      for (int i = 0; i < listToReturn.size(); i++)
         listToReturn.get(i).setRegionId(i);

      return listToReturn;
   }

   public static List<SteppableRegion> createSteppableRegions(ConcaveHullFactoryParameters concaveHullFactoryParameters,
                                                              PolygonizerParameters polygonizerParameters,
                                                              SteppableRegionDataHolder regionDataHolder,
                                                              HeightMapData heightMapData)
   {
      List<Point3D> outerRing = getOuterRingPoints(regionDataHolder, heightMapData, 0.75);
      if (outerRing == null)
         return new ArrayList<>();
      Point3DReadOnly centroid = regionDataHolder.getCentroidInWorld();
      Vector3DReadOnly normal = regionDataHolder.getNormalInWorld();
      AxisAngle orientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(normal);

      List<Point2D> pointCloudInRegion = outerRing.parallelStream().map(pointInWorld ->
                                                                                {
                                                                                   return PolygonizerTools.toPointInPlane(pointInWorld, centroid, orientation);
                                                                                }).toList();

      long startTime = System.nanoTime();
      ConcaveHullCollection concaveHullCollection = SimpleConcaveHullFactory.createConcaveHullCollection(pointCloudInRegion, concaveHullFactoryParameters);

      // Apply some simple filtering to reduce the number of vertices and hopefully the number of convex polygons.
      double shallowAngleThreshold = polygonizerParameters.getShallowAngleThreshold();
      double peakAngleThreshold = polygonizerParameters.getPeakAngleThreshold();
      double lengthThreshold = polygonizerParameters.getLengthThreshold();

      ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullCollection);
      ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHullCollection);
      if (polygonizerParameters.getCutNarrowPassage())
         concaveHullCollection = ConcaveHullPruningFilteringTools.concaveHullNarrowPassageCutter(lengthThreshold, concaveHullCollection);

      return createSteppableRegions(centroid, orientation, concaveHullCollection, heightMapData, regionDataHolder);
   }

   private static List<Point3D> getOuterRingPoints(SteppableRegionDataHolder regionDataHolder, HeightMapData heightMapData, double inflationFraction)
   {
      if (regionDataHolder.getBorderRings().size() == 0)
         return null;

      List<List<SteppableCell>> ringList = new ArrayList<>(regionDataHolder.getBorderRings());
      ringList.sort(Comparator.comparingInt(List::size));

      double inflationSize = inflationFraction * heightMapData.getGridResolutionXY() / 2.0;

      List<SteppableCell> longestRing = ringList.get(ringList.size() - 1);
      List<Point3D> points = new ArrayList<>();
      for (SteppableCell cell : longestRing)
      {
         double x = HeightMapTools.indexToCoordinate(cell.x,
                                                     heightMapData.getGridCenter().getX(),
                                                     heightMapData.getGridResolutionXY(),
                                                     heightMapData.getCenterIndex());
         double y = HeightMapTools.indexToCoordinate(cell.y,
                                                     heightMapData.getGridCenter().getY(),
                                                     heightMapData.getGridResolutionXY(),
                                                     heightMapData.getCenterIndex());
         double height = heightMapData.getHeightAt(cell.x, cell.y);

         if (longestRing.size() > 100)
         {
            points.add(new Point3D(x, y, height));
         }
         else
         {
            points.add(new Point3D(x + inflationSize, y + inflationSize, height));
            points.add(new Point3D(x + inflationSize, y - inflationSize, height));
            points.add(new Point3D(x - inflationSize, y - inflationSize, height));
            points.add(new Point3D(x - inflationSize, y + inflationSize, height));
         }
      }
      return points;
   }


   public static List<SteppableRegion> createSteppableRegions(Point3DReadOnly origin,
                                                              Orientation3DReadOnly orientation,
                                                              ConcaveHullCollection concaveHullCollection,
                                                              HeightMapData heightMapData,
                                                              SteppableRegionDataHolder regionDataHolder)
   {
      List<SteppableRegion> regions = concaveHullCollection.getConcaveHulls()
                                                           .parallelStream()
                                                           .map(hull -> createSteppableRegion(origin, orientation, hull))
                                                           .toList();
      HeightMapData regionHeightMap = createHeightMapForRegion(regionDataHolder, heightMapData);
      regions.forEach(region -> region.setLocalHeightMap(regionHeightMap));
      return regions;
   }

   public static SteppableRegion createSteppableRegion(Point3DReadOnly origin, Orientation3DReadOnly orientation, ConcaveHull concaveHull)
   {
      return new SteppableRegion(origin, orientation, concaveHull.getConcaveHullVertices());
   }

   public static HeightMapData createHeightMapForRegion(SteppableRegionDataHolder regionDataHolder,
                                                        HeightMapData heightMapData)
   {
      double resolution = heightMapData.getGridResolutionXY();
      int xWidth = regionDataHolder.getMaxX() - regionDataHolder.getMinX();
      int yWidth = regionDataHolder.getMaxY() - regionDataHolder.getMinY();

      HeightMapData regionHeightMap = new HeightMapData(resolution, heightMapData.getGridSizeXY(), heightMapData.getGridCenter().getX(), heightMapData.getGridCenter().getY());

      for (SteppableCell cell : regionDataHolder.getCells())
      {
         int key = HeightMapTools.indicesToKey(cell.x, cell.y, heightMapData.getCenterIndex());
         double height = heightMapData.getHeightAt(key);
         if (Double.isNaN(height))
            LogTools.info("shoot");
         regionHeightMap.setHeightAt(key, height);//cell.getZ());
      }

      return regionHeightMap;
   }

   private static SteppableRegionsEnvironmentModel createUnsortedSteppableRegionEnvironment(BytedecoImage steppability, BytedecoImage snappedHeight, BytedecoImage connections)
   {
      int cellsPerSide = steppability.getImageHeight();
      SteppableRegionsEnvironmentModel steppableRegionsToConvert = new SteppableRegionsEnvironmentModel(cellsPerSide);

      for (int x = 0; x < cellsPerSide; x++)
      {
         for (int y = 0; y < cellsPerSide; y++)
         {
            if (x == 0 && y == 0)
               continue;

            int row = cellsPerSide - x - 1;
            int col = cellsPerSide - y - 1;
            // this cell is steppable. Also remember the image x-y is switched
            if (steppability.getInt(row, col) == 0)
            {
               boolean isBorderCell = connections.getInt(row, col) == 255;
               double z = snappedHeight.getFloat(row, col);
               SteppableCell cell = new SteppableCell(x, y, z, cellsPerSide, isBorderCell);
               steppableRegionsToConvert.addUnexpandedSteppableCell(cell);
            }
         }
      }

      return steppableRegionsToConvert;
   }

   private static void recursivelyAddNeighbors(SteppableCell cellToExpand,
                                               BytedecoImage steppability,
                                               BytedecoImage connections,
                                               SteppableRegionsEnvironmentModel environmentModel,
                                               int maxDepth,
                                               int currentDepth)
   {
      if (!cellToExpand.cellHasBeenAssigned())
         throw new RuntimeException("Should only be expanding assigned cells.");

      environmentModel.markCellAsExpanded(cellToExpand);

      int cellsPerSide = steppability.getImageHeight();
      int row = cellsPerSide - cellToExpand.x - 1;
      int col = cellsPerSide - cellToExpand.y - 1;

      int boundaryConnectionsEncodedAsOnes = connections.getInt(row, col);

      int counter = 0;
      for (int x_offset = -1; x_offset <= 1; x_offset++)
      {
         for (int y_offset = -1; y_offset <= 1; y_offset++)
         {
            if (x_offset == 0 && y_offset == 0)
               continue;

            // These are switched because of the difference between coordinates in the height map and image frame
            int neighborX = cellToExpand.x - y_offset;
            int neighborY = cellToExpand.y - x_offset;

            if (neighborX < 0 || neighborY < 0 || neighborX >= cellsPerSide || neighborY >= cellsPerSide)
               continue;

            SteppableCell neighbor = environmentModel.getCellAt(neighborX, neighborY);
            if (neighbor == null)
               continue;
            //            throw new? RuntimeException("This should have never happened.");

            if (isConnected(counter, boundaryConnectionsEncodedAsOnes))
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
                  cellToExpand.getRegion().addCell(neighbor);
               }

               if (!neighbor.cellHasBeenExpanded() && currentDepth < maxDepth && cellToExpand.cellHasBeenAssigned())
                  recursivelyAddNeighbors(neighbor, steppability, connections, environmentModel, maxDepth, currentDepth + 1);
            }

            counter++;
         }
      }
   }

   private static void recursivelyAddBorderNeighbors(SteppableCell cellToExpand,
                                                     BytedecoImage steppability,
                                                     BytedecoImage connections,
                                                     SteppableRegionsEnvironmentModel environmentModel,
                                                     int maxDepth,
                                                     int currentDepth)
   {
      if (!cellToExpand.cellHasBeenAssigned())
         throw new RuntimeException("Should only be expanding assigned cells.");

      environmentModel.markCellAsExpanded(cellToExpand);

      int cellsPerSide = steppability.getImageHeight();
      int row = cellsPerSide - cellToExpand.x - 1;
      int col = cellsPerSide - cellToExpand.y - 1;

      int boundaryConnectionsEncodedAsOnes = connections.getInt(row, col);

      int counter = 0;
      for (int x_offset = -1; x_offset <= 1; x_offset++)
      {
         for (int y_offset = -1; y_offset <= 1; y_offset++)
         {
            if (x_offset == 0 && y_offset == 0)
               continue;

            // These are switched because of the difference between coordinates in the height map and image frame
            int neighborX = cellToExpand.x - y_offset;
            int neighborY = cellToExpand.y - x_offset;

            if (neighborX < 0 || neighborY < 0 || neighborX >= cellsPerSide || neighborY >= cellsPerSide)
               continue;

            SteppableCell neighbor = environmentModel.getCellAt(neighborX, neighborY);
            if (neighbor == null)
               continue;
            //            throw new? RuntimeException("This should have never happened.");

            if (isConnected(counter, boundaryConnectionsEncodedAsOnes))
            {

               if (neighbor.cellHasBeenAssigned())
               {
                  SteppableRegionDataHolder neighborRegion = neighbor.getRegion();
                  if (neighbor.isBorderCell())
                  {
                     if (cellToExpand.getRegion() != neighborRegion || cellToExpand.getBorderRingNumber() != neighbor.getBorderRingNumber())
                     {
                        List<SteppableCell> neighborRing = neighbor.getBorderRing();
                        neighborRing.forEach(cell -> cell.setBorderRingNumber(cellToExpand.getBorderRingNumber()));
                        if (cellToExpand.getRegion() != neighborRegion)
                           cellToExpand.getBorderRing().addAll(neighborRing);
                     }
                  }

                  if (cellToExpand.getRegion().mergeRegion(neighborRegion))
                     environmentModel.getRegions().remove(neighborRegion);
               }
               else
               {
                  // the cell has not been assigned
                  cellToExpand.getRegion().addCell(neighbor);
                  if (neighbor.isBorderCell())
                  {
                     cellToExpand.getBorderRing().add(neighbor);
                     neighbor.setBorderRingNumber(cellToExpand.getBorderRingNumber());
                  }
               }

               if (neighbor.isBorderCell() && !neighbor.cellHasBeenExpanded() && currentDepth < maxDepth && cellToExpand.cellHasBeenAssigned())
                  recursivelyAddBorderNeighbors(neighbor, steppability, connections, environmentModel, maxDepth, currentDepth + 1);
            }

            counter++;
         }
      }
   }

   private static boolean isConnected(int counter, int connectionValue)
   {
      int mask = (1 << counter);
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
      int planarRegionIslandIndex = 0;

      private final Stack<SteppableCell> unexpandedInteriorCellsInTheEnvironment = new Stack<>();
      private final Stack<SteppableCell> unexpandedBorderCellsInTheEnvironment = new Stack<>();
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

      public SteppableRegionDataHolder getRegion(int i)
      {
         return steppableRegions.get(i);
      }

      public Collection<SteppableRegionDataHolder> getRegions()
      {
         return steppableRegions;
      }

      public boolean hasUnexpandedInteriorCells()
      {
         return !unexpandedInteriorCellsInTheEnvironment.isEmpty();
      }

      public boolean hasUnexpandedBorderCells()
      {
         return !unexpandedBorderCellsInTheEnvironment.isEmpty();
      }

      public SteppableCell getNextUnexpandedInteriorCell()
      {
         SteppableCell cell = unexpandedInteriorCellsInTheEnvironment.remove(0);
         cell.setCellHasBeenExpanded(true);
         return cell;
      }

      public SteppableCell getNextUnexpandedBorderCell()
      {
         SteppableCell cell = unexpandedBorderCellsInTheEnvironment.remove(0);
         cell.setCellHasBeenExpanded(true);
         return cell;
      }

      public void markCellAsExpanded(SteppableCell cell)
      {
         if (cell.isBorderCell())
            unexpandedBorderCellsInTheEnvironment.remove(cell);
         else
            unexpandedInteriorCellsInTheEnvironment.remove(cell);
         cell.setCellHasBeenExpanded(true);
      }

      public void addUnexpandedSteppableCell(SteppableCell cell)
      {
         if (cell.isBorderCell())
            unexpandedBorderCellsInTheEnvironment.add(cell);
         else
            unexpandedInteriorCellsInTheEnvironment.add(cell);
         steppableCellsGrid[cell.x][cell.y] = cell;
      }

      public SteppableRegionDataHolder createNewSteppableRegion()
      {
         SteppableRegionDataHolder steppableRegion = new SteppableRegionDataHolder(planarRegionIslandIndex++);
         steppableRegions.add(steppableRegion);

         return steppableRegion;
      }
   }

   public static class SteppableRegionDataHolder
   {
      public final int regionNumber;
      private boolean isCellPointCloudUpToDate = false;
      private boolean isCentroidUpToDate = false;
      private boolean isNormalUpToDate = false;

      private final List<SteppableCell> memberCells = new ArrayList<>();
      private final List<List<SteppableCell>> borderCellRings = new ArrayList<>();

      private List<Point3D> pointCloud;
      private final Point3D regionCentroidTotal = new Point3D();
      private final Point3D regionCentroid = new Point3D();
      private final Vector3D regionNormal = new Vector3D();

      private int minX = Integer.MAX_VALUE;
      private int maxX = Integer.MIN_VALUE;
      private int minY = Integer.MAX_VALUE;
      private int maxY = Integer.MIN_VALUE;

      public SteppableRegionDataHolder(int regionNumber)
      {
         this.regionNumber = regionNumber;
      }

      public int createNewBorderRing()
      {
         int size = borderCellRings.size();
         borderCellRings.add(new ArrayList<>());
         return size;
      }

      public List<SteppableCell> getBorderRing(int i)
      {
         return borderCellRings.get(i);
      }

      public List<List<SteppableCell>> getBorderRings()
      {
         return borderCellRings;
      }

      public void removeBorderRing(int ringNumber)
      {
         borderCellRings.remove(ringNumber);
      }

      public void addCell(SteppableCell cell)
      {
         markChanged();
         memberCells.add(cell);
         cell.setRegion(this);

         minX = Math.min(minX, cell.x);
         maxX = Math.max(maxX, cell.x);
         minY = Math.min(minY, cell.y);
         maxY = Math.max(maxY, cell.y);
      }

      public boolean mergeRegion(SteppableRegionDataHolder other)
      {
         if (this == other)
            return false;

         for (SteppableCell otherCell : other.memberCells)
         {
            addCell(otherCell);
         }
         this.borderCellRings.addAll(other.borderCellRings);

         return true;
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

      public int getMinX()
      {
         return minX;
      }

      public int getMaxX()
      {
         return maxX;
      }

      public int getMinY()
      {
         return minY;
      }

      public int getMaxY()
      {
         return maxY;
      }
   }

   public static class SteppableCell
   {
      private final int x;
      private final int y;
      private final double z;

      private final int hashCode;
      private final boolean isBorderCell;
      private int borderRingNumber = -1;

      private SteppableRegionDataHolder region;
      private boolean cellHasBeenExpanded = false;

      public SteppableCell(int x, int y, double z, int cellsPerSide, boolean isBorderCell)
      {
         this.x = x;
         this.y = y;
         this.z = z;
         this.isBorderCell = isBorderCell;

         hashCode = x * cellsPerSide + y;
      }

      public void setCellHasBeenExpanded(boolean cellHasBeenExpanded)
      {
         this.cellHasBeenExpanded = cellHasBeenExpanded;
      }

      public boolean getCellHasBeenExpanded()
      {
         return cellHasBeenExpanded;
      }

      public boolean isBorderCell()
      {
         return isBorderCell;
      }

      public int getX()
      {
         return x;
      }

      public int getY()
      {
         return y;
      }

      public double getZ()
      {
         return z;
      }

      public boolean hasRegion()
      {
         return getRegion() != null;
      }

      public SteppableRegionDataHolder getRegion()
      {
         return region;
      }

      public List<SteppableCell> getBorderRing()
      {
         return region.getBorderRing(getBorderRingNumber());
      }

      public void setRegion(SteppableRegionDataHolder region)
      {
         this.region = region;
      }

      public int getBorderRingNumber()
      {
         return borderRingNumber;
      }

      public boolean hasBorderRingNumber()
      {
         return borderRingNumber > -1;
      }

      public void setBorderRingNumber(int i)
      {
         borderRingNumber = i;
      }

      public boolean cellHasBeenAssigned()
      {
         return region != null;
      }

      public boolean cellHasBeenExpanded()
      {
         return cellHasBeenExpanded;
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
