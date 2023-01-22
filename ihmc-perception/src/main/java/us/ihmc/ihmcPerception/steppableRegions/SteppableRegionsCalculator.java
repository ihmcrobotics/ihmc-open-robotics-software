package us.ihmc.ihmcPerception.steppableRegions;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.stream.Collectors;

public class SteppableRegionsCalculator
{
   private final List<SteppableCell> unassignedCells = new ArrayList<>();
   private SteppableCell[][] steppableCellGrid;
   private final List<SteppableRegionData> steppableRegions = new ArrayList<>();

   public List<ConvexPolygon2D> calculate(HeightMapData heightMapData, BytedecoImage steppability, BytedecoImage connections)
   {
      unassignedCells.clear();
      steppableRegions.clear();

      if (steppability.getImageHeight() != steppability.getImageWidth())
         throw new RuntimeException("Should be square");

      createAllTheCells(steppability);

      while (!unassignedCells.isEmpty())
         recursivelyAddNeighbors(unassignedCells.get(0), steppability, connections);

      return steppableRegions.parallelStream()
                             .map(region -> SteppableRegionsCalculator.convertToConvexRegion(heightMapData, region))
                             .collect(Collectors.toList());
   }

   private void createAllTheCells(BytedecoImage steppability)
   {
      int cellsPerSide = steppability.getImageHeight();
      steppableCellGrid = new SteppableCell[cellsPerSide][];
      for (int x = 0; x < cellsPerSide; x++)
      {
         steppableCellGrid[x] = new SteppableCell[cellsPerSide];
         for (int y = 0; y < cellsPerSide; y++)
         {
            // this cell is steppable
            if (steppability.getInt(y, x) == 1)
            {
               SteppableCell cell = new SteppableCell(x, y, cellsPerSide);
               unassignedCells.add(cell);
               steppableCellGrid[x][y] = cell;
            }
         }
      }
   }

   private void recursivelyAddNeighbors(SteppableCell cell, BytedecoImage steppability, BytedecoImage connections)
   {
      if (!cell.hasRegion())
      {
         SteppableRegionData region = new SteppableRegionData();
         region.addCell(cell);

         steppableRegions.add(region);
      }
      unassignedCells.remove(cell);

      int boundaryConnectionsEncodedAsOnes = connections.getInt(cell.x, cell.y);

      int cellsPerSide = steppability.getImageHeight();
      for (int i = 0; i < 4; i++)
      {
         int neighborX = cell.x;
         int neighborY = cell.y;

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
            SteppableCell neighbor = steppableCellGrid[neighborX][neighborY];
            if (unassignedCells.contains(neighbor))
            {
               cell.getRegion().addCell(neighbor);
               recursivelyAddNeighbors(neighbor, steppability, connections);
            }
            else
            {
               cell.getRegion().mergeRegion(neighbor.getRegion());
            }
         }
      }
   }

   private boolean isConnected(int edgeNumber, int connectionValue)
   {
      int mask = (1 << edgeNumber);
      int maskedValue = mask & connectionValue;
      return maskedValue > 0;
   }

   private static ConvexPolygon2D convertToConvexRegion(HeightMapData heightMapData, SteppableRegionData steppableRegionData)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      steppableRegionData.memberCells.forEach(cell -> polygon.addVertex(getAsPoint(heightMapData, cell)));
      polygon.update();

      return polygon;
   }

   private static Point2D getAsPoint(HeightMapData heightMapData, SteppableCell steppableCell)
   {
      double x = HeightMapTools.indexToCoordinate(steppableCell.x,
                                                  heightMapData.getGridCenter().getX(),
                                                  heightMapData.getGridResolutionXY(),
                                                  heightMapData.getCenterIndex());
      double y = HeightMapTools.indexToCoordinate(steppableCell.y,
                                                  heightMapData.getGridCenter().getY(),
                                                  heightMapData.getGridResolutionXY(),
                                                  heightMapData.getCenterIndex());
      return new Point2D(x, y);
   }

   private static class SteppableRegionData
   {
      private final HashSet<SteppableCell> memberCells = new HashSet<>();

      public void addCell(SteppableCell cell)
      {
         memberCells.add(cell);
         cell.setRegion(this);
      }

      public void mergeRegion(SteppableRegionData other)
      {
         other.memberCells.forEach(this::addCell);
      }
   }

   private static class SteppableCell
   {
      private final int x;
      private final int y;

      private final int hashCode;

      private SteppableRegionData region;

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

      public SteppableRegionData getRegion()
      {
         return region;
      }

      public void setRegion(SteppableRegionData region)
      {
         this.region = region;
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
