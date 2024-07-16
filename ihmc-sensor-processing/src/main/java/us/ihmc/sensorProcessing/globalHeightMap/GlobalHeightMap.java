package us.ihmc.sensorProcessing.globalHeightMap;

import com.esotericsoftware.kryo.util.IntMap;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.Collection;
import java.util.HashSet;

public class GlobalHeightMap
{
   private final IntMap<GlobalMapTile> heightMapDataIntMap = new IntMap<>();
   private final HashSet<GlobalMapTile> modifiedCells = new HashSet<>();

   public GlobalHeightMap()
   {
   }

   //adding a height map tile to the global height map
   public void addHeightMap(HeightMapData heightMapData)
   {
      modifiedCells.clear();

      //for a particular height map there are occupied cells, iterating over only the occupied cells and adding them to the globalmap tile
      for (int occupiedCell = 0; occupiedCell < heightMapData.getNumberOfOccupiedCells(); occupiedCell++)
      {
         double cellHeight = heightMapData.getHeight(occupiedCell);

         //get cell position in the globalMapTile
         Point2DReadOnly occupiedCellPosition = heightMapData.getCellPosition(occupiedCell);

         GlobalMapTile globalMapTile = getOrCreateDataContainingCell(occupiedCellPosition, heightMapData.getGridResolutionXY());

         globalMapTile.setHeightAt(occupiedCellPosition.getX(), occupiedCellPosition.getY(), cellHeight);

         modifiedCells.add(globalMapTile);
      }
   }

   public Collection<GlobalMapTile> getModifiedMapTiles()
   {
      return modifiedCells;
   }

   private GlobalMapTile getOrCreateDataContainingCell(Point2DReadOnly cellPosition, double resolution)
   {
      int xIndex = GlobalLattice.toIndex(cellPosition.getX());
      int yIndex = GlobalLattice.toIndex(cellPosition.getY());

      int hashOfMap = GlobalLattice.hashCodeOfTileIndices(xIndex, yIndex);
      GlobalMapTile data = heightMapDataIntMap.get(hashOfMap);

      if (data == null)
      {
         data = new GlobalMapTile(resolution, GlobalLattice.toPosition(xIndex), GlobalLattice.toPosition(yIndex));
         heightMapDataIntMap.put(hashOfMap, data);
      }

      return data;
   }
}

//# File: GlobalHeightMap.msg
//float64 grid_resolution_xy   # Resolution of the grid in XY plane
//float64 grid_size_xy         # Size of the grid in XY plane
//float64 grid_center_x        # X coordinate of the grid center
//float64 grid_center_y        # Y coordinate of the grid center
//float64[] heights            # Array of height values for each cell in the grid

