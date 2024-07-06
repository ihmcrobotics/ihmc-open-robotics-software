package us.ihmc.sensorProcessing.globalHeightMap;

import com.esotericsoftware.kryo.util.IntMap;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.Collection;
import java.util.HashSet;

public class GlobalHeightMap
{
   private final IntMap<GlobalMapCell> heightMapDataIntMap = new IntMap<>();
   private final HashSet<GlobalMapCell> modifiedCells = new HashSet<>();

   public void addHeightMap(HeightMapData heightMapData)
   {
      modifiedCells.clear();

      for (int occupiedCell = 0; occupiedCell < heightMapData.getNumberOfOccupiedCells(); occupiedCell++)
      {
         double cellHeight = heightMapData.getHeight(occupiedCell);
         Point2DReadOnly cellPosition = heightMapData.getCellPosition(occupiedCell);

         GlobalMapCell globalMapCell = getOrCreateDataContainingCell(cellPosition, heightMapData.getGridResolutionXY());
         globalMapCell.setHeightAt(cellPosition.getX(), cellPosition.getY(), cellHeight);

         modifiedCells.add(globalMapCell);
      }
   }

   public Collection<GlobalMapCell> getModifiedMapCells()
   {
      return modifiedCells;
   }

   private GlobalMapCell getOrCreateDataContainingCell(Point2DReadOnly cellPosition, double resolution)
   {
      int xIndex = GlobalLattice.toIndex(cellPosition.getX());
      int yIndex = GlobalLattice.toIndex(cellPosition.getY());

      int hashOfMap = GlobalLattice.hashCodeOfCell(xIndex, yIndex);
      GlobalMapCell data = heightMapDataIntMap.get(hashOfMap);

      if (data == null)
      {
         data = new GlobalMapCell(resolution, GlobalLattice.toPosition(xIndex), GlobalLattice.toPosition(yIndex));
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

