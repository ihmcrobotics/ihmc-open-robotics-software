package us.ihmc.sensorProcessing.globalHeightMap;

import com.esotericsoftware.kryo.util.IntMap;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.Collection;
import java.util.HashSet;

/**
 * The GlobalHeightMap class processes local height maps to create a global height map.
 * This class iterates through occupied cells in the local height maps, converts them
 * into tiles, and stitches these tiles together to represent the area the robot has
 * visited and observed. The process is done in an egocentric manner, centering around
 * the robot's current position.
 */

public class GlobalHeightMap
{
   // Zero value in world minus the thickness of the foot
   private final static double MIN_GLOBAL_HEIGHT_MAP_VALUE = -0.02;

   // A map that stores GlobalMapTile objects using their hashed indices.
   private final IntMap<GlobalMapTile> heightMapDataIntMap = new IntMap<>();
   // A set to keep track of modified tiles.
   private final HashSet<GlobalMapTile> modifiedCells = new HashSet<>();

   public GlobalHeightMap()
   {
   }

   // Adds a local height map to the global height map.
   public void addHeightMap(HeightMapData heightMapData)
   {
      // Clear the set of modified cells before processing the new height map data
      modifiedCells.clear();

      // Iterate over each occupied cell in the local height map
      for (int occupiedCell = 0; occupiedCell < heightMapData.getNumberOfOccupiedCells(); occupiedCell++)
      {
         double cellHeight = heightMapData.getHeight(occupiedCell);

         // Get the height of the current occupied cell
         Point2DReadOnly occupiedCellPosition = heightMapData.getCellPosition(occupiedCell);
         // Get or create the GlobalMapTile that contains the current cell
         GlobalMapTile globalMapTile = getOrCreateDataContainingCell(occupiedCellPosition, heightMapData.getGridResolutionXY());

         if (Double.isNaN(globalMapTile.getEstimatedGroundHeight()))
         {
            globalMapTile.setEstimatedGroundHeight(MIN_GLOBAL_HEIGHT_MAP_VALUE);
         }

         // Set the height of the cell within the global map tile
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
      // Convert the cell position to global map tile indices
      int xIndex = GlobalLattice.toIndex(cellPosition.getX());
      int yIndex = GlobalLattice.toIndex(cellPosition.getY());
      // Generate a hash code for the tile indices
      int hashOfMap = GlobalLattice.hashCodeOfTileIndices(xIndex, yIndex);
      // Retrieve the global map tile from the map, or create a new one if it doesn't exist
      GlobalMapTile data = heightMapDataIntMap.get(hashOfMap);

      if (data == null)
      {
         data = new GlobalMapTile(resolution, GlobalLattice.toPosition(xIndex), GlobalLattice.toPosition(yIndex));
         heightMapDataIntMap.put(hashOfMap, data);
      }

      return data;
   }
}

