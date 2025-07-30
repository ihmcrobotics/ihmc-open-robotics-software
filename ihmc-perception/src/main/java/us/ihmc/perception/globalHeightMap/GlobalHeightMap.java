package us.ihmc.perception.globalHeightMap;

import com.esotericsoftware.kryo.util.IntMap;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.heightMap.HeightMapTools;

import java.nio.ShortBuffer;
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
   // A map that stores GlobalMapTile objects using their hashed indices.
   private final IntMap<GlobalMapTile> heightMapDataIntMap = new IntMap<>();
   // A set to keep track of modified tiles.
   private final HashSet<GlobalMapTile> modifiedCells = new HashSet<>();

   public GlobalHeightMap()
   {
   }

   public void addHeightMap(Mat heightMap, Point3DReadOnly heightMapCenter, double gridSize, double gridResolution)
   {
      modifiedCells.clear();

      int centerIndex = HeightMapTools.computeCenterIndex(gridSize, gridResolution);

      int centerIndexLocal = HeightMapTools.computeCenterIndex(4.0, gridResolution);
      int cellsPerAxisLocal = 2 * centerIndexLocal + 1;
      int totalCells = cellsPerAxisLocal * cellsPerAxisLocal;
      // This is done for speed optimization
      short[] heightsArray = new short[totalCells];

      ShortBuffer shortBuffer = heightMap.createBuffer(); // or ByteBuffer -> ShortBuffer
      shortBuffer.get(heightsArray);

      for (int i = 0; i < heightMap.rows(); i++)
      {
         for (int j = 0; j < heightMap.cols(); j++)
         {
            double XCord = HeightMapTools.indexToCoordinate(i, heightMapCenter.getX(), gridResolution, centerIndex);
            double YCord = HeightMapTools.indexToCoordinate(j, heightMapCenter.getY(), gridResolution, centerIndex);

            GlobalMapTile globalMapTile = getOrCreateTileContainingCell(XCord, YCord, GlobalLattice.latticeWidth, gridResolution);

            int index = i * cellsPerAxisLocal + j;
            short height = heightsArray[index];

            globalMapTile.setHeightAt(XCord, YCord, height, gridResolution, centerIndex);

            modifiedCells.add(globalMapTile);
         }
      }
   }

   public Collection<GlobalMapTile> getModifiedMapTiles()
   {
      return modifiedCells;
   }

   public GlobalMapTile getOrCreateTileContainingCell(double x, double y, double tileSizeMeters, double resolution)
   {
      int tileWidthInCells = (int) Math.round(tileSizeMeters / resolution); // e.g., 5 / 0.02 = 250

      int globalCellX = (int) Math.floor(x / resolution);
      int globalCellY = (int) Math.floor(y / resolution);

      int tileIndexX = (int) Math.floor((double) globalCellX / tileWidthInCells);
      int tileIndexY = (int) Math.floor((double) globalCellY / tileWidthInCells);

      double tileCenterX = (tileIndexX + 0.5) * tileSizeMeters;
      double tileCenterY = (tileIndexY + 0.5) * tileSizeMeters;
      int hash = GlobalLattice.hashCodeOfTileIndices(tileCenterX, tileCenterY);

      GlobalMapTile tile = heightMapDataIntMap.get(hash);
      if (tile == null)
      {
         tile = new GlobalMapTile(resolution, tileCenterX, tileCenterY);
         heightMapDataIntMap.put(hash, tile);
      }

      return tile;
   }
}

