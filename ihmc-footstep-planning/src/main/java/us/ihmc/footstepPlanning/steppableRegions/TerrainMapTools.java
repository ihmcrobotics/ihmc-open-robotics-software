package us.ihmc.footstepPlanning.steppableRegions;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

public class TerrainMapTools
{
   /**
    * Computes and returns the index along the desired axis of the grid that matches this point. It is assumed that the coordinate and center values are
    * expressed in the same dimension of the same frame.
    *
    * @param cellsPerMeter this is the number of cells found in one meter of the grid, which maps to the resolution.
    * @param cellsPerSide  this is the cells per side of the grid.
    * @param coordinate    the coordinate in question.
    * @param center        the center of the grid along the defined axis.
    * @return index that contains the cell along the axis in question.
    */
   public static int getLocalIndex(int cellsPerMeter, int cellsPerSide, double coordinate, double center)
   {
      // TODO probably a height map tools method for this.
      return (int) ((coordinate - center) * cellsPerMeter + (double) cellsPerSide / 2);
   }

   public static boolean isOutOfBounds(int cellsPerSide, int rIndex, int cIndex)
   {
      return rIndex < 0 || rIndex >= cellsPerSide || cIndex < 0 || cIndex >= cellsPerSide;
   }

   /**
    * This method is meant to be as fast as possible, so we don't create local arrays, and we don't do any bounds checks.
    */
   public static void convertToTerrainMapData(Mat heightMap,
                                              Mat snapNormalXMap,
                                              Mat snapNormalYMap,
                                              Mat snapNormalZMap,
                                              Mat traversabilityMap,
                                              Mat traversabilityClassMap,
                                              TerrainMapData terrainMapData)
   {
      // How this looks like is we create a pointer for the Mat object.
      // Doing Pointer.get() takes in a parameter that will be packed with the data that is from the pointer.
      // So it looks like Pointer.get(dataToPack) where dataToPack = TerrainMapData.getMap()
      FloatPointer heightMapPointer = new FloatPointer(heightMap.data());
      heightMapPointer.get(terrainMapData.getHeightMap());

      BytePointer bytePointerForSnapNormalXMap = new BytePointer(snapNormalXMap.data());
      bytePointerForSnapNormalXMap.get(terrainMapData.getSnapNormalXMap());

      BytePointer bytePointerForSnapNormalYMap = new BytePointer(snapNormalYMap.data());
      bytePointerForSnapNormalYMap.get(terrainMapData.getSnapNormalYMap());

      BytePointer bytePointerForSnapNormalZMap = new BytePointer(snapNormalZMap.data());
      bytePointerForSnapNormalZMap.get(terrainMapData.getSnapNormalZMap());

      BytePointer bytePointerForTraversabilityMap = new BytePointer(traversabilityMap.data());
      bytePointerForTraversabilityMap.get(terrainMapData.getSnappedAreaFractionMap());

      BytePointer bytePointerForTraversabilityClassMap = new BytePointer(traversabilityClassMap.data());
      bytePointerForTraversabilityClassMap.get(terrainMapData.getSteppabilityMap());
   }

   public static UnitVector3DReadOnly computeSurfaceNormalInWorld(TerrainMapData terrainMapData, double x, double y)
   {
      int cellsPerMeter = terrainMapData.getCenterIndex();
      int localGridSize = terrainMapData.getCellsPerAxis();
      double centerX = terrainMapData.getTerrainMapCenter().getX();
      double centerY = terrainMapData.getTerrainMapCenter().getY();
      int rIndex = getLocalIndex(cellsPerMeter, localGridSize, x, centerX);
      int cIndex = getLocalIndex(cellsPerMeter, localGridSize, y, centerY);

      return terrainMapData.getNormalLocal(rIndex, cIndex);
   }
}
