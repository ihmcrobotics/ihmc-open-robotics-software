package us.ihmc.footstepPlanning.steppableRegions;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.TerrainMapMessage;
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

   public static void convertToTerrainMapData(Mat heightMap,
                                              Mat terrainCostMap,
                                              Mat contactMap,
                                              Mat snapNormalXMap,
                                              Mat snapNormalYMap,
                                              Mat snapNormalZMap,
                                              Mat steppabilityMap,
                                              Mat steppabilityConnectionsMap,
                                              Mat snappedAreaFractionMap,
                                              TerrainMapData terrainMapData)
   {
      int centerIndex = terrainMapData.getCenterIndex();
      int cellsPerAxis = 2 * centerIndex + 1;
      int totalCells = cellsPerAxis * cellsPerAxis;

      FloatPointer floatPointer = new FloatPointer(heightMap.data());
      float[] values = new float[totalCells];
      floatPointer.get(values);
      terrainMapData.setHeightMap(values);

      BytePointer bytePointerTerrainMap = new BytePointer(terrainCostMap.data());
      byte[] bytesForTerrainCost = new byte[totalCells];
      bytePointerTerrainMap.get(bytesForTerrainCost);
      terrainMapData.setTerrainCostMap(bytesForTerrainCost);

      BytePointer bytePointerContactMap = new BytePointer(contactMap.data());
      byte[] bytesForContactMap = new byte[totalCells];
      bytePointerContactMap.get(bytesForContactMap);
      terrainMapData.setContactMap(bytesForContactMap);

      BytePointer bytePointerForSnapNormalXMap = new BytePointer(snapNormalXMap.data());
      byte[] bytesForSnapNormalX = new byte[totalCells];
      bytePointerForSnapNormalXMap.get(bytesForSnapNormalX);
      terrainMapData.setSnapNormalXMap(bytesForSnapNormalX);

      BytePointer bytePointerForSnapNormalYMap = new BytePointer(snapNormalYMap.data());
      byte[] bytesForSnapNormalY = new byte[totalCells];
      bytePointerForSnapNormalYMap.get(bytesForSnapNormalY);
      terrainMapData.setSnapNormalYMap(bytesForSnapNormalY);

      BytePointer bytePointerForSnapNormalZMap = new BytePointer(snapNormalZMap.data());
      byte[] bytesForSnapNormalZ = new byte[totalCells];
      bytePointerForSnapNormalZMap.get(bytesForSnapNormalZ);
      terrainMapData.setSnapNormalZMap(bytesForSnapNormalZ);

      BytePointer bytePointerForSnappedAreaFractionMap = new BytePointer(snappedAreaFractionMap.data());
      byte[] bytesForSnappedAreaFractionMap = new byte[totalCells];
      bytePointerForSnappedAreaFractionMap.get(bytesForSnappedAreaFractionMap);
      terrainMapData.setSnappedAreaFractionMap(bytesForSnappedAreaFractionMap);

      BytePointer bytePointerForSteppabilityMap = new BytePointer(steppabilityMap.data());
      byte[] bytesForSteppabilityMap = new byte[totalCells];
      bytePointerForSteppabilityMap.get(bytesForSteppabilityMap);
      terrainMapData.setSteppabilityMap(bytesForSteppabilityMap);

      BytePointer bytePointerForSteppabilityConnectionsMap = new BytePointer(steppabilityConnectionsMap.data());
      byte[] bytesForSteppabilityConnectionsMap = new byte[totalCells];
      bytePointerForSteppabilityConnectionsMap.get(bytesForSteppabilityConnectionsMap);
      terrainMapData.setSteppabilityConnectionsMap(bytesForSteppabilityConnectionsMap);
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
