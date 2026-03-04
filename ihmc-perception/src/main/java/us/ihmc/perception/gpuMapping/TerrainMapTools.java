package us.ihmc.perception.gpuMapping;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class TerrainMapTools
{
   public static boolean isOutOfBounds(int cellsPerSide, int xIndex, int yIndex)
   {
      return xIndex < 0 || xIndex >= cellsPerSide || yIndex < 0 || yIndex >= cellsPerSide;
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
                                              Mat collisionMap,
                                              Point3DReadOnly gridCenter,
                                              TerrainMapData terrainMapData)
   {
      terrainMapData.setGridCenterX(gridCenter.getX());
      terrainMapData.setGridCenterY(gridCenter.getY());

      // How this looks like is we create a pointer for the Mat object.
      // Doing Pointer.get() takes in a parameter that will be packed with the data that is from the pointer.
      // So it looks like Pointer.get(dataToPack) where dataToPack = TerrainMapData.getMap()
      FloatPointer bytePointerForHeightMap = new FloatPointer(heightMap.data());
      bytePointerForHeightMap.get(terrainMapData.getHeightMap());

      BytePointer bytePointerForSnapNormalXMap = new BytePointer(snapNormalXMap.data());
      bytePointerForSnapNormalXMap.get(terrainMapData.getSnapNormalXMap());

      BytePointer bytePointerForSnapNormalYMap = new BytePointer(snapNormalYMap.data());
      bytePointerForSnapNormalYMap.get(terrainMapData.getSnapNormalYMap());

      BytePointer bytePointerForSnapNormalZMap = new BytePointer(snapNormalZMap.data());
      bytePointerForSnapNormalZMap.get(terrainMapData.getSnapNormalZMap());

      FloatPointer floatPointerForTraversabilityMap = new FloatPointer(traversabilityMap.data());
      floatPointerForTraversabilityMap.get(terrainMapData.getTraversabilityScoreMap());

      BytePointer bytePointerForTraversabilityClassMap = new BytePointer(traversabilityClassMap.data());
      bytePointerForTraversabilityClassMap.get(terrainMapData.getTraversabilityClassMap());

      FloatPointer floatPointerForCollisionMap = new FloatPointer(collisionMap.data());
      floatPointerForCollisionMap.get(terrainMapData.getObstacleClearanceScoreMap());
   }
}
