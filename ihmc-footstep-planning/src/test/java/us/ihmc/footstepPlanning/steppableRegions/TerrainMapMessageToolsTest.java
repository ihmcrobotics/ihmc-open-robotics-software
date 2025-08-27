package us.ihmc.footstepPlanning.steppableRegions;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.TerrainMapMessage;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class TerrainMapMessageToolsTest
{
   @Test
   public void testMessagingRoundTrip()
   {
      double cellResolution = 0.2;
      double gridSizeXY = 1.0;
      int cellsPerAxis = (int) (gridSizeXY / cellResolution);

      TerrainMapData terrainMapData = new TerrainMapData(cellsPerAxis, cellResolution, gridSizeXY);

      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(100));
      Mat terrainCostMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(1));
      Mat contactMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(2));
      Mat snapNormalXMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(3));
      Mat snapNormalYMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(4));
      Mat snapNormalZMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(5));
      Mat steppabilityMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(6));
      Mat steppabilityConnectionsMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(7));
      Mat snappedAreaFractionMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(8));

      TerrainMapTools.convertToTerrainMapData(heightMap,
                                              terrainCostMap,
                                              contactMap,
                                              snapNormalXMap,
                                              snapNormalYMap,
                                              snapNormalZMap,
                                              steppabilityMap,
                                              steppabilityConnectionsMap,
                                              snappedAreaFractionMap,
                                              terrainMapData);

      TerrainMapMessage message = TerrainMapMessageTools.toMessage(terrainMapData);


      TerrainMapData resultingData = TerrainMapMessageTools.unpackMessage(message);

      // Check results match the input
      float[] heightMapResult = terrainMapData.getHeightMap();
      byte[] terrainCostMapResult = terrainMapData.getTerrainCostMap();
      byte[] contactMapResult = terrainMapData.getContactMap();
      byte[] snapNormalXMapResult = terrainMapData.getSnapNormalXMap();
      byte[] snapNormalYMapResult = terrainMapData.getSnapNormalYMap();
      byte[] snapNormalZMapResult = terrainMapData.getSnapNormalZMap();
      byte[] steppabilityMapResult = terrainMapData.getSteppabilityMap();
      byte[] steppabilityConnectionsMapResult = terrainMapData.getSteppabilityConnectionsMap();
      byte[] snappedAreaFractionMapResult = terrainMapData.getSnappedAreaFractionMap();

      int totalCells = cellsPerAxis * cellsPerAxis;

      for (int i = 0; i < totalCells; i++)
      {
         int x = i % cellsPerAxis;
         int y = i / cellsPerAxis;

         assertEquals(heightMapResult[i], heightMap.ptr(x, y).getFloat());
         assertEquals(terrainCostMapResult[i], terrainCostMap.ptr(x, y).get());
         assertEquals(contactMapResult[i], contactMap.ptr(x, y).get());
         assertEquals(snapNormalXMapResult[i], snapNormalXMap.ptr(x).get());
         assertEquals(snapNormalYMapResult[i], snapNormalYMap.ptr(x).get());
         assertEquals(snapNormalZMapResult[i], snapNormalZMap.ptr(x).get());
         assertEquals(steppabilityMapResult[i], steppabilityMap.ptr(x, y).get());
         assertEquals(steppabilityConnectionsMapResult[i], steppabilityConnectionsMap.ptr(x, y).get());
         assertEquals(snappedAreaFractionMapResult[i], snappedAreaFractionMap.ptr(x).get());
      }
   }
}
