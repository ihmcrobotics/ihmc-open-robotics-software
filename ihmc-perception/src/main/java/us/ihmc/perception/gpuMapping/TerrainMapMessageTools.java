package us.ihmc.perception.gpuMapping;

import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.idl.IDLSequence.Byte;
import us.ihmc.idl.IDLSequence.Float;

public class TerrainMapMessageTools
{
   public static void toMessage(TerrainMapData terrainMapData, TerrainMapMessage message)
   {
      message.setCellsPerAxis(terrainMapData.getCellsPerAxis());
      message.setCellSizeInMeters(terrainMapData.getCellSize());
      message.setWidthInMeters(terrainMapData.getMapSize());
      message.setGridCenterX(terrainMapData.getGridCenterX());
      message.setGridCenterY(terrainMapData.getGridCenterY());

      message.getHeightMap().resetQuick();
      message.getHeightMap().add(terrainMapData.getHeightMap());

      message.getTraversabilityScore().resetQuick();
      message.getTraversabilityScore().add(terrainMapData.getTraversabilityScoreMap());
      message.getTraversabilityClass().resetQuick();
      message.getTraversabilityClass().add(terrainMapData.getTraversabilityClassMap());

      message.getSnappedNormalXData().resetQuick();
      message.getSnappedNormalXData().add(terrainMapData.getSnapNormalXMap());
      message.getSnappedNormalYData().resetQuick();
      message.getSnappedNormalYData().add(terrainMapData.getSnapNormalYMap());
      message.getSnappedNormalZData().resetQuick();
      message.getSnappedNormalZData().add(terrainMapData.getSnapNormalZMap());
   }

   public static TerrainMapData unpackMessage(TerrainMapMessage message)
   {
      TerrainMapData terrainMapData = new TerrainMapData(message.getCellSizeInMeters(),
                                                         message.getWidthInMeters(),
                                                         message.getGridCenterX(),
                                                         message.getGridCenterY());

      Float heightMap = message.getHeightMap();
      float[] heightMapArray = heightMap.toArray();
      terrainMapData.setHeightMap(heightMapArray);

      Float traversabilityScoreMap = message.getTraversabilityScore();
      float[] traversabilityScoreArray = traversabilityScoreMap.toArray();
      terrainMapData.setTraversabilityScoreMap(traversabilityScoreArray);

      Byte traversabilityClassMap = message.getTraversabilityClass();
      byte[] traversabilityClassArray = traversabilityClassMap.copyArray();
      terrainMapData.setTraversabilityClassMap(traversabilityClassArray);

      Byte snappedNormalXMap = message.getSnappedNormalXData();
      byte[] snappedNormalXMapArray = snappedNormalXMap.copyArray();
      terrainMapData.setSnapNormalXMap(snappedNormalXMapArray);

      Byte snappedNormalYMap = message.getSnappedNormalYData();
      byte[] snappedNormalYMapArray = snappedNormalYMap.copyArray();
      terrainMapData.setSnapNormalYMap(snappedNormalYMapArray);

      Byte snappedNormalZMap = message.getSnappedNormalZData();
      byte[] snappedNormalZMapArray = snappedNormalZMap.copyArray();
      terrainMapData.setSnapNormalZMap(snappedNormalZMapArray);

      return terrainMapData;
   }
}
