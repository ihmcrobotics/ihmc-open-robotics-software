package us.ihmc.perception.gpuHeightMap;

import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.idl.IDLSequence.Byte;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.perception.heightMap.HeightMapMessageTools;

public class TerrainMapMessageTools
{
   public static void toMessage(TerrainMapData terrainMapData, TerrainMapMessage message)
   {
      HeightMapMessageTools.toMessage(terrainMapData.getHeightMapData(), message.getHeightMap());

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
      HeightMapMessage heightMapMessage = message.getHeightMap();
      TerrainMapData terrainMapData = new TerrainMapData(heightMapMessage.getCellSizeInMeters(), heightMapMessage.getWidthInMeters(), heightMapMessage.getGridCenterX(), heightMapMessage.getGridCenterY());
      terrainMapData.setHeightMapData(HeightMapMessageTools.unpackMessageToHeightMapData(heightMapMessage));

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
