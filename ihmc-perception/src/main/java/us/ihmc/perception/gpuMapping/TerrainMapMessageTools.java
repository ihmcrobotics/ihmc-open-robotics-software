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

      float[] heightMap = message.getHeightMap().toArray();
      terrainMapData.setHeightMap(heightMap);

      float[] traversabilityScoreMap = message.getTraversabilityScore().toArray();
      if (traversabilityScoreMap.length > 0)
         terrainMapData.setTraversabilityScoreMap(traversabilityScoreMap);

      byte[] traversabilityClassMap = message.getTraversabilityClass().copyArray();
      if (traversabilityClassMap.length > 0)
         terrainMapData.setTraversabilityClassMap(traversabilityClassMap);

      byte[] snappedNormalXMap = message.getSnappedNormalXData().copyArray();
      if (snappedNormalXMap.length > 0)
         terrainMapData.setSnapNormalXMap(snappedNormalXMap);

      byte[] snappedNormalYMap = message.getSnappedNormalYData().copyArray();
      if (snappedNormalYMap.length > 0)
         terrainMapData.setSnapNormalYMap(snappedNormalYMap);

      byte[] snappedNormalZMap = message.getSnappedNormalZData().copyArray();
      if (snappedNormalZMap.length > 0)
         terrainMapData.setSnapNormalZMap(snappedNormalZMap);

      return terrainMapData;
   }
}
