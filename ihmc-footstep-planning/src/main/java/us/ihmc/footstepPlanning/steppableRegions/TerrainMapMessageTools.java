package us.ihmc.footstepPlanning.steppableRegions;

import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.idl.IDLSequence.Byte;

public class TerrainMapMessageTools
{
   public static TerrainMapMessage toMessage(TerrainMapData terrainMapData)
   {
      TerrainMapMessage message = new TerrainMapMessage();

      message.setWidthInMeters(terrainMapData.getCellsPerAxis());
      message.setCellsPerMeter((byte) terrainMapData.getCenterIndex());

      message.setMapCenterX(terrainMapData.getTerrainMapCenter().getX());
      message.setMapCenterY(terrainMapData.getTerrainMapCenter().getY());

      message.getTerrainCostData().add(terrainMapData.getTerrainCostMap());
      message.getContactMapData().add(terrainMapData.getContactMap());

      message.getHeights().add(terrainMapData.getHeightMap());
      message.getSnappedNormalXData().add(terrainMapData.getSnapNormalXMap());
      message.getSnappedNormalYData().add(terrainMapData.getSnapNormalYMap());
      message.getSnappedNormalZData().add(terrainMapData.getSnapNormalZMap());
      message.getSnappedAreaData().add(terrainMapData.getSnappedAreaFractionMap());
      message.getSteppabilityData().add(terrainMapData.getSteppabilityMap());
      message.getSteppableConnectionsData().add(terrainMapData.getSteppabilityConnectionsMap());

      return message;
   }

   public static TerrainMapData unpackMessage(TerrainMapMessage message)
   {
      int cellsPerAxis = (int) (message.getWidthInMeters() / message.getCellSizeInMeters());
      TerrainMapData terrainMapData = new TerrainMapData(cellsPerAxis, message.getCellSizeInMeters(), message.getWidthInMeters());

      Point2DReadOnly terrainMapCenter = new Point2D(message.getMapCenterX(), message.getMapCenterY());

      terrainMapData.setTerrainMapCenter(terrainMapCenter);

      Float heights = message.getHeights();
      float[] heightsArray = heights.toArray();
      terrainMapData.setHeightMap(heightsArray);

      Byte terrainCost = message.getTerrainCostData();
      byte[] terrainCostArray = terrainCost.copyArray();
      terrainMapData.setTerrainCostMap(terrainCostArray);

      Byte contactMap = message.getContactMapData();
      byte[] contactMapArray = contactMap.copyArray();
      terrainMapData.setContactMap(contactMapArray);

      Byte snappedNormalXMap = message.getSnappedNormalXData();
      byte[] snappedNormalXMapArray = snappedNormalXMap.copyArray();
      terrainMapData.setSnapNormalXMap(snappedNormalXMapArray);

      Byte snappedNormalYMap = message.getSnappedNormalYData();
      byte[] snappedNormalYMapArray = snappedNormalYMap.copyArray();
      terrainMapData.setSnapNormalYMap(snappedNormalYMapArray);

      Byte snappedNormalZMap = message.getSnappedNormalZData();
      byte[] snappedNormalZMapArray = snappedNormalZMap.copyArray();
      terrainMapData.setSnapNormalZMap(snappedNormalZMapArray);

      Byte steppabilityMap = message.getSteppabilityData();
      byte[] steppabilityMapArray = steppabilityMap.copyArray();
      terrainMapData.setSnappedAreaFractionMap(steppabilityMapArray);

      Byte steppabilityConnectionsMap = message.getSteppableConnectionsData();
      byte[] steppabilityConnectionsMapArray = steppabilityConnectionsMap.copyArray();
      terrainMapData.setSteppabilityConnectionsMap(steppabilityConnectionsMapArray);

      Byte snappedAreaFractionMap = message.getSnappedAreaData();
      byte[] snappedAreaFractionMapArray = snappedAreaFractionMap.copyArray();
      terrainMapData.setSnappedAreaFractionMap(snappedAreaFractionMapArray);

      return terrainMapData;
   }
}
