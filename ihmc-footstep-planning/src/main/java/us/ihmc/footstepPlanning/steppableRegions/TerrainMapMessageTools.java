package us.ihmc.footstepPlanning.steppableRegions;

import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.idl.IDLSequence.Byte;

public class TerrainMapMessageTools
{
   public static void toMessage(TerrainMapData terrainMapData, TerrainMapMessage message)
   {
      message.setWidthInMeters(terrainMapData.getGridSizeXY());
      message.setCellsPerMeter((byte) terrainMapData.getCenterIndex());
      message.setMapCenterX(terrainMapData.getTerrainMapCenter().getX());
      message.setMapCenterY(terrainMapData.getTerrainMapCenter().getY());

      int totalCells = terrainMapData.getCellsPerAxis() *  terrainMapData.getCellsPerAxis();

      float[] heightsFromData = terrainMapData.getHeightMap();
      Float heights = message.getHeights();

      for (int i = 0; i < totalCells; i++)
      {
         if (i < heights.size())
            heights.set(i, heightsFromData[i]);
         else
            heights.add(heightsFromData[i]);
      }

      updateByteList(message.getTerrainCostData(), terrainMapData.getTerrainCostMap());
      updateByteList(message.getContactMapData(), terrainMapData.getContactMap());
      updateByteList(message.getSnappedNormalXData(), terrainMapData.getSnapNormalXMap());
      updateByteList(message.getSnappedNormalYData(), terrainMapData.getSnapNormalYMap());
      updateByteList(message.getSnappedNormalZData(), terrainMapData.getSnapNormalZMap());
      updateByteList(message.getSteppabilityData(),  terrainMapData.getSteppabilityMap());
      updateByteList(message.getSteppableConnectionsData(), terrainMapData.getSteppabilityConnectionsMap());
      updateByteList(message.getSnappedAreaData(), terrainMapData.getSnappedAreaFractionMap());
   }

   private static void updateByteList(Byte target, byte[] source)
   {
      for (int i = 0; i < source.length; i++)
      {
         byte value = source[i];
         if (i < target.size())
            target.set(i, value);
         else
            target.add(value);
      }
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
      terrainMapData.setSteppabilityMap(steppabilityMapArray);

      Byte steppabilityConnectionsMap = message.getSteppableConnectionsData();
      byte[] steppabilityConnectionsMapArray = steppabilityConnectionsMap.copyArray();
      terrainMapData.setSteppabilityConnectionsMap(steppabilityConnectionsMapArray);

      Byte snappedAreaFractionMap = message.getSnappedAreaData();
      byte[] snappedAreaFractionMapArray = snappedAreaFractionMap.copyArray();
      terrainMapData.setSnappedAreaFractionMap(snappedAreaFractionMapArray);

      return terrainMapData;
   }
}
