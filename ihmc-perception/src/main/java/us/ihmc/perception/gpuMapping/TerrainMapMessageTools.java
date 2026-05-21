package us.ihmc.perception.gpuMapping;

import perception_msgs.TerrainMapMessage;
import us.ihmc.fastddsjava.cdr.idl.IDLByteSequence;
import us.ihmc.fastddsjava.cdr.idl.IDLFloatSequence;

public class TerrainMapMessageTools
{
   public static void toMessage(TerrainMapData terrainMapData, TerrainMapMessage message)
   {
      message.setCellsPerAxis(terrainMapData.getCellsPerAxis());
      message.setCellSizeInMeters(terrainMapData.getCellSize());
      message.setWidthInMeters(terrainMapData.getMapSize());
      message.setGridCenterX(terrainMapData.getGridCenterX());
      message.setGridCenterY(terrainMapData.getGridCenterY());

      message.getHeightMap().getBuffer().reset();
      message.getHeightMap().addAll(terrainMapData.getHeightMap());

      message.getObstacleClearanceScore().getBuffer().reset();
      message.getObstacleClearanceScore().addAll(terrainMapData.getObstacleClearanceScoreMap());
      message.getTraversabilityScore().getBuffer().reset();
      message.getTraversabilityScore().addAll(terrainMapData.getTraversabilityScoreMap());
      message.getTraversabilityClass().getBuffer().reset();
      message.getTraversabilityClass().addAll(terrainMapData.getTraversabilityClassMap());

      message.getSnappedNormalXData().getBuffer().reset();
      message.getSnappedNormalXData().addAll(terrainMapData.getSnapNormalXMap());
      message.getSnappedNormalYData().getBuffer().reset();
      message.getSnappedNormalYData().addAll(terrainMapData.getSnapNormalYMap());
      message.getSnappedNormalZData().getBuffer().reset();
      message.getSnappedNormalZData().addAll(terrainMapData.getSnapNormalZMap());
   }

   public static TerrainMapData unpackMessage(TerrainMapMessage message)
   {
      TerrainMapData terrainMapData = new TerrainMapData(message.getCellSizeInMeters(),
                                                         message.getWidthInMeters(),
                                                         message.getGridCenterX(),
                                                         message.getGridCenterY());

      float[] heightMap = copyFloatSequenceToArray(message.getHeightMap());
      terrainMapData.setHeightMap(heightMap);

      float[] obstacleClearanceScoreMap = copyFloatSequenceToArray(message.getObstacleClearanceScore());
      if (obstacleClearanceScoreMap.length > 0)
         terrainMapData.setObstacleClearanceScoreMap(obstacleClearanceScoreMap);

      float[] traversabilityScoreMap = copyFloatSequenceToArray(message.getTraversabilityScore());
      if (traversabilityScoreMap.length > 0)
         terrainMapData.setTraversabilityScoreMap(traversabilityScoreMap);

      byte[] traversabilityClassMap = copyByteSequenceToArray(message.getTraversabilityClass());
      if (traversabilityClassMap.length > 0)
         terrainMapData.setTraversabilityClassMap(traversabilityClassMap);

      byte[] snappedNormalXMap = copyByteSequenceToArray(message.getSnappedNormalXData());
      if (snappedNormalXMap.length > 0)
         terrainMapData.setSnapNormalXMap(snappedNormalXMap);

      byte[] snappedNormalYMap = copyByteSequenceToArray(message.getSnappedNormalYData());
      if (snappedNormalYMap.length > 0)
         terrainMapData.setSnapNormalYMap(snappedNormalYMap);

      byte[] snappedNormalZMap = copyByteSequenceToArray(message.getSnappedNormalZData());
      if (snappedNormalZMap.length > 0)
         terrainMapData.setSnapNormalZMap(snappedNormalZMap);

      return terrainMapData;
   }

   private static float[] copyFloatSequenceToArray(IDLFloatSequence sequence)
   {
      float[] array = new float[sequence.size()];
      for (int i = 0; i < array.length; i++)
         array[i] = sequence.get(i);
      return array;
   }

   private static byte[] copyByteSequenceToArray(IDLByteSequence sequence)
   {
      byte[] array = new byte[sequence.size()];
      for (int i = 0; i < array.length; i++)
         array[i] = sequence.get(i);
      return array;
   }
}
