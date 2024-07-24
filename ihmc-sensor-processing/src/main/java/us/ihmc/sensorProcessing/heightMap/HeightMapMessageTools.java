package us.ihmc.sensorProcessing.heightMap;

import perception_msgs.msg.dds.HeightMapMessage;

public class HeightMapMessageTools
{
   public static HeightMapData unpackMessage(HeightMapMessage heightMapMessage)
   {
      if (heightMapMessage == null)
         return null;

      HeightMapData heightMapData = new HeightMapData(heightMapMessage.getXyResolution(),
                                                      heightMapMessage.getGridSizeXy(),
                                                      heightMapMessage.getGridCenterX(),
                                                      heightMapMessage.getGridCenterY());

      for (int i = 0; i < heightMapMessage.getHeights().size(); i++)
      {
         double height = heightMapMessage.getHeights().get(i);
         int key = heightMapMessage.getKeys().get(i);
         heightMapData.setHeightAt(key, height);
      }

      heightMapData.setEstimatedGroundHeight(heightMapMessage.getEstimatedGroundHeight());
      return heightMapData;
   }

   public static HeightMapMessage toMessage(HeightMapData heightMapData)
   {
      HeightMapMessage message = new HeightMapMessage();
      toMessage(heightMapData, message);

      return message;
   }

   public static void toMessage(HeightMapData heightMapData, HeightMapMessage messageToPack)
   {
      clear(messageToPack);

      messageToPack.setGridSizeXy(heightMapData.getGridSizeXY());
      messageToPack.setXyResolution(heightMapData.getGridResolutionXY());
      messageToPack.setGridCenterX(heightMapData.getGridCenter().getX());
      messageToPack.setGridCenterY(heightMapData.getGridCenter().getY());
      messageToPack.setEstimatedGroundHeight(heightMapData.getEstimatedGroundHeight());

      for (int i = 0; i < heightMapData.getNumberOfOccupiedCells() && i < messageToPack.getKeys().capacity(); i++)
      {
         int key = heightMapData.getKey(i);
         messageToPack.getKeys().add(key);
         messageToPack.getHeights().add((float) heightMapData.getHeightAt(key));
      }
   }

   public static void clear(HeightMapMessage messageToClear)
   {
      messageToClear.setGridSizeXy(-1.0);
      messageToClear.setXyResolution(-1.0);
      messageToClear.setGridCenterX(-1.0);
      messageToClear.setGridCenterY(-1.0);
      messageToClear.setEstimatedGroundHeight(-1.0);

      messageToClear.getKeys().clear();
      messageToClear.getHeights().clear();
      messageToClear.getNormals().clear();
      messageToClear.getVariances().clear();
      messageToClear.getCentroids().clear();
   }
}
