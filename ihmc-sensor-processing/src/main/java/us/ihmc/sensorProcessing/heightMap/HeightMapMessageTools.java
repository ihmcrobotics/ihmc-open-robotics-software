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
      message.setGridSizeXy(heightMapData.getGridSizeXY());
      message.setXyResolution(heightMapData.getGridResolutionXY());
      message.setGridCenterX(heightMapData.getGridCenter().getX());
      message.setGridCenterY(heightMapData.getGridCenter().getY());

      for (int i = 0; i < heightMapData.getNumberOfOccupiedCells(); i++)
      {
         int key = heightMapData.getKey(i);
         message.getKeys().add(key);
         message.getHeights().add((float) heightMapData.getHeightAt(key));
      }

      return message;
   }
}
