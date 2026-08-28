package us.ihmc.sensors.zed;

import perception_msgs.ZEDSVOCurrentFileMessage;
import std_msgs.Int64;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.jros2.ROS2Node;

public class ROS2ZEDSVOPlaybackSensor extends ZEDSVOPlaybackSensor
{
   private final ZEDSVOCurrentFileMessage svoStatusMessage = new ZEDSVOCurrentFileMessage();
   private final ROS2PublisherMap publisherMap;

   public ROS2ZEDSVOPlaybackSensor(ROS2Node ros2Node, int cameraID, ZEDModelData zedModel, int slDepthMode, String svoFileName)
   {
      super(cameraID, zedModel, slDepthMode, svoFileName);

      this.publisherMap = new ROS2PublisherMap(ros2Node);

      ros2Node.createSubscription(PerceptionAPI.ZED_SVO_PAUSE, reader ->
      {
         if (reader.read() != null)
            pause();
      });
      ros2Node.createSubscription(PerceptionAPI.ZED_SVO_PLAY, reader ->
      {
         if (reader.read() != null)
            play();
      });
      ros2Node.createSubscription(PerceptionAPI.ZED_SVO_SET_POSITION, reader ->
      {
         Int64 position = reader.read();
         if (position != null)
         {
            setCurrentPosition((int) position.getData() + 1); // +1 is required to set the SVO to the correct frame. Otherwise, sets to 1 frame behind. Idk why.
            if (getGrabThread().getScheduled() == 0)
               getGrabThread().setScheduled(1);
         }
      });
   }

   private void publishSVOInfo()
   {
      svoStatusMessage.setCurrentFileName(svoFilePath);
      svoStatusMessage.setRecordMode((byte) 1); // playback
      svoStatusMessage.setCurrentPosition(getCurrentPosition());
      svoStatusMessage.setLength(getLength());
      publisherMap.publish(PerceptionAPI.ZED_SVO_CURRENT_FILE, svoStatusMessage);
   }

   @Override
   public boolean grab()
   {
      boolean result = super.grab();
      publishSVOInfo();
      return result;
   }
}
