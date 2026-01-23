package us.ihmc.sensors.zed;

import perception_msgs.msg.dds.ZEDSVOCurrentFileMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;

public class ROS2ZEDSVOPlaybackSensor extends ZEDSVOPlaybackSensor
{
   private final ZEDSVOCurrentFileMessage svoStatusMessage = new ZEDSVOCurrentFileMessage();
   private final ROS2PublishSubscribeAPI ros2;

   public ROS2ZEDSVOPlaybackSensor(ROS2PublishSubscribeAPI ros2, int cameraID, ZEDModelData zedModel, int slDepthMode, String svoFileName)
   {
      super(cameraID, zedModel, slDepthMode, svoFileName);

      this.ros2 = ros2;

      ros2.subscribeViaCallback(PerceptionAPI.ZED_SVO_PAUSE, this::pause);
      ros2.subscribeViaCallback(PerceptionAPI.ZED_SVO_PLAY, this::play);
      ros2.subscribeViaCallback(PerceptionAPI.ZED_SVO_SET_POSITION, position ->
      {
         setCurrentPosition((int) position.getData() + 1);
         if (getGrabThread().getScheduled() == 0)
            getGrabThread().setScheduled(1);
      });
   }

   private void publishSVOInfo()
   {
      svoStatusMessage.setCurrentFileName(svoFileName);
      svoStatusMessage.setRecordMode((byte) 1); // playback
      svoStatusMessage.setCurrentPosition(getCurrentPosition());
      svoStatusMessage.setLength(getLength());
      ros2.publish(PerceptionAPI.ZED_SVO_CURRENT_FILE, svoStatusMessage);
   }

   @Override
   public boolean grab()
   {
      boolean result = super.grab();
      publishSVOInfo();
      return result;
   }
}
