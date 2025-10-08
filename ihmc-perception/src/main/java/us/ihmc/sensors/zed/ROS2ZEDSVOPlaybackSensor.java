package us.ihmc.sensors.zed;

import perception_msgs.msg.dds.ZEDSVOCurrentFileMessage;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;

public class ROS2ZEDSVOPlaybackSensor extends ZEDSVOPlaybackSensor
{
   private final ZEDSVOCurrentFileMessage svoStatusMessage = new ZEDSVOCurrentFileMessage();
   private final RepeatingTaskThread publishInfoThread;

   public ROS2ZEDSVOPlaybackSensor(ROS2PublishSubscribeAPI ros2, int cameraID, ZEDModelData zedModel, int slDepthMode, String svoFileName)
   {
      super(cameraID, zedModel, slDepthMode, svoFileName);

      ros2.subscribeViaCallback(PerceptionAPI.ZED_SVO_PAUSE, this::pause);
      ros2.subscribeViaCallback(PerceptionAPI.ZED_SVO_PLAY, this::play);
      ros2.subscribeViaCallback(PerceptionAPI.ZED_SVO_SET_POSITION, position ->
      {
         setCurrentPosition((int) position.getData());
         if (getGrabThread().getScheduled() == 0)
            getGrabThread().setScheduled(1);
      });

      publishInfoThread = new RepeatingTaskThread("PublishSVOInfoThread", () ->
      {
         svoStatusMessage.setCurrentFileName(svoFileName);
         svoStatusMessage.setRecordMode((byte) 1); // playback
         svoStatusMessage.setCurrentPosition(getCurrentPosition());
         svoStatusMessage.setLength(getLength());
         ros2.publish(PerceptionAPI.ZED_SVO_CURRENT_FILE, svoStatusMessage);
      });
   }

   @Override
   protected boolean startSensor()
   {
      if (super.startSensor())
      {
         publishInfoThread.setFrequencyLimit(getFps()).startRepeating();
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public void close()
   {
      publishInfoThread.blockingKill();

      super.close();
   }
}
