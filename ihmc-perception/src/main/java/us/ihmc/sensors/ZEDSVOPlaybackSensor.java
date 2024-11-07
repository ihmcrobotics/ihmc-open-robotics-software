package us.ihmc.sensors;

import perception_msgs.msg.dds.ZEDSVOCurrentFileMessage;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.zed.SL_InitParameters;

import java.nio.file.Files;
import java.nio.file.Path;

import static us.ihmc.zed.global.zed.*;

public class ZEDSVOPlaybackSensor extends ZEDImageSensor
{
   private final String svoFileName;
   private final ZEDSVOCurrentFileMessage svoStatusMessage = new ZEDSVOCurrentFileMessage();
   private final RepeatingTaskThread publishInfoThread;

   public ZEDSVOPlaybackSensor(ROS2PublishSubscribeAPI ros2, int cameraID, ZEDModelData zedModel, String svoFileName)
   {
      super(cameraID, zedModel, SL_INPUT_TYPE_SVO);
      this.svoFileName = svoFileName;

      if (!Files.exists(Path.of(svoFileName)))
         throw new RuntimeException("SVO file does not exist");

      // Subscription to set position message
      ros2.subscribeViaCallback(PerceptionAPI.ZED_SVO_SET_POSITION, position ->
      {
         setCurrentPosition((int) position.getData());
         if (getGrabThread().getScheduled() == 0)
            getGrabThread().setScheduled(1);
      });

      // Subscribe for pause message
      ros2.subscribeViaCallback(PerceptionAPI.ZED_SVO_PAUSE, () -> getGrabThread().stopRepeating());

      // Subscribe to play message
      ros2.subscribeViaCallback(PerceptionAPI.ZED_SVO_PLAY, () -> getGrabThread().startRepeating());

      publishInfoThread = new RepeatingTaskThread(() ->
      {
         svoStatusMessage.setCurrentFileName(svoFileName);
         svoStatusMessage.setRecordMode((byte) 1); // playback
         svoStatusMessage.setCurrentPosition(getCurrentPosition());
         svoStatusMessage.setLength(getLength());

         ros2.publish(PerceptionAPI.ZED_SVO_CURRENT_FILE, svoStatusMessage);
      }, "PublishSVOInfoThread").setFrequencyLimit(ZEDImageSensor.CAMERA_FPS);
      publishInfoThread.start();
   }

   public void useTrackedPose(boolean useTrackedPose)
   {
      enablePositionalTracking(useTrackedPose);
      if (useTrackedPose)
         setSensorFrameSupplier(this::getTrackedSensorFrame);
   }

   @Override
   protected void setInitParameters(SL_InitParameters parametersToSet)
   {
      super.setInitParameters(parametersToSet);
      parametersToSet.svo_real_time_mode(true);
   }

   @Override
   protected int openCamera()
   {
      return sl_open_camera(getCameraID(), zedInitParameters, 0, svoFileName, "", 0, "", "", "");
   }

   @Override
   public void close()
   {
      publishInfoThread.blockingKill();

      super.close();
   }

   public int getLength()
   {
      return sl_get_svo_number_of_frames(getCameraID());
   }

   public int getCurrentPosition()
   {
      return sl_get_svo_position(getCameraID());
   }

   public void setCurrentPosition(int position)
   {
      sl_set_svo_position(getCameraID(), position);
   }
}
