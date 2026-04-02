package us.ihmc.sensors.zed;

import static us.ihmc.zed.global.zed.*;

public class ZEDSVOPlaybackSensor extends ZEDImageSensor
{
   protected final String svoFilePath;

   public ZEDSVOPlaybackSensor(int cameraID, ZEDModelData zedModel, int slDepthMode, String svoFilePath)
   {
      super(cameraID, zedModel, slDepthMode, svoFilePath);

      this.svoFilePath = svoFilePath;
   }

   @Override
   public boolean startSensor()
   {
      return super.startSensor();
   }

   public void play()
   {
      run(true);
   }

   public void pause()
   {
      run(false);
   }

   public void useTrackedPose(boolean useTrackedPose)
   {
      enablePositionalTracking(useTrackedPose);
      if (useTrackedPose)
         setSensorFrame(getTrackedSensorFrame());
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

   public String getSVOFilePath()
   {
      return svoFilePath;
   }
}
