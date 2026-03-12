package us.ihmc.sensors.zed;

import us.ihmc.zed.SL_InitParameters;

import java.nio.file.Files;
import java.nio.file.Path;

import static us.ihmc.zed.global.zed.*;

public class ZEDSVOPlaybackSensor extends ZEDImageSensor
{
   private final int cameraID;
   protected final String svoFileName;

   public ZEDSVOPlaybackSensor(int cameraID, ZEDModelData zedModel, int slDepthMode, String svoFileName)
   {
      super(cameraID, zedModel, SL_INPUT_TYPE_SVO, slDepthMode);
      this.cameraID = cameraID;
      this.svoFileName = svoFileName;

      if (!Files.exists(Path.of(svoFileName)))
         throw new RuntimeException("SVO file does not exist");
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
   public boolean isSensorRunning()
   {
      return sl_is_opened(cameraID);
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

   public String getSVOFileName()
   {
      return svoFileName;
   }
}
