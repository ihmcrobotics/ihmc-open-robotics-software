package us.ihmc.sensors.zed;

import us.ihmc.zed.SL_InitParameters;

import java.nio.file.Files;
import java.nio.file.Path;

import static us.ihmc.zed.global.zed.*;

public class ZEDSVOPlaybackSensor extends ZEDImageSensor
{
   private final String svoFileName;

   public ZEDSVOPlaybackSensor(int cameraID, ZEDModelData zedModel, String svoFileName)
   {
      super(cameraID, zedModel, SL_INPUT_TYPE_SVO);
      this.svoFileName = svoFileName;

      if (!Files.exists(Path.of(svoFileName)))
         throw new RuntimeException("SVO file does not exist");
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
   protected void openCamera() throws ZEDException
   {
      int returnCode = sl_open_camera(getCameraID(), zedInitParameters, 0, svoFileName, "", 0, "", "", "");
      throwOnError(returnCode);
   }

   @Override
   public void close()
   {
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
