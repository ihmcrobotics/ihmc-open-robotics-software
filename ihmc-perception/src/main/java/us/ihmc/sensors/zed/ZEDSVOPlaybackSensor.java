package us.ihmc.sensors.zed;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.sensors.ImageSensorPosesFile;
import us.ihmc.zed.SL_InitParameters;

import javax.annotation.Nullable;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.zed.global.zed.*;

public class ZEDSVOPlaybackSensor extends ZEDImageSensor
{
   private final String svoFileName;
   @Nullable
   private ImageSensorPosesFile sensorPosesFile;
   private final MutableReferenceFrame sensorPosesFileFrame = new MutableReferenceFrame();

   public ZEDSVOPlaybackSensor(int cameraID, ZEDModelData zedModel, String svoFileName)
   {
      super(cameraID, zedModel, SL_INPUT_TYPE_SVO);
      this.svoFileName = svoFileName;

      if (!Files.exists(Path.of(svoFileName)))
         throw new RuntimeException("SVO file does not exist");

      // Default to not use any tracking
      useTrackedPose(false);
      useSensorPosesFile(false);
   }

   public void useTrackedPose(boolean useTrackedPose)
   {
      enablePositionalTracking(useTrackedPose);
      if (useTrackedPose)
         setSensorFrameSupplier(this::getTrackedSensorFrame);
   }

   public void useSensorPosesFile(boolean useSensorPosesFile)
   {
      if (useSensorPosesFile)
      {
         Path filePath = Path.of(svoFileName + ".sensorposes");

         if (Files.exists(filePath))
         {
            sensorPosesFile = new ImageSensorPosesFile(filePath);
            setSensorFrameSupplier(this::getSensorPosesFileFrame);
         }
      }
   }

   public ReferenceFrame getSensorPosesFileFrame()
   {
      return sensorPosesFileFrame.getReferenceFrame();
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
   protected boolean grab()
   {
      boolean grab = super.grab();

      if (sensorPosesFile != null)
      {
         // Right now, doing nothing with the read acquisition time
         AtomicReference<Instant> acquisitionTime = new AtomicReference<>();

         Point3D position = new Point3D();
         Quaternion orientation = new Quaternion();

         sensorPosesFile.readFrameData(grabSequenceNumber, acquisitionTime::set, position, orientation);

         if (!orientation.containsNaN() && !position.containsNaN())
            sensorPosesFileFrame.update(transformToWorld -> transformToWorld.set(orientation, position));
      }

      return grab;
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
