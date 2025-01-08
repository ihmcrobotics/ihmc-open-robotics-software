package us.ihmc.sensors.zed;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.sensors.ImageSensorPosesFile;
import us.ihmc.tools.IHMCCommonPaths;

import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.time.Instant;
import java.util.Date;

import static us.ihmc.zed.global.zed.*;

public class ZEDSVORecordSensor extends ZEDImageSensor
{
   private ImageSensorPosesFile poseRecorder;

   public ZEDSVORecordSensor(int cameraID, ZEDModelData zedModel, int slInputType)
   {
      super(cameraID, zedModel, slInputType);
   }

   @Override
   protected void openCamera() throws ZEDException
   {
      String svoFileName = generateSVOFileName();

      int returnCode = sl_open_camera(cameraID, zedInitParameters, 0, svoFileName, "", 0, "", "", "");
      throwOnError(returnCode);

      returnCode = sl_enable_recording(getCameraID(), svoFileName, SL_SVO_COMPRESSION_MODE_H264, 8000, CAMERA_FPS, true);
      throwOnError(returnCode);

      poseRecorder = new ImageSensorPosesFile(Path.of(svoFileName + ".sensorposes"));
   }

   @Override
   protected boolean grab()
   {
      boolean grab = super.grab();

      if (poseRecorder != null)
      {
         Instant acquisitionTime = getImage(DEPTH_IMAGE_KEY).getAcquisitionTime();
         Point3DBasics position = getImage(DEPTH_IMAGE_KEY).getPosition();
         QuaternionBasics orientation = getImage(DEPTH_IMAGE_KEY).getOrientation();
         poseRecorder.recordFrameData(acquisitionTime, position, orientation);
      }

      return grab;
   }

   @Override
   public void close()
   {
      poseRecorder.flush();

      super.close();
   }

   private static String generateSVOFileName()
   {
      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      String depthLogFileName = dateFormat.format(new Date()) + "_" + "ZEDRecording.svo2";
      return IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.toAbsolutePath() + "/" + depthLogFileName;
   }
}
