package us.ihmc.sensors.zed;

import us.ihmc.sensors.ImageSensorPoseRecorder;
import us.ihmc.tools.IHMCCommonPaths;

import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;

import static us.ihmc.zed.global.zed.*;

public class ZEDSVORecordSensor extends ZEDImageSensor
{
   private ImageSensorPoseRecorder poseRecorder;

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

      poseRecorder = new ImageSensorPoseRecorder(Path.of(svoFileName + ".sensorposes"));
   }

   @Override
   protected boolean grab()
   {
      boolean grab = super.grab();

      if (poseRecorder != null)
         poseRecorder.recordFrame(getImage(DEPTH_IMAGE_KEY));

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
