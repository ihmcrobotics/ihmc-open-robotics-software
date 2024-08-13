package us.ihmc.sensors;

import perception_msgs.msg.dds.ZEDSVOCurrentFileMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.RestartableThrottledThread;
import us.ihmc.zed.SL_InitParameters;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static us.ihmc.zed.global.zed.*;

// https://www.stereolabs.com/docs/video/recording
public class ZEDColorDepthImageRetrieverSVO extends ZEDColorDepthImageRetriever
{
   private final RecordMode recordMode;
   private final String svoFileName;
   private final RestartableThrottledThread publishInfoThread;

   public ZEDColorDepthImageRetrieverSVO(int cameraID,
                                         BooleanSupplier depthDemandSupplier,
                                         BooleanSupplier colorDemandSupplier,
                                         ROS2Helper ros2Helper,
                                         RecordMode recordMode,
                                         String svoFileName)
   {
      this(cameraID, ReferenceFrame::getWorldFrame, true, depthDemandSupplier, colorDemandSupplier, ros2Helper, recordMode, svoFileName);

   }

   public ZEDColorDepthImageRetrieverSVO(int cameraID,
                                         Supplier<ReferenceFrame> sensorFrameSupplier,
                                         boolean useSensorPoseTracking,
                                         BooleanSupplier depthDemandSupplier,
                                         BooleanSupplier colorDemandSupplier,
                                         ROS2Helper ros2Helper,
                                         RecordMode recordMode,
                                         String svoFileName)
   {
      super(cameraID, sensorFrameSupplier, depthDemandSupplier, colorDemandSupplier, useSensorPoseTracking);

      if (recordMode == RecordMode.PLAYBACK && svoFileName == null)
      {
         throw new RuntimeException("Must specify an SVO file name for playback");
      }

      this.recordMode = recordMode;
      this.svoFileName = Objects.requireNonNullElseGet(svoFileName, this::generateSVOFileName);

      File svoFile = new File(this.svoFileName);

      if (recordMode == RecordMode.PLAYBACK && !svoFile.exists())
      {
         throw new RuntimeException("SVO file does not exist");
      }

      ros2Helper.subscribeViaCallback(PerceptionAPI.ZED_SVO_SET_POSITION, int64 ->
      {
         setCurrentPosition((int) int64.getData());
         if (!isRunning())
            grabOneFrame();
      });
      ros2Helper.subscribeViaCallback(PerceptionAPI.ZED_SVO_PAUSE, () ->
      {
         if (recordMode == RecordMode.RECORD)
         {
            pauseRecording(true);
         }
         else if (recordMode == RecordMode.PLAYBACK)
         {
            stop();
         }
      });
      ros2Helper.subscribeViaCallback(PerceptionAPI.ZED_SVO_PLAY, () ->
      {
         if (recordMode == RecordMode.RECORD)
         {
            pauseRecording(false);
         }
         else if (recordMode == RecordMode.PLAYBACK)
         {
            start();
         }
      });

      publishInfoThread = new RestartableThrottledThread("PublishSVOInfoThread", ZEDColorDepthImageRetriever.CAMERA_FPS, () ->
      {
         ZEDSVOCurrentFileMessage message = new ZEDSVOCurrentFileMessage();

         message.setCurrentFileName(this.svoFileName);
         message.setRecordMode(recordMode.toByte());
         message.setCurrentPosition(getCurrentPosition());
         message.setLength(getLength());

         ros2Helper.publish(PerceptionAPI.ZED_SVO_CURRENT_FILE, message);
      });

      publishInfoThread.start();

      if (recordMode == RecordMode.RECORD)
      {
         LogTools.info("| | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | ");
         LogTools.info("Starting recording: " + this.svoFileName);
         LogTools.info("| | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | ");
      }
      else
      {
         LogTools.info("| | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | ");
         LogTools.info("Starting playback: " + this.svoFileName);
         LogTools.info("| | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | ");
      }
   }

   private String generateSVOFileName()
   {
      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      String depthLogFileName = dateFormat.format(new Date()) + "_" + "ZEDRecording.svo2";
      return IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.toAbsolutePath() + "/" + depthLogFileName;
   }

   @Override
   protected boolean openCamera()
   {
      SL_InitParameters initParameters = getInitParameters();
      boolean openCameraResult = checkError("sl_open_camera", sl_open_camera(getCameraID(), initParameters, 0, svoFileName, "", 0, "", "", ""));
      if (recordMode == RecordMode.RECORD)
         sl_enable_recording(getCameraID(), svoFileName, SL_SVO_COMPRESSION_MODE_H264, 8000, ZEDColorDepthImageRetriever.CAMERA_FPS, true);
      return openCameraResult;
   }

   @Override
   protected SL_InitParameters getInitParameters()
   {
      SL_InitParameters parentInitParameters = super.getInitParameters();
      parentInitParameters.svo_real_time_mode(false);
      if (recordMode == RecordMode.PLAYBACK)
         parentInitParameters.input_type(SL_INPUT_TYPE_SVO);
      return parentInitParameters;
   }

   @Override
   public void destroy()
   {
      if (recordMode == RecordMode.RECORD)
      {
         // Cannot use LogTools here, this may get called on shutdown
         System.out.println("| | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | ");
         System.out.println("Stopping recording: " + svoFileName);
         System.out.println("| | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | ");
      }
      else
      {
         System.out.println("| | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | ");
         System.out.println("Stopping playback: " + svoFileName);
         System.out.println("| | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | ");
      }

      publishInfoThread.stop();

      super.destroy();
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

   public void pauseRecording(boolean pause)
   {
      sl_pause_recording(getCameraID(), pause);
   }

   public enum RecordMode
   {
      RECORD((byte) 0), PLAYBACK((byte) 1);

      private final byte byteValue;

      RecordMode(byte byteValue)
      {
         this.byteValue = byteValue;
      }

      public byte toByte()
      {
         return byteValue;
      }

      public static RecordMode fromByte(byte b)
      {
         for (RecordMode value : values())
         {
            if (value.byteValue == b)
               return value;
         }
         return null;
      }
   }
}
