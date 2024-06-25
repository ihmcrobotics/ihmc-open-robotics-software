package us.ihmc.sensors;

import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.zed.SL_InitParameters;

import javax.annotation.Nullable;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.function.Supplier;

import static us.ihmc.zed.global.zed.*;

// https://www.stereolabs.com/docs/video/recording
public class ZEDColorDepthImageRetrieverSVO extends ZEDColorDepthImageRetriever
{
   public enum RecordMode
   {
      RECORD, PLAYBACK
   }

   private final RecordMode recordMode;
   private String svoFileName;

   public ZEDColorDepthImageRetrieverSVO(int cameraID,
                                         Supplier<ReferenceFrame> sensorFrameSupplier,
                                         ROS2DemandGraphNode depthDemandNode,
                                         ROS2DemandGraphNode colorDemandNode,
                                         RecordMode recordMode,
                                         String svoFileName)
   {
      super(cameraID, sensorFrameSupplier, depthDemandNode, colorDemandNode);

      if (recordMode == RecordMode.PLAYBACK && svoFileName == null)
      {
         throw new RuntimeException("Must specify an SVO file name for playback");
      }

      this.recordMode = recordMode;
      this.svoFileName = svoFileName;
   }

   @Override
   protected boolean openCamera()
   {
      SL_InitParameters initParameters = getInitParameters();
      boolean openCameraResult = checkError("sl_open_camera", sl_open_camera(getCameraID(), initParameters, 0, svoFileName, "", 0, "", "", ""));
      if (recordMode == RecordMode.RECORD)
         sl_enable_recording(getCameraID(), svoFileName, SL_SVO_COMPRESSION_MODE_H264, 8000, 30, true);
      return openCameraResult;
   }

   @Override
   protected SL_InitParameters getInitParameters()
   {
      SL_InitParameters parentInitParameters = super.getInitParameters();
      parentInitParameters.svo_real_time_mode(true);
      if (recordMode == RecordMode.PLAYBACK)
         parentInitParameters.input_type(SL_INPUT_TYPE_SVO);
      return parentInitParameters;
   }

   @Override
   public void start()
   {
      if (recordMode == RecordMode.RECORD)
      {
         SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
         String depthLogFileName = dateFormat.format(new Date()) + "_" + "ZEDRecording.svo2";
         svoFileName = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.toAbsolutePath() + "/" + depthLogFileName;
         LogTools.info("| | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | ");
         LogTools.info("Starting recording: " + svoFileName);
         LogTools.info("| | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | ");
      }
      else
      {
         LogTools.info("| | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | ");
         LogTools.info("Starting playback: " + svoFileName);
         LogTools.info("| | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | ");
      }

      super.start();
   }

   @Override
   public void stop()
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

      svoFileName = null;

      super.stop();
   }

   @Nullable
   public String getSVOFileName()
   {
      return svoFileName;
   }
}
