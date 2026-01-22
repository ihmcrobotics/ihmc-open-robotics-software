package us.ihmc.sensors.realsense;

import org.bytedeco.librealsense2.global.realsense2;
import org.bytedeco.librealsense2.rs2_device;
import us.ihmc.log.LogTools;

import java.nio.file.Files;
import java.nio.file.Path;

public class RealSenseBagPlaybackSensor extends RealSenseImageSensor
{
   private final String bagFileName;
   private final RealSenseDeviceManager realsenseManager;
   private rs2_device playbackDevice;

   public RealSenseBagPlaybackSensor(RealSenseConfiguration realsenseConfiguration, String bagFileName)
   {
      super(realsenseConfiguration);
      this.bagFileName = bagFileName;
      this.realsenseManager = new RealSenseDeviceManager();

      if (!Files.exists(Path.of(bagFileName)))
         throw new RuntimeException("BAG file does not exist: " + bagFileName);
   }

   @Override
   protected boolean startSensor()
   {
      if (realsense != null)
      {
         if (realsense.getDevice() != null)
            realsense.deleteDevice();
         realsense = null;
      }

      realsense = realsenseManager.createPlaybackDevice(realsenseConfiguration, bagFileName);

      boolean success = realsense != null;
      if (success)
      {
         LogTools.info("Initializing RealSense playback from BAG file: {}", bagFileName);
         realsense.enableColor(realsenseConfiguration);
         realsense.initialize();

         // Get the playback device from the pipeline profile after initialization
         playbackDevice = realsense2.rs2_pipeline_profile_get_device(realsense.pipelineProfile, realsense.getError());
         realsense.checkError("Failed to get playback device from pipeline profile");

         grabFailureCount = 0;
      }
      else
      {
         LogTools.error("Failed to initialize RealSense playback");
      }

      return success;
   }

   @Override
   public boolean isSensorRunning()
   {
      return realsense != null && playbackDevice != null && grabFailureCount < realsenseConfiguration.getDepthFPS();
   }

   public void play()
   {
      run(true);
      if (playbackDevice != null)
      {
         realsense2.rs2_playback_device_resume(playbackDevice, realsense.getError());
         realsense.checkError("Failed to resume playback");
      }
   }

   public void pause()
   {
      run(false);
      if (playbackDevice != null)
      {
         realsense2.rs2_playback_device_pause(playbackDevice, realsense.getError());
         realsense.checkError("Failed to pause playback");
      }
   }

   public long getDuration()
   {
      if (playbackDevice != null)
      {
         long duration = realsense2.rs2_playback_get_duration(playbackDevice, realsense.getError());
         realsense.checkError("Failed to get playback duration");
         return duration;
      }
      return 0;
   }

   public long getCurrentPosition()
   {
      if (playbackDevice != null)
      {
         long position = realsense2.rs2_playback_get_position(playbackDevice, realsense.getError());
         realsense.checkError("Failed to get playback position");
         return position;
      }
      return 0;
   }

   public void setCurrentPosition(long timeNanoseconds)
   {
      if (playbackDevice != null)
      {
         realsense2.rs2_playback_seek(playbackDevice, timeNanoseconds, realsense.getError());
         realsense.checkError("Failed to seek playback position");
      }
   }

   public String getBagFileName()
   {
      return bagFileName;
   }
}
