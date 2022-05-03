package us.ihmc.perception.realsense;

import us.ihmc.perception.MutableBytePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.librealsense2.*;
import org.bytedeco.librealsense2.global.realsense2;
import us.ihmc.log.LogTools;
import us.ihmc.tools.string.StringTools;

import java.util.function.Supplier;

import static org.bytedeco.librealsense2.global.realsense2.rs2_release_frame;

public class BytedecoRealsenseL515
{
   private static final int RS2_FRAME_POINTER_SIZE = Pointer.sizeof(rs2_frame.class);

   protected final int width = 1024;
   protected final int height = 768;
   protected final int fps = 30;
   protected final int DEPTH_STREAM_INDEX = -1;
   protected final int COLOR_STREAM_INDEX = -1;

   protected final rs2_device device; // The device (a device contains sensors like cameras and IMUS)
   private final String serialNumber;
   protected final rs2_pipeline pipeline; // Declare RealSense pipeline, encapsulating the actual device and sensors
   protected final rs2_config config; // Create a configuration for configuring the pipeline with a non default profile
   protected rs2_sensor sensor; // The depth sensor
   protected final rs2_error error = new rs2_error(); // error pointer, have to check it after every call
   protected rs2_pipeline_profile pipelineProfile;
   protected rs2_intrinsics depthStreamIntrinsics = new rs2_intrinsics();
   protected rs2_intrinsics colorStreamIntrinsics = new rs2_intrinsics();
   protected rs2_stream_profile depthFrameStreamProfile;
   protected rs2_stream_profile colorFrameStreamProfile;
   protected double depthToMeterConversion;
   protected rs2_frame syncedFrames = new rs2_frame();
   protected mutable_rs2_frame depthFrame = new mutable_rs2_frame();
   protected mutable_rs2_frame colorFrame = new mutable_rs2_frame();
   protected MutableBytePointer depthFrameData = null;
   protected MutableBytePointer colorFrameData = null;
   private int depthFrameDataSize;
   private int colorFrameDataSize;
   private rs2_processing_block colorAlignProcessingBlock;
   private rs2_frame_queue colorFrameQueue;
   private long depthFrameDataAddress;
   private long colorFrameDataAddress;
   private boolean colorEnabled = false;
   private long colorFrameAddress;

   public BytedecoRealsenseL515(rs2_context context, rs2_device device, String serialNumber)
   {
      this.device = device;
      this.serialNumber = serialNumber;
      pipeline = realsense2.rs2_create_pipeline(context, error);
      config = realsense2.rs2_create_config(error);

      realsense2.rs2_config_enable_stream(config, realsense2.RS2_STREAM_DEPTH, DEPTH_STREAM_INDEX, width, height, realsense2.RS2_FORMAT_Z16, fps, error);
      checkError(true, "Failed to enable stream.");

      rs2_sensor_list sensorList = realsense2.rs2_query_sensors(device, error);
      sensor = realsense2.rs2_create_sensor(sensorList, 0, error);

      depthToMeterConversion = realsense2.rs2_get_depth_scale(sensor, error);

      LogTools.info("Configured Depth Stream of L515 Device. Serial number: {}", serialNumber);
   }

   public void enableColor(int width, int height, int fps)
   {
      realsense2.rs2_config_enable_stream(config, realsense2.RS2_STREAM_COLOR, COLOR_STREAM_INDEX, width, height, realsense2.RS2_FORMAT_RGB8, fps, error);
      checkError(true, "Failed to enable stream.");

      colorAlignProcessingBlock = realsense2.rs2_create_align(realsense2.RS2_STREAM_COLOR, error);
      checkError(true, "");

      colorFrameQueue = realsense2.rs2_create_frame_queue(1, error);
      checkError(true, "");
   }

   /**
    * Call this method if you plan on trigger the laser from an external signal See
    * https://dev.intelrealsense.com/docs/lidar-camera-l515-multi-camera-setup for more information
    */
   public void enableInterCamSyncMode()
   {
      rs2_options options = new rs2_options(sensor);
      realsense2.rs2_set_option(options, realsense2.RS2_OPTION_INTER_CAM_SYNC_MODE, 1.0f, error);
      checkError(true, "Failed to set sync mode.");
   }

   public void initialize()
   {
      pipelineProfile = realsense2.rs2_pipeline_start_with_config(pipeline, config, error);
      checkError(true, "Error starting pipeline.");

      colorEnabled = colorAlignProcessingBlock != null;
      if (colorEnabled)
      {
         realsense2.rs2_start_processing_queue(colorAlignProcessingBlock, colorFrameQueue, error);
         checkError(true, "");
      }
   }

   public boolean readFrameData()
   {
      boolean dataWasRead = false;
      boolean frameAvailable = realsense2.rs2_pipeline_poll_for_frames(pipeline, syncedFrames, error) == 1;
      checkError(false, "");

      if (frameAvailable)
      {
//         depthFrame.address(syncedFrames.address());
//         rs2_frame pointer = syncedFrames.getPointer(1);
//         long address = pointer.address();
//         System.out.println(address);
//         colorFrame.address(syncedFrames.address() + RS2_FRAME_POINTER_SIZE);
//
         rs2_frame extractedColorFrame = null;
         if (colorEnabled)
         {
            extractedColorFrame = realsense2.rs2_extract_frame(syncedFrames, 1, error);
         }

//
//         BytePointer bytePointer = new BytePointer(pointer);
//         long mightBeAddress = bytePointer.getLong();
//         System.out.println(mightBeAddress);
//         new Pointer
//         BytePointer bytePointer = new BytePointer(pointer);
//         long mightBeAddress = bytePointer.getLong();
//         System.out.println(mightBeAddress);
         //         syncedFrames.

//         new rs2_frame
//         checkError(false, "");
//
//
//         int numberOfFrames = realsense2.rs2_embedded_frames_count(syncedFrames, error);
//         checkError(false, "");

//         System.out.println(numberOfFrames);
//         colorFrameAddress = colorFrame.address();

         //         depthFrame.set(syncedFrames.address(), 0, rs2_frame.totalBytes(), rs2_frame.totalBytes());

//            depthFrame = realsense2.rs2_extract_frame(syncedFrames, 0, error);
//            checkError(true, "");



//            if (colorEnabled)
//            {
////               colorFrame = realsense2.rs2_extract_frame(syncedFrames, 1, error);
//               checkError(true, "");
//            }
//         realsense2.rs2_extract_frame()

         depthFrameDataSize = realsense2.rs2_get_frame_data_size(syncedFrames, error);
         checkError(false, "");

         if (colorEnabled)
         {
            colorFrameDataSize = realsense2.rs2_get_frame_data_size(extractedColorFrame, error);
            checkError(false, "");
         }

         if (depthFrameDataSize > 0)
         {
            if (depthFrameData == null)
            {
               depthFrameData = new MutableBytePointer(realsense2.rs2_get_frame_data(syncedFrames, error));
            }
            else
            {
               depthFrameDataAddress = realsense2.rs2_get_frame_data_address(syncedFrames, error);
            }

            if (depthFrameStreamProfile == null)
            {
               depthFrameStreamProfile = realsense2.rs2_get_frame_stream_profile(syncedFrames, error);
               realsense2.rs2_get_video_stream_intrinsics(depthFrameStreamProfile, depthStreamIntrinsics, error);
            }

            if (colorEnabled)
            {
               if (colorFrameData == null)
               {
                  colorFrameData = new MutableBytePointer(realsense2.rs2_get_frame_data(extractedColorFrame, error));
               }
               else
               {
                  colorFrameDataAddress = realsense2.rs2_get_frame_data_address(extractedColorFrame, error);
               }

               if (colorFrameStreamProfile == null)
               {
                  colorFrameStreamProfile = realsense2.rs2_get_frame_stream_profile(extractedColorFrame, error);
                  realsense2.rs2_get_video_stream_intrinsics(colorFrameStreamProfile, colorStreamIntrinsics, error);
               }
            }

            dataWasRead = true;
         }

//         rs2_release_frame(depthFrame);
         rs2_release_frame(syncedFrames);

         if (colorEnabled)
         {
            rs2_release_frame(extractedColorFrame);
         }
      }

      return dataWasRead;
   }

   public void updateDataBytePointers()
   {
      if (depthFrameDataAddress > 0)
         depthFrameData.setAddress(depthFrameDataAddress);

      if (colorEnabled)
      {
         if (colorFrameDataAddress > 0)
            colorFrameData.setAddress(colorFrameDataAddress);
      }
   }

   public void setLaserPower(float laserPower)
   {
      rs2_options options = new rs2_options(sensor);
      realsense2.rs2_set_option(options, realsense2.RS2_OPTION_LASER_POWER, laserPower, error);
      checkError(true, "Failed to set laser power.");
   }

   public void setDigitalGail(int digitalGain)
   {
      rs2_options options = new rs2_options(sensor);
      realsense2.rs2_set_option(options, realsense2.RS2_OPTION_DIGITAL_GAIN, digitalGain, error);
      checkError(true, "");
   }

   public void deleteDevice()
   {
      realsense2.rs2_pipeline_stop(pipeline, error);
      checkError(false, "Error stopping pipeline.");

      realsense2.rs2_delete_pipeline_profile(pipelineProfile);
      realsense2.rs2_delete_config(config);
      realsense2.rs2_delete_pipeline(pipeline);
      realsense2.rs2_delete_device(device);
   }

   private void checkError(boolean throwRuntimeException, String extraMessage)
   {
      if (!error.isNull())
      {
         Supplier<String> errorMessage = StringTools.format("{} {}({}): {}",
                                                            extraMessage,
                                                            realsense2.rs2_get_failed_function(error).getString(),
                                                            realsense2.rs2_get_failed_args(error).getString(),
                                                            realsense2.rs2_get_error_message(error).getString());
         LogTools.error(errorMessage);
         if (throwRuntimeException)
         {
            throw new RuntimeException(errorMessage.get());
         }
      }
   }

   private boolean getColorEnabled()
   {
      return colorEnabled;
   }

   public MutableBytePointer getDepthFrameData()
   {
      return depthFrameData;
   }

   public int getDepthFrameDataSize()
   {
      return depthFrameDataSize;
   }

   public double getDepthToMeterConversion()
   {
      return depthToMeterConversion;
   }

   public rs2_intrinsics getIntrinsicParameters()
   {
      return depthStreamIntrinsics;
   }

   public double getFocalLengthPixelsX()
   {
      return depthStreamIntrinsics.fx();
   }

   public double getFocalLengthPixelsY()
   {
      return depthStreamIntrinsics.fy();
   }

   public double getPrincipalOffsetXPixels()
   {
      return depthStreamIntrinsics.ppx();
   }

   public double getPrincipalOffsetYPixels()
   {
      return depthStreamIntrinsics.ppy();
   }

   public int getDepthWidth()
   {
      return width;
   }

   public int getDepthHeight()
   {
      return height;
   }

   public rs2_device getDevice()
   {
      return device;
   }
}
