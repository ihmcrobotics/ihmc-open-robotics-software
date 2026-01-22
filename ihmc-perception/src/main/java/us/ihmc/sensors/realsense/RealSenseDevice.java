package us.ihmc.sensors.realsense;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.librealsense2.global.realsense2;
import org.bytedeco.librealsense2.rs2_config;
import org.bytedeco.librealsense2.rs2_context;
import org.bytedeco.librealsense2.rs2_device;
import org.bytedeco.librealsense2.rs2_error;
import org.bytedeco.librealsense2.rs2_extrinsics;
import org.bytedeco.librealsense2.rs2_frame;
import org.bytedeco.librealsense2.rs2_frame_queue;
import org.bytedeco.librealsense2.rs2_intrinsics;
import org.bytedeco.librealsense2.rs2_options;
import org.bytedeco.librealsense2.rs2_pipeline;
import org.bytedeco.librealsense2.rs2_pipeline_profile;
import org.bytedeco.librealsense2.rs2_processing_block;
import org.bytedeco.librealsense2.rs2_sensor;
import org.bytedeco.librealsense2.rs2_sensor_list;
import org.bytedeco.librealsense2.rs2_stream_profile;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.tools.string.StringTools;

import java.util.function.Supplier;

import static org.bytedeco.librealsense2.global.realsense2.RS2_CAMERA_INFO_SERIAL_NUMBER;
import static org.bytedeco.librealsense2.global.realsense2.rs2_release_frame;

public class RealSenseDevice
{
   protected final int depthWidth;
   protected final int depthHeight;
   protected final int fps;
   protected final int DEPTH_STREAM_INDEX = -1;
   protected final int COLOR_STREAM_INDEX = -1;

   protected final rs2_device device; // The device (a device contains sensors like cameras and IMUS)
   protected final rs2_pipeline pipeline; // Declare RealSense pipeline, encapsulating the actual device and sensors
   protected final rs2_config config; // Create a configuration for configuring the pipeline with a non default profile
   protected rs2_sensor depthSensor; // The depth sensor
   protected final rs2_error error = new rs2_error(); // error pointer, have to check it after every call
   protected final boolean isPlayback; // Whether this is a playback device from a BAG file
   protected rs2_pipeline_profile pipelineProfile;
   protected rs2_intrinsics depthStreamIntrinsics = new rs2_intrinsics();
   protected rs2_intrinsics colorStreamIntrinsics = new rs2_intrinsics();
   protected rs2_extrinsics depthToColorExtrinsics = new rs2_extrinsics();
   // Realsense is X right, Y down, Z forward
   // IHMC is X forward, Y left, Z up
   private final RigidBodyTransform realsenseToIHMCZUpTransform = new RigidBodyTransform();
   {
      realsenseToIHMCZUpTransform.getRotation().setYawPitchRoll(Math.toRadians(-90.0), 0.0, Math.toRadians(-90.0));
   }
   private final ReferenceFrame realsenseFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("Realsense",
                                                                                                                   ReferenceFrame.getWorldFrame(),
                                                                                                                   realsenseToIHMCZUpTransform);
   protected final FrameVector3D depthToColorTranslation = new FrameVector3D(realsenseFrame);
   protected final Quaternion depthToColorQuaternion = new Quaternion();
   protected rs2_stream_profile depthFrameStreamProfile;
   protected rs2_stream_profile colorFrameStreamProfile;
   protected double depthDiscretization;
   protected rs2_frame syncedFrames = new rs2_frame();
   protected MutableBytePointer depthFrameData = new MutableBytePointer();
   protected MutableBytePointer colorFrameData = new MutableBytePointer();
   private int depthFrameDataSize;
   private int colorFrameDataSize;
   private rs2_processing_block colorAlignProcessingBlock;
   private rs2_frame_queue colorFrameQueue;
   private long depthFrameDataAddress;
   private long colorFrameDataAddress;
   private boolean colorEnabled = false;
   private int colorWidth;
   private int colorHeight;
   private CameraIntrinsics depthCameraIntrinsics;
   private CameraIntrinsics colorCameraIntrinsics;

   public RealSenseDevice(rs2_context context, rs2_device device, int depthWidth, int depthHeight, int fps)
   {
      this(context, device, depthWidth, depthHeight, fps, null);
   }

   public RealSenseDevice(rs2_context context, rs2_device device, int depthWidth, int depthHeight, int fps, String bagFilePath)
   {
      this.device = device;
      this.depthWidth = depthWidth;
      this.depthHeight = depthHeight;
      this.fps = fps;
      this.isPlayback = bagFilePath != null;

      pipeline = realsense2.rs2_create_pipeline(context, error);
      checkError("Failed to create pipeline.");
      config = realsense2.rs2_create_config(error);
      checkError("Failed to create config.");

      if (isPlayback)
      {
         // For playback from BAG file
         realsense2.rs2_config_enable_device_from_file(config, bagFilePath, error);
         checkError("Failed to enable device from file.");
         LogTools.info("Configured playback from bag file: {}", bagFilePath);
         return; // Skip device-specific configuration for playback
      }

      // For live device
      realsense2.rs2_config_enable_stream(config, realsense2.RS2_STREAM_DEPTH, DEPTH_STREAM_INDEX, depthWidth, depthHeight, realsense2.RS2_FORMAT_Z16, fps, error);
      checkError("Failed to enable stream.");

      BytePointer serialNumber = realsense2.rs2_get_device_info(device, RS2_CAMERA_INFO_SERIAL_NUMBER, error);
      checkError("Failed to get device serial number.");

      realsense2.rs2_config_enable_device(config, serialNumber, error);
      checkError("Failed to enable device.");

      rs2_sensor_list sensorList = realsense2.rs2_query_sensors(device, error);
      checkError("Failed to query sensors.");
      int numberOfSensors = realsense2.rs2_get_sensors_count(sensorList, error);
      checkError("Failed to get sensors count.");
      LogTools.info("{} Realsense sensors detected.", numberOfSensors);

      for (int i = 0; i < numberOfSensors; i++)
      {
         rs2_sensor sensor = realsense2.rs2_create_sensor(sensorList, i, error);
         checkError("Failed to create sensor.");

         int sensorType = -1;
         if (checkSensorType(sensor, realsense2.RS2_EXTENSION_DEPTH_SENSOR))
            sensorType = realsense2.RS2_EXTENSION_DEPTH_SENSOR;
         if (checkSensorType(sensor, realsense2.RS2_EXTENSION_DEPTH_STEREO_SENSOR))
            sensorType = realsense2.RS2_EXTENSION_DEPTH_STEREO_SENSOR;
         if (checkSensorType(sensor, realsense2.RS2_EXTENSION_SOFTWARE_SENSOR))
            sensorType = realsense2.RS2_EXTENSION_SOFTWARE_SENSOR;
         if (checkSensorType(sensor, realsense2.RS2_EXTENSION_POSE_SENSOR))
            sensorType = realsense2.RS2_EXTENSION_POSE_SENSOR;
         if (checkSensorType(sensor, realsense2.RS2_EXTENSION_L500_DEPTH_SENSOR))
            sensorType = realsense2.RS2_EXTENSION_L500_DEPTH_SENSOR;
         if (checkSensorType(sensor, realsense2.RS2_EXTENSION_TM2_SENSOR))
            sensorType = realsense2.RS2_EXTENSION_TM2_SENSOR;
         if (checkSensorType(sensor, realsense2.RS2_EXTENSION_COLOR_SENSOR))
            sensorType = realsense2.RS2_EXTENSION_COLOR_SENSOR;
         if (checkSensorType(sensor, realsense2.RS2_EXTENSION_MOTION_SENSOR))
            sensorType = realsense2.RS2_EXTENSION_MOTION_SENSOR;
         if (checkSensorType(sensor, realsense2.RS2_EXTENSION_FISHEYE_SENSOR))
            sensorType = realsense2.RS2_EXTENSION_FISHEYE_SENSOR;
         if (checkSensorType(sensor, realsense2.RS2_EXTENSION_CALIBRATED_SENSOR))
            sensorType = realsense2.RS2_EXTENSION_CALIBRATED_SENSOR;
         if (checkSensorType(sensor, realsense2.RS2_EXTENSION_MAX_USABLE_RANGE_SENSOR))
            sensorType = realsense2.RS2_EXTENSION_MAX_USABLE_RANGE_SENSOR;
         if (checkSensorType(sensor, realsense2.RS2_EXTENSION_DEBUG_STREAM_SENSOR))
            sensorType = realsense2.RS2_EXTENSION_DEBUG_STREAM_SENSOR;

         String sensorTypeName = switch (sensorType)
               {
                  case realsense2.RS2_EXTENSION_DEPTH_SENSOR -> "Depth";
                  case realsense2.RS2_EXTENSION_DEPTH_STEREO_SENSOR -> "Depth Stereo";
                  case realsense2.RS2_EXTENSION_SOFTWARE_SENSOR -> "Software";
                  case realsense2.RS2_EXTENSION_POSE_SENSOR -> "Pose";
                  case realsense2.RS2_EXTENSION_L500_DEPTH_SENSOR -> "L500 Depth";
                  case realsense2.RS2_EXTENSION_TM2_SENSOR -> "TM2";
                  case realsense2.RS2_EXTENSION_COLOR_SENSOR -> "Color";
                  case realsense2.RS2_EXTENSION_MOTION_SENSOR -> "Motion";
                  case realsense2.RS2_EXTENSION_FISHEYE_SENSOR -> "Fisheye";
                  case realsense2.RS2_EXTENSION_CALIBRATED_SENSOR -> "Calibrated";
                  case realsense2.RS2_EXTENSION_MAX_USABLE_RANGE_SENSOR -> "Max Usable Range";
                  case realsense2.RS2_EXTENSION_DEBUG_STREAM_SENSOR -> "Debug Stream";
                  default -> "Unknown";
               };

         BytePointer friendlyNameBytePointer = realsense2.rs2_get_sensor_info(sensor, realsense2.RS2_CAMERA_INFO_NAME, error);
         LogTools.info("Sensor {}: {}: {}", i, friendlyNameBytePointer.getString(), sensorTypeName);

         realsense2.rs2_delete_sensor(sensor);
      }

      // We assume the first sensor is the depth stream. There are multiple kinds above,
      // so I didn't know a way to make this any better without testing all kinds of Realsenses.
      // Just keep in mind, maybe the first sensor is not always the depth sensor.
      depthSensor = realsense2.rs2_create_sensor(sensorList, 0, error);
      checkError("Failed to create sensor.");

      realsense2.rs2_delete_sensor_list(sensorList);

      if (serialNumber != null && !serialNumber.isNull())
      {
         LogTools.info("Configured Depth Stream of Realsense Device. Serial number: {}", serialNumber.getString());
         serialNumber.close();
      }
   }

   private boolean checkSensorType(rs2_sensor sensor, int sensorType)
   {
      int isSensorOfType = realsense2.rs2_is_sensor_extendable_to(sensor, sensorType, error);
      checkError("Failed to check sensor extendable to " + sensorType);
      return isSensorOfType == 1;
   }

   public void enableColor(RealSenseConfiguration realsenseConfiguration)
   {
      enableColor(realsenseConfiguration.getColorWidth(), realsenseConfiguration.getColorHeight(), realsenseConfiguration.getColorFPS());
   }

   public void enableColor(int colorWidth, int colorHeight, int fps)
   {
      this.colorWidth = colorWidth;
      this.colorHeight = colorHeight;

      if (!isPlayback)
      {
         realsense2.rs2_config_enable_stream(config, realsense2.RS2_STREAM_COLOR, COLOR_STREAM_INDEX, colorWidth, colorHeight, realsense2.RS2_FORMAT_BGR8, fps, error);
         checkError("Failed to enable stream.");
      }

      colorAlignProcessingBlock = realsense2.rs2_create_align(realsense2.RS2_STREAM_COLOR, error);
      checkError("");

      colorFrameQueue = realsense2.rs2_create_frame_queue(1, error);
      checkError("");
   }

   /**
    * Call this method if you plan on trigger the laser from an external signal See
    * https://dev.intelrealsense.com/docs/lidar-camera-l515-multi-camera-setup for more information
    */
   public void enableInterCamSyncMode()
   {
      rs2_options options = new rs2_options(depthSensor);
      realsense2.rs2_set_option(options, realsense2.RS2_OPTION_INTER_CAM_SYNC_MODE, 1.0f, error);
      checkError("Failed to set sync mode.");
   }

   public void initialize()
   {
      pipelineProfile = realsense2.rs2_pipeline_start_with_config(pipeline, config, error);
      checkError("Error starting pipeline.");

      // For playback devices, we need to get the device and depth sensor from the pipeline profile
      if (isPlayback && depthSensor == null)
      {
         rs2_device playbackDevice = realsense2.rs2_pipeline_profile_get_device(pipelineProfile, error);
         checkError("Failed to get device from pipeline profile.");

         rs2_sensor_list sensorList = realsense2.rs2_query_sensors(playbackDevice, error);
         checkError("Failed to query sensors.");

         // Get the first sensor as depth sensor (similar to live device)
         depthSensor = realsense2.rs2_create_sensor(sensorList, 0, error);
         checkError("Failed to create sensor.");

         realsense2.rs2_delete_sensor_list(sensorList);
         LogTools.info("Retrieved depth sensor from playback device");
      }

      colorEnabled = colorAlignProcessingBlock != null;
      if (colorEnabled)
      {
         realsense2.rs2_start_processing_queue(colorAlignProcessingBlock, colorFrameQueue, error);
         checkError("");
      }

      LogTools.info("Started processing queue");
   }

   public boolean readFrameData()
   {
      boolean dataWasRead = false;
      boolean frameAvailable = realsense2.rs2_pipeline_poll_for_frames(pipeline, syncedFrames, error) == 1;
      checkError("");

      if (frameAvailable)
      {
         rs2_frame alignedFrames = syncedFrames;
         rs2_frame extractedDepthFrame = null;
         rs2_frame extractedColorFrame = null;

         if (colorEnabled)
         {
            // Process frames through alignment block
            realsense2.rs2_process_frame(colorAlignProcessingBlock, syncedFrames, error);
            if (!error.isNull())
            {
               checkError("Failed to process frame for alignment");
               rs2_release_frame(syncedFrames);
               return false;
            }

            // Poll for aligned frameset (non-blocking)
            rs2_frame tempAlignedFrame = new rs2_frame();
            int pollResult = realsense2.rs2_poll_for_frame(colorFrameQueue, tempAlignedFrame, error);
            if (pollResult == 0 || !error.isNull() || tempAlignedFrame.isNull())
            {
               // No aligned frame available yet, skip this frame
               rs2_release_frame(syncedFrames);
               return false;
            }
            alignedFrames = tempAlignedFrame;

            // Extract depth and color from aligned frameset
            extractedDepthFrame = realsense2.rs2_extract_frame(alignedFrames, 0, error);
            extractedColorFrame = realsense2.rs2_extract_frame(alignedFrames, 1, error);

            if (extractedDepthFrame == null || extractedDepthFrame.isNull() || extractedColorFrame == null || extractedColorFrame.isNull())
            {
               // Failed to extract frames, cleanup and skip
               if (extractedDepthFrame != null && !extractedDepthFrame.isNull())
                  rs2_release_frame(extractedDepthFrame);
               if (extractedColorFrame != null && !extractedColorFrame.isNull())
                  rs2_release_frame(extractedColorFrame);
               if (alignedFrames != syncedFrames)
                  rs2_release_frame(alignedFrames);
               rs2_release_frame(syncedFrames);
               return false;
            }

            depthFrameDataSize = realsense2.rs2_get_frame_data_size(extractedDepthFrame, error);
            if (!error.isNull() || depthFrameDataSize <= 0)
            {
               rs2_release_frame(extractedDepthFrame);
               rs2_release_frame(extractedColorFrame);
               if (alignedFrames != syncedFrames)
                  rs2_release_frame(alignedFrames);
               rs2_release_frame(syncedFrames);
               return false;
            }

            colorFrameDataSize = realsense2.rs2_get_frame_data_size(extractedColorFrame, error);
            if (!error.isNull() || colorFrameDataSize <= 0)
            {
               rs2_release_frame(extractedDepthFrame);
               rs2_release_frame(extractedColorFrame);
               if (alignedFrames != syncedFrames)
                  rs2_release_frame(alignedFrames);
               rs2_release_frame(syncedFrames);
               return false;
            }
         }
         else
         {
            depthFrameDataSize = realsense2.rs2_get_frame_data_size(syncedFrames, error);
            checkError("");
         }

         if (depthFrameDataSize > 0)
         {
            rs2_frame depthFrameToUse = colorEnabled ? extractedDepthFrame : syncedFrames;
            depthFrameDataAddress = realsense2.rs2_get_frame_data_address(depthFrameToUse, error);

            if (depthFrameStreamProfile == null)
            {
               depthFrameStreamProfile = realsense2.rs2_get_frame_stream_profile(depthFrameToUse, error);
               realsense2.rs2_get_video_stream_intrinsics(depthFrameStreamProfile, depthStreamIntrinsics, error);
               checkError("Failed to get depth stream intrinsics.");

               LogTools.info("Depth intrinsics: {}", String.format("Depth: fx: %.4f, fy: %.4f, cx: %.4f, cy: %.4f, h: %d, w: %d",
                                                                   depthStreamIntrinsics.fx(),
                                                                   depthStreamIntrinsics.fy(),
                                                                   depthStreamIntrinsics.ppx(),
                                                                   depthStreamIntrinsics.ppy(),
                                                                   depthHeight,
                                                                   depthWidth));
               depthDiscretization = realsense2.rs2_get_depth_scale(depthSensor, error);
               LogTools.info("Depth discretization: {} (meters/unit)", depthDiscretization);
            }

            if (colorEnabled)
            {
               colorFrameDataAddress = realsense2.rs2_get_frame_data_address(extractedColorFrame, error);

               if (colorFrameStreamProfile == null)
               {
                  colorFrameStreamProfile = realsense2.rs2_get_frame_stream_profile(extractedColorFrame, error);
                  realsense2.rs2_get_video_stream_intrinsics(colorFrameStreamProfile, colorStreamIntrinsics, error);
                  checkError("Failed to get color stream intrinsics.");

                  LogTools.info("Color intrinsics: {}", String.format("Color: fx: %.4f, fy: %.4f, cx: %.4f, cy: %.4f, h: %d, w: %d",
                                                                      colorStreamIntrinsics.fx(),
                                                                      colorStreamIntrinsics.fy(),
                                                                      colorStreamIntrinsics.ppx(),
                                                                      colorStreamIntrinsics.ppy(),
                                                                      colorHeight,
                                                                      colorWidth));

                  realsense2.rs2_get_extrinsics(depthFrameStreamProfile, colorFrameStreamProfile, depthToColorExtrinsics, error);
                  FloatPointer translation = depthToColorExtrinsics.translation();
                  FloatPointer rotation = depthToColorExtrinsics.rotation();

                  float realsenseXRight = translation.get(0);
                  float realsenseYDown = translation.get(1);
                  float realsenseZForward = translation.get(2);
                  depthToColorTranslation.set(realsenseXRight, realsenseYDown, realsenseZForward);
                  RotationMatrix depthToColorRotationMatrix = new RotationMatrix();
                  depthToColorRotationMatrix.setAndNormalize(rotation.get(0), // Have to convert column major to row major
                                                             rotation.get(3),
                                                             rotation.get(6),
                                                             rotation.get(1),
                                                             rotation.get(4),
                                                             rotation.get(7),
                                                             rotation.get(2),
                                                             rotation.get(5),
                                                             rotation.get(8));

                  FramePose3D depthSensorPose = new FramePose3D(realsenseFrame);
                  FramePose3D colorSensorPose = new FramePose3D(realsenseFrame);
                  // Because we've got depth to color, let's do inverse transform to go color to depth
                  RigidBodyTransform depthToColorTransform = new RigidBodyTransform(depthToColorRotationMatrix, depthToColorTranslation);
                  depthToColorTransform.inverseTransform(colorSensorPose);

                  depthSensorPose.changeFrame(ReferenceFrame.getWorldFrame());
                  colorSensorPose.changeFrame(ReferenceFrame.getWorldFrame());

                  // The rotation part is actually wrong at this point; we gotta get the rotationDepthToColorIHMCFrame
                  // from the depth sensor
                  depthToColorQuaternion.difference(depthSensorPose.getRotation(), colorSensorPose.getRotation());

                  depthToColorTranslation.changeFrame(ReferenceFrame.getWorldFrame());
                  depthToColorRotationMatrix.set(depthToColorQuaternion);

                  LogTools.info("Depth to color extrinsics (ZUp frame):\n   Translation: {}\n   Rotation: {}",
                                depthToColorTranslation, new YawPitchRoll(depthToColorQuaternion));
               }
            }

            dataWasRead = true;
         }

         // Release frames
         if (colorEnabled)
         {
            if (extractedDepthFrame != null)
               rs2_release_frame(extractedDepthFrame);
            if (extractedColorFrame != null)
               rs2_release_frame(extractedColorFrame);
            if (alignedFrames != null && alignedFrames != syncedFrames)
               rs2_release_frame(alignedFrames);
         }

         rs2_release_frame(syncedFrames);
      }

      return dataWasRead;
   }

   public void updateDataBytePointers()
   {
      if (depthFrameDataAddress > 0)
      {
         depthFrameData.deallocate();
         depthFrameData.setAddress(depthFrameDataAddress);
      }

      if (colorEnabled)
      {
         if (colorFrameDataAddress > 0)
         {
            colorFrameData.deallocate();
            colorFrameData.setAddress(colorFrameDataAddress);
         }
      }
   }

   public void setLaserPower(float laserPower)
   {
      rs2_options options = new rs2_options(depthSensor);
      realsense2.rs2_set_option(options, realsense2.RS2_OPTION_LASER_POWER, laserPower, error);
      checkError("Failed to set laser power.");
   }

   public void setDigitalGain(int digitalGain)
   {
      rs2_options options = new rs2_options(depthSensor);
      realsense2.rs2_set_option(options, realsense2.RS2_OPTION_DIGITAL_GAIN, digitalGain, error);
      checkError( "Failed to set digital gain");
   }

   public void deleteDevice()
   {
      // LogTools/log4j2 is no longer operational during JVM shutdown
      System.out.println("Stopping pipeline...");
      realsense2.rs2_pipeline_stop(pipeline, error);
      checkError("Error stopping pipeline.");

      // Give threads time to finish processing
      try
      {
         Thread.sleep(300);
      }
      catch (InterruptedException e)
      {
         Thread.currentThread().interrupt();
      }

      System.out.println("Deleting pipeline profile...");
      if (pipelineProfile != null)
         realsense2.rs2_delete_pipeline_profile(pipelineProfile);

      // Delete processing block before frame queue
      if (colorAlignProcessingBlock != null)
      {
         System.out.println("Deleting color align processing block...");
         realsense2.rs2_delete_processing_block(colorAlignProcessingBlock);
      }

      if (colorFrameQueue != null)
      {
         System.out.println("Deleting color frame queue...");
         realsense2.rs2_delete_frame_queue(colorFrameQueue);
      }

      // Calling delete on these may cause a native crash - it's not required anyway
      // https://github.com/IntelRealSense/librealsense/issues/4651#issuecomment-522295675
      // if (depthFrameStreamProfile != null)
      // {
      //    System.out.println("Deleting depth stream profile...");
      //    realsense2.rs2_delete_stream_profile(depthFrameStreamProfile);
      // }
      // if (colorFrameStreamProfile != null)
      // {
      //    System.out.println("Deleting color stream profile...");
      //    realsense2.rs2_delete_stream_profile(colorFrameStreamProfile);
      // }

      if (depthSensor != null)
      {
         System.out.println("Deleting sensor...");
         realsense2.rs2_delete_sensor(depthSensor);
      }
      System.out.println("Deleting config...");
      realsense2.rs2_delete_config(config);
      System.out.println("Deleting pipeline...");
      realsense2.rs2_delete_pipeline(pipeline);
      if (device != null)
      {
         System.out.println("Deleting device...");
         realsense2.rs2_delete_device(device);
      }
   }

   public void checkError(String extraMessage)
   {
      if (!error.isNull())
      {
         Supplier<String> errorMessage = StringTools.format("{} {}({}): {}",
                                                            extraMessage,
                                                            realsense2.rs2_get_failed_function(error).getString(),
                                                            realsense2.rs2_get_failed_args(error).getString(),
                                                            realsense2.rs2_get_error_message(error).getString());
         LogTools.error(errorMessage);
      }
   }

   public rs2_error getError()
   {
      return error;
   }

   private boolean getColorEnabled()
   {
      return colorEnabled;
   }

   public MutableBytePointer getDepthFrameData()
   {
      return depthFrameData;
   }

   public MutableBytePointer getColorFrameData()
   {
      return colorFrameData;
   }

   public int getDepthFrameDataSize()
   {
      return depthFrameDataSize;
   }

   public double getDepthDiscretization()
   {
      return depthDiscretization;
   }

   public rs2_intrinsics getDepthIntrinsicParameters()
   {
      return depthStreamIntrinsics;
   }

   public double getDepthFocalLengthPixelsX()
   {
      return depthStreamIntrinsics.fx();
   }

   public double getDepthFocalLengthPixelsY()
   {
      return depthStreamIntrinsics.fy();
   }

   public double getDepthPrincipalOffsetXPixels()
   {
      return depthStreamIntrinsics.ppx();
   }

   public double getDepthPrincipalOffsetYPixels()
   {
      return depthStreamIntrinsics.ppy();
   }

   public rs2_intrinsics getColorIntrinsicParameters()
   {
      return colorStreamIntrinsics;
   }

   public double getColorFocalLengthPixelsX()
   {
      return colorStreamIntrinsics.fx();
   }

   public double getColorFocalLengthPixelsY()
   {
      return colorStreamIntrinsics.fy();
   }

   public double getColorPrincipalOffsetXPixels()
   {
      return colorStreamIntrinsics.ppx();
   }

   public double getColorPrincipalOffsetYPixels()
   {
      return colorStreamIntrinsics.ppy();
   }

   public CameraIntrinsics getDepthCameraIntrinsics()
   {
      if (depthCameraIntrinsics == null)
      {
         depthCameraIntrinsics = new CameraIntrinsics();
         depthCameraIntrinsics.setFx(getDepthFocalLengthPixelsX());
         depthCameraIntrinsics.setFy(getDepthFocalLengthPixelsY());
         depthCameraIntrinsics.setCx(getDepthPrincipalOffsetXPixels());
         depthCameraIntrinsics.setCy(getDepthPrincipalOffsetYPixels());
         depthCameraIntrinsics.setHeight(depthHeight);
         depthCameraIntrinsics.setWidth(depthWidth);
      }
      return depthCameraIntrinsics;
   }

   public CameraIntrinsics getColorCameraIntrinsics()
   {
      if (colorCameraIntrinsics == null)
      {
         colorCameraIntrinsics = new CameraIntrinsics();
         colorCameraIntrinsics.setFx(getColorFocalLengthPixelsX());
         colorCameraIntrinsics.setFy(getColorFocalLengthPixelsY());
         colorCameraIntrinsics.setCx(getColorPrincipalOffsetXPixels());
         colorCameraIntrinsics.setCy(getColorPrincipalOffsetYPixels());
         colorCameraIntrinsics.setHeight(colorHeight);
         colorCameraIntrinsics.setWidth(colorWidth);
      }
      return colorCameraIntrinsics;
   }

   public int getDepthWidth()
   {
      return depthWidth;
   }

   public int getDepthHeight()
   {
      return depthHeight;
   }

   public int getColorWidth()
   {
      return colorWidth;
   }

   public int getColorHeight()
   {
      return colorHeight;
   }

   public rs2_device getDevice()
   {
      return device;
   }

   public Vector3DReadOnly getDepthToColorTranslation()
   {
      return depthToColorTranslation;
   }

   public QuaternionReadOnly getDepthToColorRotation()
   {
      return depthToColorQuaternion;
   }
}
