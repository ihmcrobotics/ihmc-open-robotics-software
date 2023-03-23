package us.ihmc.perception.realsense;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
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
import us.ihmc.perception.MutableBytePointer;
import org.bytedeco.librealsense2.*;
import org.bytedeco.librealsense2.global.realsense2;
import us.ihmc.log.LogTools;
import us.ihmc.tools.string.StringTools;

import java.util.function.Supplier;

import static org.bytedeco.librealsense2.global.realsense2.rs2_release_frame;

public class BytedecoRealsense
{
   protected final int depthWidth;
   protected final int depthHeight;
   protected final int fps;
   protected final int DEPTH_STREAM_INDEX = -1;
   protected final int COLOR_STREAM_INDEX = -1;

   protected final rs2_device device; // The device (a device contains sensors like cameras and IMUS)
   protected final rs2_pipeline pipeline; // Declare RealSense pipeline, encapsulating the actual device and sensors
   protected final rs2_config config; // Create a configuration for configuring the pipeline with a non default profile
   protected rs2_sensor sensor; // The depth sensor
   protected final rs2_error error = new rs2_error(); // error pointer, have to check it after every call
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
   private int colorWidth;
   private int colorHeight;
   private CameraPinholeBrown depthCameraIntrinsics;
   private CameraPinholeBrown colorCameraIntrinsics;

   public BytedecoRealsense(rs2_context context, rs2_device device, String serialNumber, int depthWidth, int depthHeight, int fps)
   {
      this.device = device;
      this.depthWidth = depthWidth;
      this.depthHeight = depthHeight;
      this.fps = fps;
      pipeline = realsense2.rs2_create_pipeline(context, error);
      checkError(true, "Failed to create pipeline.");
      config = realsense2.rs2_create_config(error);
      checkError(true, "Failed to create config.");

      realsense2.rs2_config_enable_stream(config, realsense2.RS2_STREAM_DEPTH, DEPTH_STREAM_INDEX, depthWidth, depthHeight, realsense2.RS2_FORMAT_Z16, fps, error);
      checkError(true, "Failed to enable stream.");

      realsense2.rs2_config_enable_device(config, serialNumber, error);
      checkError(true, "Failed to enable device.");

      rs2_sensor_list sensorList = realsense2.rs2_query_sensors(device, error);
      checkError(true, "Failed to query sensors.");
      int numberOfSensors = realsense2.rs2_get_sensors_count(sensorList, error);
      checkError(true, "Failed to get sensors count.");
      LogTools.info("{} Realsense sensors detected.", numberOfSensors);

      for (int i = 0; i < numberOfSensors; i++)
      {
         rs2_sensor sensor = realsense2.rs2_create_sensor(sensorList, 0, error);
         checkError(true, "Failed to create sensor.");
         BytePointer friendlyNameBytePointer = realsense2.rs2_get_sensor_info(sensor, realsense2.RS2_CAMERA_INFO_NAME, error);
         LogTools.info("Sensor {}: {}", i, new String(friendlyNameBytePointer.getStringBytes()));

      }

      sensor = realsense2.rs2_create_sensor(sensorList, 0, error);
      checkError(true, "Failed to create sensor.");

      LogTools.info("Configured Depth Stream of Realsense Device. Serial number: {}", serialNumber);
   }

   public void enableColor(RealsenseConfiguration realsenseConfiguration)
   {
      enableColor(realsenseConfiguration.getColorWidth(), realsenseConfiguration.getColorHeight(), realsenseConfiguration.getColorFPS());
   }

   public void enableColor(int colorWidth, int colorHeight, int fps)
   {
      this.colorWidth = colorWidth;
      this.colorHeight = colorHeight;

      realsense2.rs2_config_enable_stream(config, realsense2.RS2_STREAM_COLOR, COLOR_STREAM_INDEX, colorWidth, colorHeight, realsense2.RS2_FORMAT_RGB8, fps, error);
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

      LogTools.info("Started processing queue");
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
               checkError(false, "Failed to get depth stream intrinsics.");

               LogTools.info("Depth intrinsics: {}", String.format("Depth: fx: %.4f, fy: %.4f, cx: %.4f, cy: %.4f, h: %d, w: %d",
                                                                   depthStreamIntrinsics.fx(),
                                                                   depthStreamIntrinsics.fy(),
                                                                   depthStreamIntrinsics.ppx(),
                                                                   depthStreamIntrinsics.ppy(),
                                                                   depthHeight,
                                                                   depthWidth));
               depthDiscretization = realsense2.rs2_get_depth_scale(sensor, error);
               LogTools.info("Depth discretization: {} (meters/unit)", depthDiscretization);
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
                  checkError(false, "Failed to get color stream intrinsics.");

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

   public void setDigitalGain(int digitalGain)
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

   public CameraPinholeBrown getDepthCameraIntrinsics()
   {
      if (depthCameraIntrinsics == null)
      {
         depthCameraIntrinsics = new CameraPinholeBrown();
         depthCameraIntrinsics.setFx(getDepthFocalLengthPixelsX());
         depthCameraIntrinsics.setFy(getDepthFocalLengthPixelsY());
         depthCameraIntrinsics.setSkew(0.0);
         depthCameraIntrinsics.setCx(getDepthPrincipalOffsetXPixels());
         depthCameraIntrinsics.setCy(getDepthPrincipalOffsetYPixels());
      }
      return depthCameraIntrinsics;
   }

   public CameraPinholeBrown getColorCameraIntrinsics()
   {
      if (colorCameraIntrinsics == null)
      {
         colorCameraIntrinsics = new CameraPinholeBrown();
         colorCameraIntrinsics.setFx(getColorFocalLengthPixelsX());
         colorCameraIntrinsics.setFy(getColorFocalLengthPixelsY());
         colorCameraIntrinsics.setSkew(0.0);
         colorCameraIntrinsics.setCx(getColorPrincipalOffsetXPixels());
         colorCameraIntrinsics.setCy(getColorPrincipalOffsetYPixels());
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
