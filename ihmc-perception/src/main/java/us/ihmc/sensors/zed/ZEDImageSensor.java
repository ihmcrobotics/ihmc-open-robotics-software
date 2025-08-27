package us.ihmc.sensors.zed;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.zed.SL_CalibrationParameters;
import us.ihmc.zed.SL_InitParameters;
import us.ihmc.zed.SL_PositionalTrackingParameters;
import us.ihmc.zed.SL_Quaternion;
import us.ihmc.zed.SL_RuntimeParameters;
import us.ihmc.zed.SL_Vector3;
import us.ihmc.zed.library.ZEDJavaAPINativeLibrary;

import java.time.Instant;

import static us.ihmc.zed.global.zed.*;

public class ZEDImageSensor extends ImageSensor
{
   static
   {
      ZEDJavaAPINativeLibrary.load();
   }

   private static int nextStreamingPort = 30000;

   public static final int LEFT_COLOR_IMAGE_KEY = 0;
   public static final int RIGHT_COLOR_IMAGE_KEY = 1;
   public static final int DEPTH_IMAGE_KEY = 2;

   public static final int OUTPUT_IMAGE_COUNT = 3;

   private static final int DEFAULT_RESOLUTION = SL_RESOLUTION_HD720;
   private static final int DEFAULT_FPS = 15;
   private static final float MILLIMETER_TO_METERS = 0.001f;

   private final int cameraID;
   private final ZEDModelData zedModel;
   private final int slInputType;
   private final int slDepthMode;
   private final int resolution;
   private int fps;
   private int bitrate;
   private String remoteStreamingAddress;
   private int remoteStreamingPort;
   private final int localStreamingPort = nextStreamingPort++;

   private final RawImage[] grabbedImages = new RawImage[OUTPUT_IMAGE_COUNT];
   private final Pointer[] slMatPointers = new Pointer[OUTPUT_IMAGE_COUNT];
   private final CameraIntrinsics leftSensorIntrinsics = new CameraIntrinsics();
   private final CameraIntrinsics rightSensorIntrinsics = new CameraIntrinsics();
   private int imageWidth;
   private int imageHeight;

   private float sensorCenterToCameraDistanceY;
   private ReferenceFrame leftSensorFrame = null;
   private ReferenceFrame rightSensorFrame = null;

   private long grabSequenceNumber = 0L;
   private Instant lastGrabTime;
   private boolean lastGrabFailed = false;
   private long lastGrabTimestamp;

   protected final SL_InitParameters zedInitParameters = new SL_InitParameters();
   protected final SL_RuntimeParameters zedRuntimeParameters = new SL_RuntimeParameters();

   private boolean positionalTrackingEnabled = false;
   private final MutableReferenceFrame trackedSensorFrame;
   private final SL_Quaternion sensorRotation = new SL_Quaternion();
   private final SL_Vector3 sensorTranslation = new SL_Vector3();

   public ZEDImageSensor(int cameraID, ZEDModelData zedModel, int slInputType, int slDepthMode)
   {
      this(cameraID, zedModel, slInputType, slDepthMode, DEFAULT_RESOLUTION, DEFAULT_FPS);
   }

   /**
    * See the documentation for the available resolutions and frame rates:
    * <ul>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_QHDPLUS}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_HD2K}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_HD1080}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_HD1200}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_HD720}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_SVGA}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_VGA}</li>
    * </ul>
    */
   public ZEDImageSensor(int cameraID, ZEDModelData zedModel, int slInputType, int slDepthMode, int resolution, int fps)
   {
      super(zedModel.name());

      this.cameraID = cameraID;
      this.zedModel = zedModel;
      this.slInputType = slInputType;
      this.slDepthMode = slDepthMode;
      this.resolution = resolution;
      this.fps = fps;

      trackedSensorFrame = new MutableReferenceFrame(getSensorName() + "_tracked", ReferenceFrameTools.getWorldFrame());

      sensorCenterToCameraDistanceY = (float) zedModel.getCenterToCameraDistance();
      updateReferenceFrames();

      // Set runtime parameters to default values
      zedRuntimeParameters.reference_frame(SL_REFERENCE_FRAME_CAMERA);
      zedRuntimeParameters.enable_depth(slDepthMode != SL_DEPTH_MODE_NONE);
      zedRuntimeParameters.confidence_threshold(70);
      zedRuntimeParameters.texture_confidence_threshold(100);
      zedRuntimeParameters.remove_saturated_areas(true);
      zedRuntimeParameters.enable_fill_mode(false);
   }

   /**
    * Constructor to connect to a remote ZED SDK instance
    */
   public ZEDImageSensor(int cameraID, ZEDModelData zedModel, int slDepthMode, String remoteStreamingAddress, int remoteStreamingPort)
   {
      this(cameraID, zedModel, SL_INPUT_TYPE_STREAM, slDepthMode);

      this.remoteStreamingAddress = remoteStreamingAddress;
      this.remoteStreamingPort = remoteStreamingPort;
   }

   private void updateReferenceFrames()
   {
      if (leftSensorFrame != null)
         leftSensorFrame.remove();
      RigidBodyTransform leftSensorTransform = new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, sensorCenterToCameraDistanceY, 0.0));
      leftSensorFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(getSensorName() + "_left", sensorFrame, leftSensorTransform);

      if (rightSensorFrame != null)
         rightSensorFrame.remove();
      RigidBodyTransform rightSensorTransform = new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, -sensorCenterToCameraDistanceY, 0.0));
      rightSensorFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(getSensorName() + "_right", sensorFrame, rightSensorTransform);
   }

   @Override
   protected void onSensorFrameChanged()
   {
      updateReferenceFrames();
   }

   @Override
   protected boolean startSensor()
   {
      try
      {
         if (sl_is_opened(cameraID))
            sl_close_camera(cameraID);

         sl_create_camera(cameraID);

         // Set the initialization parameters
         setInitParameters(zedInitParameters);

         // Open the camera
         int returnCode = openCamera();
         throwOnError(returnCode);

         if (positionalTrackingEnabled)
         {
            SL_PositionalTrackingParameters positionalTrackingParameters = sl_get_positional_tracking_parameters(cameraID);
            sl_enable_positional_tracking(cameraID, positionalTrackingParameters, "");
         }

         // Get camera intrinsics
         SL_CalibrationParameters sensorIntrinsics = sl_get_calibration_parameters(cameraID, false);
         imageWidth = sl_get_width(cameraID);
         imageHeight = sl_get_height(cameraID);
         fps = (int) sl_get_camera_fps(cameraID);
         bitrate = calculateBitrate(imageWidth, imageHeight, fps);

         if (slInputType != SL_INPUT_TYPE_STREAM)
         {
            int gopSize = -1;
            int adaptativeBitrate = 0;
            int chunkSize = 16084;
            sl_enable_streaming(cameraID, SL_STREAMING_CODEC_H264, bitrate, (short) localStreamingPort, gopSize, adaptativeBitrate, chunkSize, fps);
         }

         leftSensorIntrinsics.setWidth(imageWidth);
         leftSensorIntrinsics.setHeight(imageHeight);
         leftSensorIntrinsics.setFx(sensorIntrinsics.left_cam().fx());
         leftSensorIntrinsics.setFy(sensorIntrinsics.left_cam().fy());
         leftSensorIntrinsics.setCx(sensorIntrinsics.left_cam().cx());
         leftSensorIntrinsics.setCy(sensorIntrinsics.left_cam().cy());

         rightSensorIntrinsics.setWidth(imageWidth);
         rightSensorIntrinsics.setHeight(imageHeight);
         rightSensorIntrinsics.setFx(sensorIntrinsics.right_cam().fx());
         rightSensorIntrinsics.setFy(sensorIntrinsics.right_cam().fy());
         rightSensorIntrinsics.setCx(sensorIntrinsics.right_cam().cx());
         rightSensorIntrinsics.setCy(sensorIntrinsics.right_cam().cy());

         sensorCenterToCameraDistanceY = -0.5f * sensorIntrinsics.translation().y();
         sensorIntrinsics.close();

         updateReferenceFrames();

         // Create image retrieval pointers
         slMatPointers[LEFT_COLOR_IMAGE_KEY] = sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U8_C4, SL_MEM_GPU);
         slMatPointers[RIGHT_COLOR_IMAGE_KEY] = sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U8_C4, SL_MEM_GPU);
         slMatPointers[DEPTH_IMAGE_KEY] = sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U16_C1, SL_MEM_GPU);
      }
      catch (ZEDException exception)
      {
         LogTools.error(exception);

         return false;
      }

      lastGrabFailed = false;
      return true;
   }

   protected void setInitParameters(SL_InitParameters parametersToSet)
   {
      parametersToSet.camera_fps(fps);
      parametersToSet.resolution(resolution);
      parametersToSet.input_type(slInputType);
      parametersToSet.camera_device_id(cameraID);
      parametersToSet.camera_image_flip(SL_FLIP_MODE_OFF);
      parametersToSet.camera_disable_self_calib(false);
      parametersToSet.enable_image_enhancement(true);
      if (slDepthMode == SL_DEPTH_MODE_NEURAL || slDepthMode == SL_DEPTH_MODE_NEURAL_PLUS)
         LogTools.info("ZED SDK will use neural depth mode. This uses significant GPU resources.");
      parametersToSet.depth_mode(slDepthMode);
      parametersToSet.depth_stabilization(1);
      parametersToSet.depth_maximum_distance(zedModel.getMaximumDepthDistance());
      parametersToSet.depth_minimum_distance(zedModel.getMinimumDepthDistance());
      parametersToSet.coordinate_unit(SL_UNIT_METER);
      parametersToSet.coordinate_system(SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD);
      parametersToSet.sdk_gpu_id(-1); // Will find and use the best available GPU
      parametersToSet.sdk_verbose(0); // false
      parametersToSet.sensors_required(true);
      parametersToSet.enable_right_side_measure(false);
      parametersToSet.open_timeout_sec(5.0f);
      parametersToSet.async_grab_camera_recovery(false);
   }

   protected int openCamera()
   {
      if (slInputType == SL_INPUT_TYPE_STREAM)
         return sl_open_camera(cameraID, zedInitParameters, 0, "", remoteStreamingAddress, remoteStreamingPort, "", "", "");
      else
         return sl_open_camera(cameraID, zedInitParameters, 0, "", "", 0, "", "", "");
   }

   @Override
   public boolean isSensorRunning()
   {
      boolean recentlyGrabbed = lastGrabTime != null && lastGrabTime.isAfter(Instant.now().minusSeconds(1));
      return sl_is_opened(cameraID) && !lastGrabFailed && recentlyGrabbed;
   }

   @Override
   protected boolean grab()
   {
      int returnCode;
      try
      {
         // Grab images now
         returnCode = sl_grab(cameraID, zedRuntimeParameters);
         if (returnCode == SL_ERROR_CODE_END_OF_SVOFILE_REACHED)
         {
            sl_set_svo_position(0, 0);

            if (positionalTrackingEnabled)
            {
               sensorRotation.x(0.0f);
               sensorRotation.y(0.0f);
               sensorRotation.z(0.0f);
               sensorRotation.w(1.0f);

               sensorTranslation.x(0.0f);
               sensorTranslation.y(0.0f);
               sensorTranslation.z(0.0f);

               sl_reset_positional_tracking(cameraID, sensorRotation, sensorTranslation);
               trackedSensorFrame.update(RigidBodyTransformBasics::setToZero);
            }

            return false;
         }

         throwOnError(returnCode);
         lastGrabTime = Instant.now();
         ++grabSequenceNumber;

         lastGrabTimestamp = sl_get_current_timestamp(cameraID);

         // Update tracked position if tracking enabled
         if (positionalTrackingEnabled)
         {
            sl_get_position(cameraID, sensorRotation, sensorTranslation, SL_REFERENCE_FRAME_WORLD);

            Quaternion euclidRotation = new Quaternion(sensorRotation.x(), sensorRotation.y(), sensorRotation.z(), sensorRotation.w());
            Vector3D euclidTranslation = new Vector3D(sensorTranslation.x(), sensorTranslation.y(), sensorTranslation.z());
            if (!euclidRotation.containsNaN() && !euclidTranslation.containsNaN())
               trackedSensorFrame.update(transformToWorld -> transformToWorld.set(euclidRotation, euclidTranslation));
         }

         // Retrieve the grabbed depth image
         Pointer depthImagePointer = slMatPointers[DEPTH_IMAGE_KEY];
         returnCode = sl_retrieve_measure(cameraID, depthImagePointer, SL_MEASURE_DEPTH_U16_MM, SL_MEM_GPU, imageWidth, imageHeight, null); // TODO: Pass custream
         throwOnError(returnCode);

         // Retrieve the grabbed left color image
         Pointer leftColorImagePointer = slMatPointers[LEFT_COLOR_IMAGE_KEY];
         returnCode = sl_retrieve_image(cameraID, leftColorImagePointer, SL_VIEW_LEFT, SL_MEM_GPU, imageWidth, imageHeight, null); // TODO: Pass custream
         throwOnError(returnCode);

         // Retrieve the grabbed right color image
         Pointer rightColorImagePointer = slMatPointers[RIGHT_COLOR_IMAGE_KEY];
         returnCode = sl_retrieve_image(cameraID, rightColorImagePointer, SL_VIEW_RIGHT, SL_MEM_GPU, imageWidth, imageHeight, null); // TODO: Pass custream
         throwOnError(returnCode);

         synchronized (grabbedImages)
         {  // Create RawImages from the grabbed retrieved slMats
            if (grabbedImages[LEFT_COLOR_IMAGE_KEY] != null)
               grabbedImages[LEFT_COLOR_IMAGE_KEY].release();
            grabbedImages[LEFT_COLOR_IMAGE_KEY] = slMatToRawImage(leftColorImagePointer,
                                                                  PixelFormat.BGRA8,
                                                                  leftSensorIntrinsics,
                                                                  leftSensorFrame.getTransformToRoot());

            if (grabbedImages[RIGHT_COLOR_IMAGE_KEY] != null)
               grabbedImages[RIGHT_COLOR_IMAGE_KEY].release();
            grabbedImages[RIGHT_COLOR_IMAGE_KEY] = slMatToRawImage(rightColorImagePointer,
                                                                   PixelFormat.BGRA8,
                                                                   rightSensorIntrinsics,
                                                                   rightSensorFrame.getTransformToRoot());

            if (grabbedImages[DEPTH_IMAGE_KEY] != null)
               grabbedImages[DEPTH_IMAGE_KEY].release();
            grabbedImages[DEPTH_IMAGE_KEY] = slMatToRawImage(depthImagePointer, PixelFormat.GRAY16, leftSensorIntrinsics, leftSensorFrame.getTransformToRoot());
         }
      }
      catch (ZEDException exception)
      {
         LogTools.error(exception);
         lastGrabFailed = true;
         return false;
      }

      lastGrabFailed = false;
      return true;
   }

   private RawImage slMatToRawImage(Pointer slMatPointer,
                                    PixelFormat imagePixelFormat,
                                    CameraIntrinsics cameraIntrinsics,
                                    RigidBodyTransformReadOnly sensorTransform)
   {
      GpuMat imageGpuMat = new GpuMat(imageHeight,
                                      imageWidth,
                                      imagePixelFormat.toOpenCVType(),
                                      sl_mat_get_ptr(slMatPointer, SL_MEM_GPU),
                                      sl_mat_get_step_bytes(slMatPointer, SL_MEM_GPU));
      return new RawImage(null,
                          imageGpuMat.clone(),
                          imagePixelFormat,
                          cameraIntrinsics,
                          CameraModel.PINHOLE,
                          sensorTransform,
                          lastGrabTime,
                          grabSequenceNumber,
                          MILLIMETER_TO_METERS);
   }

   @Override
   public int[] getImageKeys()
   {
      return new int[] {LEFT_COLOR_IMAGE_KEY, RIGHT_COLOR_IMAGE_KEY, DEPTH_IMAGE_KEY};
   }

   @Override
   public RawImage getImage(int imageKey)
   {
      synchronized (grabbedImages)
      {
         if (grabbedImages[imageKey] == null)
            return null;

         return grabbedImages[imageKey].get();
      }
   }

   @Override
   public ReferenceFrame getImageFrame(int imageKey)
   {
      return switch (imageKey)
      {
         case LEFT_COLOR_IMAGE_KEY, DEPTH_IMAGE_KEY -> leftSensorFrame;
         case RIGHT_COLOR_IMAGE_KEY -> rightSensorFrame;
         default -> null;
      };
   }

   @Override
   public ReferenceFrame[] getImageFrames()
   {
      return new ReferenceFrame[] {leftSensorFrame, rightSensorFrame};
   }

   public ReferenceFrame getTrackedSensorFrame()
   {
      return trackedSensorFrame.getReferenceFrame();
   }

   public int getCameraID()
   {
      return cameraID;
   }

   public int getStreamingPort()
   {
      return localStreamingPort;
   }

   public void enablePositionalTracking(boolean enable)
   {
      positionalTrackingEnabled = enable;
   }

   @Override
   public void close()
   {
      System.out.println("Closing " + getClass().getSimpleName());
      super.close();

      for (Pointer slMat : slMatPointers)
      {
         if (slMat != null && !slMat.isNull())
         {
            sl_mat_free(slMat, SL_MEM_GPU);
            slMat.close();
         }
      }

      for (RawImage image : grabbedImages)
         if (image != null)
            image.release();

      sl_close_camera(cameraID);

      System.out.println("Closed " + getClass().getSimpleName());
   }

   private void throwOnError(int errorCode) throws ZEDException
   {
      if (errorCode != SL_ERROR_CODE_SUCCESS)
         throw new ZEDException(errorCode);
   }

   private class ZEDException extends Exception
   {
      private final int zedErrorCode;

      public ZEDException(int zedErrorCode)
      {
         this.zedErrorCode = zedErrorCode;
      }

      @Override
      public String getMessage()
      {
         return "ZED Error (%d): %s".formatted(zedErrorCode, getZEDErrorName(zedErrorCode));
      }
   }

   private String getZEDErrorName(int errorCode)
   {
      return switch (errorCode)
      {
         case SL_ERROR_CODE_CORRUPTED_FRAME -> "SL_ERROR_CODE_CORRUPTED_FRAME";
         case SL_ERROR_CODE_CAMERA_REBOOTING -> "SL_ERROR_CODE_CAMERA_REBOOTING";
         case SL_ERROR_CODE_SUCCESS -> "SL_ERROR_CODE_SUCCESS";
         case SL_ERROR_CODE_FAILURE -> "SL_ERROR_CODE_FAILURE";
         case SL_ERROR_CODE_NO_GPU_COMPATIBLE -> "SL_ERROR_CODE_NO_GPU_COMPATIBLE";
         case SL_ERROR_CODE_NOT_ENOUGH_GPU_MEMORY -> "SL_ERROR_CODE_NOT_ENOUGH_GPU_MEMORY";
         case SL_ERROR_CODE_CAMERA_NOT_DETECTED -> "SL_ERROR_CODE_CAMERA_NOT_DETECTED";
         case SL_ERROR_CODE_SENSORS_NOT_INITIALIZED -> "SL_ERROR_CODE_SENSORS_NOT_INITIALIZED";
         case SL_ERROR_CODE_SENSORS_NOT_AVAILABLE -> "SL_ERROR_CODE_SENSORS_NOT_AVAILABLE";
         case SL_ERROR_CODE_INVALID_RESOLUTION -> "SL_ERROR_CODE_INVALID_RESOLUTION";
         case SL_ERROR_CODE_LOW_USB_BANDWIDTH -> "SL_ERROR_CODE_LOW_USB_BANDWIDTH";
         case SL_ERROR_CODE_CALIBRATION_FILE_NOT_AVAILABLE -> "SL_ERROR_CODE_CALIBRATION_FILE_NOT_AVAILABLE";
         case SL_ERROR_CODE_INVALID_CALIBRATION_FILE -> "SL_ERROR_CODE_INVALID_CALIBRATION_FILE";
         case SL_ERROR_CODE_INVALID_SVO_FILE -> "SL_ERROR_CODE_INVALID_SVO_FILE";
         case SL_ERROR_CODE_SVO_RECORDING_ERROR -> "SL_ERROR_CODE_SVO_RECORDING_ERROR";
         case SL_ERROR_CODE_SVO_UNSUPPORTED_COMPRESSION -> "SL_ERROR_CODE_SVO_UNSUPPORTED_COMPRESSION";
         case SL_ERROR_CODE_END_OF_SVOFILE_REACHED -> "SL_ERROR_CODE_END_OF_SVOFILE_REACHED";
         case SL_ERROR_CODE_INVALID_COORDINATE_SYSTEM -> "SL_ERROR_CODE_INVALID_COORDINATE_SYSTEM";
         case SL_ERROR_CODE_INVALID_FIRMWARE -> "SL_ERROR_CODE_INVALID_FIRMWARE";
         case SL_ERROR_CODE_INVALID_FUNCTION_PARAMETERS -> "SL_ERROR_CODE_INVALID_FUNCTION_PARAMETERS";
         case SL_ERROR_CODE_CUDA_ERROR -> "SL_ERROR_CODE_CUDA_ERROR";
         case SL_ERROR_CODE_CAMERA_NOT_INITIALIZED -> "SL_ERROR_CODE_CAMERA_NOT_INITIALIZED";
         case SL_ERROR_CODE_NVIDIA_DRIVER_OUT_OF_DATE -> "SL_ERROR_CODE_NVIDIA_DRIVER_OUT_OF_DATE";
         case SL_ERROR_CODE_INVALID_FUNCTION_CALL -> "SL_ERROR_CODE_INVALID_FUNCTION_CALL";
         case SL_ERROR_CODE_CORRUPTED_SDK_INSTALLATION -> "SL_ERROR_CODE_CORRUPTED_SDK_INSTALLATION";
         case SL_ERROR_CODE_INCOMPATIBLE_SDK_VERSION -> "SL_ERROR_CODE_INCOMPATIBLE_SDK_VERSION";
         case SL_ERROR_CODE_INVALID_AREA_FILE -> "SL_ERROR_CODE_INVALID_AREA_FILE";
         case SL_ERROR_CODE_INCOMPATIBLE_AREA_FILE -> "SL_ERROR_CODE_INCOMPATIBLE_AREA_FILE";
         case SL_ERROR_CODE_CAMERA_FAILED_TO_SETUP -> "SL_ERROR_CODE_CAMERA_FAILED_TO_SETUP";
         case SL_ERROR_CODE_CAMERA_DETECTION_ISSUE -> "SL_ERROR_CODE_CAMERA_DETECTION_ISSUE";
         case SL_ERROR_CODE_CANNOT_START_CAMERA_STREAM -> "SL_ERROR_CODE_CANNOT_START_CAMERA_STREAM";
         case SL_ERROR_CODE_NO_GPU_DETECTED -> "SL_ERROR_CODE_NO_GPU_DETECTED";
         case SL_ERROR_CODE_PLANE_NOT_FOUND -> "SL_ERROR_CODE_PLANE_NOT_FOUND";
         case SL_ERROR_CODE_MODULE_NOT_COMPATIBLE_WITH_CAMERA -> "SL_ERROR_CODE_MODULE_NOT_COMPATIBLE_WITH_CAMERA";
         case SL_ERROR_CODE_MOTION_SENSORS_REQUIRED -> "SL_ERROR_CODE_MOTION_SENSORS_REQUIRED";
         case SL_ERROR_CODE_MODULE_NOT_COMPATIBLE_WITH_CUDA_VERSION -> "SL_ERROR_CODE_MODULE_NOT_COMPATIBLE_WITH_CUDA_VERSION";
         case SL_ERROR_CODE_SENSORS_DATA_REQUIRED -> "SL_ERROR_CODE_SENSORS_DATA_REQUIRED";
         default -> "UNKNOWN";
      };
   }

   private static int calculateBitrate(int width, int height, int fps)
   {
      double bpp = 0.128; // Derived from 8000 kbps for 1080p30
      return (int) ((width * height * fps * bpp) / 1000);
   }

   public int getFps()
   {
      return fps;
   }

   /** In the case we are streaming */
   public int getStreamingBitrate()
   {
      return bitrate;
   }

   public long getLastGrabTimestamp()
   {
      return lastGrabTimestamp;
   }
}
