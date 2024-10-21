package us.ihmc.sensors;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.zed.SL_CalibrationParameters;
import us.ihmc.zed.SL_CameraInformation;
import us.ihmc.zed.SL_InitParameters;
import us.ihmc.zed.SL_RuntimeParameters;
import us.ihmc.zed.library.ZEDJavaAPINativeLibrary;

import java.time.Instant;
import java.util.function.Supplier;

import static us.ihmc.zed.global.zed.*;

public class ZEDImageGrabber implements ImageSensorGrabber
{
   static
   {
      ZEDJavaAPINativeLibrary.load();
   }

   public static final int LEFT_COLOR_IMAGE_KEY = 0;
   public static final int RIGHT_COLOR_IMAGE_KEY = 1;
   public static final int DEPTH_IMAGE_KEY = 2;

   public static final int OUTPUT_IMAGE_COUNT = 3;

   protected static final int CAMERA_FPS = 30;
   private static final float MILLIMETER_TO_METERS = 0.001f;

   private final int cameraID;
   private final ZEDModelData zedModel;

   private final RawImage[] grabbedImages = new RawImage[OUTPUT_IMAGE_COUNT];
   private final Pointer[] slMatPointers = new Pointer[OUTPUT_IMAGE_COUNT];
   private final CameraIntrinsics leftSensorIntrinsics = new CameraIntrinsics();
   private final CameraIntrinsics rightSensorIntrinsics = new CameraIntrinsics();
   private int imageWidth;
   private int imageHeight;

   private final Supplier<ReferenceFrame> sensorFrameSupplier;
   private float sensorCenterToCameraDistanceY = 0.0f;
   private final FramePose3D leftSensorPose = new FramePose3D();
   private final FramePose3D rightSensorPose = new FramePose3D();

   private long grabSequenceNumber = 0L;
   private Instant lastGrabTime;
   private boolean lastGrabFailed = false;

   private final SL_InitParameters zedInitParameters = new SL_InitParameters();
   private final SL_RuntimeParameters zedRuntimeParameters = new SL_RuntimeParameters();

   public ZEDImageGrabber(int cameraID, ZEDModelData zedModel, Supplier<ReferenceFrame> sensorFrameSupplier)
   {
      this.cameraID = cameraID;
      this.zedModel = zedModel;
      this.sensorFrameSupplier = sensorFrameSupplier;

      // Set runtime parameters to default values
      zedRuntimeParameters.reference_frame(SL_REFERENCE_FRAME_CAMERA);
      zedRuntimeParameters.enable_depth(true);
      zedRuntimeParameters.confidence_threshold(100);
      zedRuntimeParameters.texture_confidence_threshold(100);
      zedRuntimeParameters.remove_saturated_areas(true);
      zedRuntimeParameters.enable_fill_mode(false);
   }

   @Override
   public boolean startSensor()
   {
      try
      {
         sl_create_camera(cameraID);

         // Set the initialization parameters
         setInitParameters(zedInitParameters);

         // Open the camera
         int returnCode = sl_open_camera(cameraID, zedInitParameters, 0, "", "", 0, "", "", "");
         throwOnError(returnCode);

         // Get camera intrinsics
         SL_CalibrationParameters sensorIntrinsics = sl_get_calibration_parameters(cameraID, false);
         imageWidth = sl_get_width(cameraID);
         imageHeight = sl_get_height(cameraID);

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
         sensorIntrinsics.close();

         // Get center to camera distance
         SL_CameraInformation cameraInformation = sl_get_camera_information(cameraID, 0, 0);
         sensorCenterToCameraDistanceY = 0.5f * cameraInformation.camera_configuration().calibration_parameters().translation().y();
         cameraInformation.close();

         // Create image retrieval pointers
         slMatPointers[LEFT_COLOR_IMAGE_KEY] = new Pointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U8_C4, SL_MEM_GPU));
         slMatPointers[RIGHT_COLOR_IMAGE_KEY] = new Pointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U8_C4, SL_MEM_GPU));
         slMatPointers[DEPTH_IMAGE_KEY] = new Pointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U16_C1, SL_MEM_GPU));
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
      parametersToSet.camera_fps(CAMERA_FPS);
      parametersToSet.resolution(SL_RESOLUTION_HD720);
      parametersToSet.input_type(SL_INPUT_TYPE_USB);
      parametersToSet.camera_device_id(cameraID);
      parametersToSet.camera_image_flip(SL_FLIP_MODE_OFF);
      parametersToSet.camera_disable_self_calib(false);
      parametersToSet.enable_image_enhancement(true);
      parametersToSet.svo_real_time_mode(true);
      parametersToSet.depth_mode(SL_DEPTH_MODE_ULTRA);
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

   @Override
   public boolean isRunning()
   {
      return sl_is_opened(cameraID) && !lastGrabFailed;
   }

   @Override
   public synchronized boolean grab()
   {
      // Update the sensor pose
      ReferenceFrame sensorFrame = sensorFrameSupplier.get();

      leftSensorPose.setToZero(sensorFrame);
      leftSensorPose.getPosition().subY(sensorCenterToCameraDistanceY);
      leftSensorPose.changeFrame(ReferenceFrame.getWorldFrame());

      rightSensorPose.setToZero(sensorFrame);
      rightSensorPose.getPosition().addY(sensorCenterToCameraDistanceY);
      rightSensorPose.changeFrame(ReferenceFrame.getWorldFrame());

      int returnCode;
      try
      {
         // Grab images now
         returnCode = sl_grab(cameraID, zedRuntimeParameters);
         throwOnError(returnCode);
         lastGrabTime = Instant.now();
         ++grabSequenceNumber;

         // Retrieve the grabbed depth image
         Pointer depthImagePointer = slMatPointers[DEPTH_IMAGE_KEY];
         returnCode = sl_retrieve_measure(cameraID, depthImagePointer, SL_MEASURE_DEPTH_U16_MM, SL_MEM_GPU, imageWidth, imageHeight);
         throwOnError(returnCode);

         // Retrieve the grabbed left color image
         Pointer leftColorImagePointer = slMatPointers[LEFT_COLOR_IMAGE_KEY];
         returnCode = sl_retrieve_image(cameraID, leftColorImagePointer, SL_VIEW_LEFT, SL_MEM_GPU, imageWidth, imageHeight);
         throwOnError(returnCode);

         // Retrieve the grabbed right color image
         Pointer rightColorImagePointer = slMatPointers[RIGHT_COLOR_IMAGE_KEY];
         returnCode = sl_retrieve_image(cameraID, rightColorImagePointer, SL_VIEW_RIGHT, SL_MEM_GPU, imageWidth, imageHeight);
         throwOnError(returnCode);

         synchronized (grabbedImages)
         {  // Create RawImages from the grabbed retrieved slMats
            grabbedImages[LEFT_COLOR_IMAGE_KEY] = slMatToRawImage(leftColorImagePointer, PixelFormat.BGRA8, leftSensorIntrinsics, leftSensorPose);
            grabbedImages[RIGHT_COLOR_IMAGE_KEY] = slMatToRawImage(rightColorImagePointer, PixelFormat.BGRA8, rightSensorIntrinsics, rightSensorPose);
            grabbedImages[DEPTH_IMAGE_KEY] = slMatToRawImage(depthImagePointer, PixelFormat.GRAY16, leftSensorIntrinsics, leftSensorPose);
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

   private RawImage slMatToRawImage(Pointer slMatPointer, PixelFormat imagePixelFormat, CameraIntrinsics cameraIntrinsics, FramePose3D sensorPose)
   {
      GpuMat imageGpuMat = new GpuMat(imageHeight,
                                      imageWidth,
                                      imagePixelFormat.toOpenCVType(),
                                      sl_mat_get_ptr(slMatPointer, SL_MEM_GPU),
                                      sl_mat_get_step_bytes(slMatPointer, SL_MEM_GPU));
      return new RawImage(null, imageGpuMat, imagePixelFormat, cameraIntrinsics, sensorPose, lastGrabTime, grabSequenceNumber, MILLIMETER_TO_METERS);
   }

   @Override
   public RawImage getImage(int imageKey)
   {
      synchronized (grabbedImages)
      {
         return grabbedImages[imageKey].get();
      }
   }

   @Override
   public String getSensorName()
   {
      return zedModel.name();
   }

   @Override
   public synchronized void close()
   {
      System.out.println("Closing " + getClass().getSimpleName());

      for (Pointer slMat : slMatPointers)
      {
         if (slMat != null && !slMat.isNull())
         {
            sl_mat_free(slMat, SL_MEM_GPU);
            slMat.close();
         }
      }

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
}
