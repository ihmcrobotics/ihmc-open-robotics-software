package us.ihmc.sensors;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.RestartableThread;
import us.ihmc.zed.SL_CalibrationParameters;
import us.ihmc.zed.SL_InitParameters;
import us.ihmc.zed.SL_RuntimeParameters;
import us.ihmc.zed.library.ZEDJavaAPINativeLibrary;

import java.time.Instant;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import static us.ihmc.zed.global.zed.*;

/**
 * Encodes and publishes color and depth images from a ZED sensor.
 * The depth image is aligned to the left camera of the ZED.
 */
public class ZEDColorDepthImageRetriever
{
   static {
      ZEDJavaAPINativeLibrary.load();
   }

   private static final int CAMERA_FPS = 30;
   private static final float MILLIMETER_TO_METERS = 0.001f;

   private final int cameraID;
   private ZEDModelData zedModelData;
   private SL_RuntimeParameters zedRuntimeParameters;

   private int imageWidth; // Width of rectified image in pixels (color image width == depth image width)
   private int imageHeight; // Height of rectified image in pixels (color image height ==  depth image height)
   private final SideDependentList<Float> cameraFocalLengthX = new SideDependentList<>();
   private final SideDependentList<Float> cameraFocalLengthY = new SideDependentList<>();
   private final SideDependentList<Float> cameraPrincipalPointX = new SideDependentList<>();
   private final SideDependentList<Float> cameraPrincipalPointY = new SideDependentList<>();

   private Pointer colorImagePointer;
   private Pointer depthImagePointer;
   private long grabSequenceNumber = 0L;

   private final AtomicReference<Instant> colorImageAcquisitionTime = new AtomicReference<>();
   private final AtomicReference<Instant> depthImageAcquisitionTime = new AtomicReference<>();

   private GpuMat depthGpuMat;
   private final SideDependentList<GpuMat> colorGpuMats = new SideDependentList<>();
   private final SideDependentList<RawImage> colorImages = new SideDependentList<>(null, null);
   private RawImage depthImage = null;

   // Frame poses of left and right cameras of ZED. Depth is always in the left pose.
   private final SideDependentList<FramePose3D> cameraFramePoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final RestartableThread zedGrabThread;
   
   private final Lock newDepthImageLock = new ReentrantLock();
   private final Condition newDepthImageAvailable = newDepthImageLock.newCondition();
   private long lastDepthSequenceNumber = -1L;

   private final SideDependentList<Lock> newColorImageLocks = new SideDependentList<>(new ReentrantLock(), new ReentrantLock());
   private final SideDependentList<Condition> newColorImagesAvailable = new SideDependentList<>(newColorImageLocks.get(RobotSide.LEFT).newCondition(),
                                                                                                newColorImageLocks.get(RobotSide.RIGHT).newCondition());
   private SideDependentList<Long> lastColorSequenceNumbers = new SideDependentList<>(-1L, -1L);

   private boolean initialized = false;

   public ZEDColorDepthImageRetriever(int cameraID,
                                      Supplier<ReferenceFrame> sensorFrameSupplier,
                                      ROS2DemandGraphNode depthDemandNode,
                                      ROS2DemandGraphNode colorDemandNode)
   {
      this.cameraID = cameraID;


      zedGrabThread = new RestartableThread("ZEDImageGrabber", () ->
      {
         if (depthDemandNode == null || colorDemandNode == null || depthDemandNode.isDemanded() || colorDemandNode.isDemanded())
         {
            if (!initialized)
            {
               if (startZED())
                  initialized = true;
               else
                  ThreadTools.sleep(3000);
            }
            else
            {

               // sl_grab is blocking, and will wait for next frame available. No need to use throttler
               checkError("sl_grab", sl_grab(cameraID, zedRuntimeParameters));

               // Frame supplier provides frame pose of center of camera. Add Y to get left camera's frame pose
               for (RobotSide side : RobotSide.values)
               {
                  cameraFramePoses.get(side).setToZero(sensorFrameSupplier.get());
                  cameraFramePoses.get(side).getPosition().addY(side.negateIfRightSide(zedModelData.getCenterToCameraDistance()));
                  cameraFramePoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
               }

               retrieveAndSaveDepthImage();
               retrieveAndSaveColorImage(RobotSide.LEFT);
               retrieveAndSaveColorImage(RobotSide.RIGHT);
               grabSequenceNumber++;
            }
         }
         else
            ThreadTools.sleep(500);
      });
      zedGrabThread.start();
   }

   private void retrieveAndSaveColorImage(RobotSide side)
   {
      int slViewSide = side == RobotSide.LEFT ? SL_VIEW_LEFT : SL_VIEW_RIGHT;
      // Retrieve color image
      checkError("sl_retrieve_image", sl_retrieve_image(cameraID, colorImagePointer, slViewSide, SL_MEM_GPU, imageWidth, imageHeight));
      colorImageAcquisitionTime.set(Instant.now());

      // Convert to BGR and encode to jpeg
      GpuMat colorImageBGRA = new GpuMat(imageHeight,
                                         imageWidth,
                                         opencv_core.CV_8UC4,
                                         sl_mat_get_ptr(colorImagePointer, SL_MEM_GPU),
                                         sl_mat_get_step_bytes(colorImagePointer, SL_MEM_GPU));

      if (colorGpuMats.get(side) != null)
         colorGpuMats.get(side).close();
      colorGpuMats.put(side, new GpuMat(imageHeight, imageWidth, opencv_core.CV_8UC3));
      opencv_cudaimgproc.cvtColor(colorImageBGRA, colorGpuMats.get(side), opencv_imgproc.COLOR_BGRA2BGR);

      newColorImageLocks.get(side).lock();
      try
      {
         if (colorImages.get(side) != null)
            colorImages.get(side).release();
         colorImages.put(side,
                         new RawImage(grabSequenceNumber,
                                      colorImageAcquisitionTime.get(),
                                      imageWidth,
                                      imageHeight,
                                      MILLIMETER_TO_METERS,
                                      null,
                                      colorGpuMats.get(side).clone(),
                                      opencv_core.CV_8UC3,
                                      cameraFocalLengthX.get(side),
                                      cameraFocalLengthY.get(side),
                                      cameraPrincipalPointX.get(side),
                                      cameraPrincipalPointY.get(side),
                                      cameraFramePoses.get(side).getPosition(),
                                      cameraFramePoses.get(side).getOrientation()));

         newColorImagesAvailable.get(side).signal();
      }
      finally
      {
         newColorImageLocks.get(side).unlock();
      }

      // Close stuff
      colorImageBGRA.close();
   }

   private void retrieveAndSaveDepthImage()
   {
      // Retrieve depth image
      // There is a bug where retrieving the depth image using SL_MEM_CPU causes the depth image to be misaligned and very dark.
      // Thus, the image is retrieved onto a GpuMat then downloaded onto the CPU for further processing.
      checkError("sl_retrieve_measure", sl_retrieve_measure(cameraID, depthImagePointer, SL_MEASURE_DEPTH_U16_MM, SL_MEM_GPU, imageWidth, imageHeight));
      depthImageAcquisitionTime.set(Instant.now());

      if (depthGpuMat != null)
         depthGpuMat.close();
      depthGpuMat = new GpuMat(imageHeight,
                               imageWidth,
                               opencv_core.CV_16UC1,
                               sl_mat_get_ptr(depthImagePointer, SL_MEM_GPU),
                               sl_mat_get_step_bytes(depthImagePointer, SL_MEM_GPU));

      newDepthImageLock.lock();
      try
      {
         if (depthImage != null)
            depthImage.release();
         depthImage = new RawImage(grabSequenceNumber,
                                   depthImageAcquisitionTime.get(),
                                   imageWidth,
                                   imageHeight,
                                   MILLIMETER_TO_METERS,
                                   null,
                                   depthGpuMat.clone(),
                                   opencv_core.CV_16UC1,
                                   cameraFocalLengthX.get(RobotSide.LEFT),
                                   cameraFocalLengthY.get(RobotSide.LEFT),
                                   cameraPrincipalPointX.get(RobotSide.LEFT),
                                   cameraPrincipalPointY.get(RobotSide.LEFT),
                                   cameraFramePoses.get(RobotSide.LEFT).getPosition(),
                                   cameraFramePoses.get(RobotSide.LEFT).getOrientation());

         newDepthImageAvailable.signal();
      }
      finally
      {
         newDepthImageLock.unlock();
      }
   }

   public RawImage getLatestRawDepthImage()
   {
      newDepthImageLock.lock();
      try
      {
         while (depthImage == null || depthImage.isEmpty() || depthImage.getSequenceNumber() == lastDepthSequenceNumber)
         {
            newDepthImageAvailable.await();
         }

         lastDepthSequenceNumber = depthImage.getSequenceNumber();
      }
      catch (InterruptedException interruptedException)
      {
         LogTools.error(interruptedException.getMessage());
      }

      return depthImage.get();
   }

   public RawImage getLatestRawColorImage(RobotSide side)
   {
      newColorImageLocks.get(side).lock();
      try
      {
         while (colorImages.get(side) == null ||
                colorImages.get(side).isEmpty() ||
                colorImages.get(side).getSequenceNumber() == lastColorSequenceNumbers.get(side))
         {
            newColorImagesAvailable.get(side).await();
         }

         lastColorSequenceNumbers.put(side, colorImages.get(side).getSequenceNumber());
      }
      catch (InterruptedException interruptedException)
      {
         LogTools.error(interruptedException.getMessage());
      }

      return colorImages.get(side).get();
   }

   public ZEDModelData getZedModelData()
   {
      return zedModelData;
   }

   public void start()
   {
      zedGrabThread.start();
   }

   public void stop()
   {
      zedGrabThread.stop();
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      zedGrabThread.stop();

      if (depthImagePointer != null)
         depthImagePointer.close();
      if (depthImage != null)
         depthImage.release();

      if (colorImagePointer != null)
         colorImagePointer.close();
      for (RobotSide side : RobotSide.values)
      {
         if (colorImages.get(side) != null)
            colorImages.get(side).release();
      }
      sl_close_camera(cameraID);
      System.out.println("Destroyed " + getClass().getSimpleName());
   }

   private boolean startZED()
   {
      boolean success;

      zedRuntimeParameters = new SL_RuntimeParameters();

      LogTools.info("ZED SDK version: " + sl_get_sdk_version().getString());

      if (colorImagePointer != null && !colorImagePointer.isNull())
         colorImagePointer.close();
      if (depthImagePointer != null && !depthImagePointer.isNull())
         depthImagePointer.close();
      if (sl_is_opened(cameraID))
         sl_close_camera(cameraID);

      LogTools.info("Starting ZED...");

      // Create and initialize the camera
      success = sl_create_camera(cameraID);

      SL_InitParameters zedInitializationParameters = new SL_InitParameters();

      // TODO: We can't do this anymore since upgrading ZED SDK; it gives an invalid camera model
      // Open camera with default parameters to find model
      // Can't get the model number without opening the camera first
      // success = checkError("sl_open_camera", sl_open_camera(cameraID, zedInitializationParameters, 0, "", "", 0, "", "", ""));
      // if (!success)
      //    return success;
      // setZEDConfiguration(cameraID);
      // sl_close_camera(cameraID);
      zedModelData = ZEDModelData.ZED_2I;

      // Set initialization parameters based on camera model
      zedInitializationParameters.camera_fps(CAMERA_FPS);
      zedInitializationParameters.resolution(SL_RESOLUTION_HD720);
      zedInitializationParameters.input_type(SL_INPUT_TYPE_USB);
      zedInitializationParameters.camera_device_id(cameraID);
      zedInitializationParameters.camera_image_flip(SL_FLIP_MODE_OFF);
      zedInitializationParameters.camera_disable_self_calib(false);
      zedInitializationParameters.enable_image_enhancement(true);
      zedInitializationParameters.svo_real_time_mode(true);
      zedInitializationParameters.depth_mode(SL_DEPTH_MODE_ULTRA);
      zedInitializationParameters.depth_stabilization(1);
      zedInitializationParameters.depth_maximum_distance(zedModelData.getMaximumDepthDistance());
      zedInitializationParameters.depth_minimum_distance(zedModelData.getMinimumDepthDistance());
      zedInitializationParameters.coordinate_unit(SL_UNIT_METER);
      zedInitializationParameters.coordinate_system(SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD);
      zedInitializationParameters.sdk_gpu_id(-1); // Will find and use the best available GPU
      zedInitializationParameters.sdk_verbose(0); // false
      zedInitializationParameters.sensors_required(true);
      zedInitializationParameters.enable_right_side_measure(false);
      zedInitializationParameters.open_timeout_sec(5.0f);
      zedInitializationParameters.async_grab_camera_recovery(false);

      // Reopen camera with specific parameters set
      success = checkError("sl_open_camera", sl_open_camera(cameraID, zedInitializationParameters, 0, "", "", 0, "", "", ""));
      if (!success)
         return success;

      zedRuntimeParameters.enable_depth(true);
      zedRuntimeParameters.confidence_threshold(100);
      zedRuntimeParameters.reference_frame(SL_REFERENCE_FRAME_CAMERA);
      zedRuntimeParameters.texture_confidence_threshold(100);
      zedRuntimeParameters.remove_saturated_areas(true);
      zedRuntimeParameters.enable_fill_mode(false);

      // Get camera's parameters
      SL_CalibrationParameters zedCalibrationParameters = sl_get_calibration_parameters(cameraID, false);
      cameraFocalLengthX.put(RobotSide.LEFT, zedCalibrationParameters.left_cam().fx());
      cameraFocalLengthX.put(RobotSide.RIGHT, zedCalibrationParameters.right_cam().fx());
      cameraFocalLengthY.put(RobotSide.LEFT, zedCalibrationParameters.left_cam().fy());
      cameraFocalLengthY.put(RobotSide.RIGHT, zedCalibrationParameters.right_cam().fy());
      cameraPrincipalPointX.put(RobotSide.LEFT, zedCalibrationParameters.left_cam().cx());
      cameraPrincipalPointX.put(RobotSide.RIGHT, zedCalibrationParameters.right_cam().cx());
      cameraPrincipalPointY.put(RobotSide.LEFT, zedCalibrationParameters.left_cam().cy());
      cameraPrincipalPointY.put(RobotSide.RIGHT, zedCalibrationParameters.right_cam().cy());

      imageWidth = sl_get_width(cameraID);
      imageHeight = sl_get_height(cameraID);

      colorImagePointer = new Pointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U8_C4, SL_MEM_GPU));
      depthImagePointer = new Pointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U16_C1, SL_MEM_GPU));

      LogTools.info("Started {} camera", getCameraModel(cameraID));
      LogTools.info("Firmware version: {}", sl_get_camera_firmware(cameraID));
      LogTools.info("Image resolution: {} x {}", imageWidth, imageHeight);

      return success;
   }

   private boolean checkError(String functionName, int returnedState)
   {
      if (returnedState != SL_ERROR_CODE_SUCCESS)
      {
         LogTools.error(String.format("%s returned '%d'", functionName, returnedState));
      }
      return returnedState == SL_ERROR_CODE_SUCCESS;
   }

   private String getCameraModel(int cameraID)
   {
      return switch (sl_get_camera_model(cameraID))
      {
         case SL_MODEL_ZED -> "ZED";
         case SL_MODEL_ZED_M -> "ZED Mini";
         case SL_MODEL_ZED2 -> "ZED 2";
         case SL_MODEL_ZED2i -> "ZED 2i";
         case SL_MODEL_ZED_X -> "ZED X";
         case SL_MODEL_ZED_XM -> "ZED XM";
         default -> "Unknown model";
      };
   }

   private void setZEDConfiguration(int cameraID)
   {
      switch (sl_get_camera_model(cameraID))
      {
         case SL_MODEL_ZED -> zedModelData = ZEDModelData.ZED;
         case SL_MODEL_ZED_M -> zedModelData = ZEDModelData.ZED_MINI;
         case SL_MODEL_ZED2 -> zedModelData = ZEDModelData.ZED_2;
         case SL_MODEL_ZED2i -> zedModelData = ZEDModelData.ZED_2I;
         case SL_MODEL_ZED_X -> zedModelData = ZEDModelData.ZED_X;
         case SL_MODEL_ZED_XM -> zedModelData = ZEDModelData.ZED_X_MINI;
         default ->
         {
            zedModelData = ZEDModelData.ZED;
            LogTools.error("Failed to associate model number with a ZED sensor model");
         }
      }
   }
}
