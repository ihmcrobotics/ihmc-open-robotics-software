package us.ihmc.sensors;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.zed.SL_CalibrationParameters;
import org.bytedeco.zed.SL_InitParameters;
import org.bytedeco.zed.SL_RuntimeParameters;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.time.Instant;
import java.util.concurrent.atomic.AtomicReference;

import static org.bytedeco.zed.global.zed.*;

/**
 * Encodes and publishes color and depth images from a ZED sensor.
 * The depth image is aligned to the left camera of the ZED.
 */
public class ZEDColorDepthImageRetriever
{
   private static final int CAMERA_FPS = 30;
   private static final float MILLIMETER_TO_METERS = 0.001f;

   private final int cameraID;
   private ZEDModelData zedModelData;
   private final SL_RuntimeParameters zedRuntimeParameters = new SL_RuntimeParameters();

   private final int imageWidth; // Width of rectified image in pixels (color image width == depth image width)
   private final int imageHeight; // Height of rectified image in pixels (color image height ==  depth image height)
   private final SideDependentList<Float> cameraFocalLengthX;
   private final SideDependentList<Float> cameraFocalLengthY;
   private final SideDependentList<Float> cameraPrincipalPointX;
   private final SideDependentList<Float> cameraPrincipalPointY;

   private final Pointer colorImagePointer;
   private final Pointer depthImagePointer;
   private long grabSequenceNumber = 0L;
   private final SideDependentList<Long> colorImageSequenceNumber = new SideDependentList<>(0L, 0L);

   private final AtomicReference<Instant> colorImageAcquisitionTime = new AtomicReference<>();
   private final AtomicReference<Instant> depthImageAcquisitionTime = new AtomicReference<>();

   private GpuMat depthGpuMat;
   private final SideDependentList<GpuMat> colorGpuMats = new SideDependentList<>();
   // TODO: remove atomic reference?
   private final SideDependentList<AtomicReference<RawImage>> atomicColorImages = new SideDependentList<>(new AtomicReference<>(null),
                                                                                                          new AtomicReference<>(null));
   private final AtomicReference<RawImage> atomicDepthImage = new AtomicReference<>(null);

   private final Thread grabImageThread;
   boolean running = true;

   public ZEDColorDepthImageRetriever(int cameraID)
   {
      this.cameraID = cameraID;

      LogTools.info("ZED SDK version: " + sl_get_sdk_version().getString());

      // Create and initialize the camera
      sl_create_camera(cameraID);

      SL_InitParameters zedInitializationParameters = new SL_InitParameters();

      // Open camera with default parameters to find model
      // Can't get the model number without opening the camera first
      checkError("sl_open_camera", sl_open_camera(cameraID, zedInitializationParameters, 0, "", "", 0, "", "", ""));
      setZEDConfiguration(cameraID);
      sl_close_camera(cameraID);

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
      checkError("sl_open_camera", sl_open_camera(cameraID, zedInitializationParameters, 0, "", "", 0, "", "", ""));

      zedRuntimeParameters.enable_depth(true);
      zedRuntimeParameters.confidence_threshold(100);
      zedRuntimeParameters.reference_frame(SL_REFERENCE_FRAME_CAMERA);
      zedRuntimeParameters.texture_confidence_threshold(100);
      zedRuntimeParameters.remove_saturated_areas(true);
      zedRuntimeParameters.enable_fill_mode(false);

      // Get camera's parameters
      SL_CalibrationParameters zedCalibrationParameters = sl_get_calibration_parameters(cameraID, false);
      cameraFocalLengthX = new SideDependentList<>(zedCalibrationParameters.left_cam().fx(), zedCalibrationParameters.right_cam().fx());
      cameraFocalLengthY = new SideDependentList<>(zedCalibrationParameters.left_cam().fy(), zedCalibrationParameters.right_cam().fy());
      cameraPrincipalPointX = new SideDependentList<>(zedCalibrationParameters.left_cam().cx(), zedCalibrationParameters.right_cam().cx());
      cameraPrincipalPointY = new SideDependentList<>(zedCalibrationParameters.left_cam().cy(), zedCalibrationParameters.right_cam().cy());
      imageWidth = sl_get_width(cameraID);
      imageHeight = sl_get_height(cameraID);

      colorImagePointer = new Pointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U8_C4, SL_MEM_GPU));
      depthImagePointer = new Pointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U16_C1, SL_MEM_GPU));

      depthGpuMat = new GpuMat(imageHeight, imageWidth, opencv_core.CV_16UC1);
      colorGpuMats.put(RobotSide.LEFT, new GpuMat(imageHeight, imageWidth, opencv_core.CV_8UC3));
      colorGpuMats.put(RobotSide.RIGHT, new GpuMat(imageHeight, imageWidth, opencv_core.CV_8UC3));

      grabImageThread = new Thread(() ->
      {
         while (running)
         {
            // sl_grab is blocking, and will wait for next frame available. No need to use throttler
            checkError("sl_grab", sl_grab(cameraID, zedRuntimeParameters));
            retrieveAndSaveDepthImage();
            retrieveAndSaveColorImage(RobotSide.LEFT);
            retrieveAndSaveColorImage(RobotSide.RIGHT);
         }
      }, "ZEDGrabImageThread");

      LogTools.info("Starting {} camera", getCameraModel(cameraID));
      LogTools.info("Firmware version: {}", sl_get_camera_firmware(cameraID));
      LogTools.info("Image resolution: {} x {}", imageWidth, imageHeight);
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

      colorGpuMats.get(side).close();
      colorGpuMats.put(side, new GpuMat(imageHeight, imageWidth, opencv_core.CV_8UC3));
      opencv_cudaimgproc.cvtColor(colorImageBGRA, colorGpuMats.get(side), opencv_imgproc.COLOR_BGRA2BGR);

      atomicColorImages.get(side)
                       .set(new RawImage(colorImageSequenceNumber.get(side),
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
                                      cameraPrincipalPointY.get(side)));

      colorImageSequenceNumber.set(side, colorImageSequenceNumber.get(side) + 1);

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

      depthGpuMat.close();
      depthGpuMat = new GpuMat(imageHeight,
                               imageWidth,
                               opencv_core.CV_16UC1,
                               sl_mat_get_ptr(depthImagePointer, SL_MEM_GPU),
                               sl_mat_get_step_bytes(depthImagePointer, SL_MEM_GPU));

      atomicDepthImage.set(new RawImage(grabSequenceNumber++,
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
                                        cameraPrincipalPointY.get(RobotSide.LEFT)));
   }

   public RawImage getLatestRawDepthImage()
   {
      return atomicDepthImage.get();
   }

   public RawImage getLatestRawColorImage(RobotSide side)
   {
      return atomicColorImages.get(side).get();
   }

   public ZEDModelData getZedModelData()
   {
      return zedModelData;
   }

   public void start()
   {
      grabImageThread.start();
   }

   public void destroy()
   {
      running = false;
      try
      {
         grabImageThread.join();
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }

      depthImagePointer.close();
      atomicDepthImage.get().destroy();

      colorImagePointer.close();
      for (RobotSide side : RobotSide.values)
      {
         atomicColorImages.get(side).get().destroy();
      }
      sl_close_camera(cameraID);
   }

   private void checkError(String functionName, int returnedState)
   {
      if (returnedState != SL_ERROR_CODE_SUCCESS)
      {
         LogTools.error(String.format("%s returned '%d'", functionName, returnedState));
      }
   }

   private String getCameraModel(int cameraID)
   {
      switch (sl_get_camera_model(cameraID))
      {
         case 0: return "ZED";
         case 1: return "ZED Mini";
         case 2: return "ZED 2";
         case 3: return "ZED 2i";
         case 4: return "ZED X";
         case 5: return "ZED XM";
         default: return "Unknown model";
      }
   }

   private void setZEDConfiguration(int cameraID)
   {
      switch (sl_get_camera_model(cameraID))
      {
         case 0 -> zedModelData = ZEDModelData.ZED;
         case 1 -> zedModelData = ZEDModelData.ZED_MINI;
         case 2 -> zedModelData = ZEDModelData.ZED_2;
         case 3 -> zedModelData = ZEDModelData.ZED_2I;
         case 4 -> zedModelData = ZEDModelData.ZED_X;
         case 5 -> zedModelData = ZEDModelData.ZED_X_MINI;
         default ->
         {
            zedModelData = ZEDModelData.ZED;
            LogTools.error("Failed to associate model number with a ZED sensor model");
         }
      }
   }
}
