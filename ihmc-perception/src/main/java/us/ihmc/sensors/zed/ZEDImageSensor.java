package us.ihmc.sensors.zed;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.sensors.CameraIntrinsics;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.TransformCsvLogger;
import us.ihmc.zed.SL_CalibrationParameters;
import us.ihmc.zed.SL_InitParameters;
import us.ihmc.zed.SL_PositionalTrackingParameters;
import us.ihmc.zed.SL_Quaternion;
import us.ihmc.zed.SL_RuntimeParameters;
import us.ihmc.zed.SL_Vector3;
import us.ihmc.zed.ZEDException;
import us.ihmc.zed.library.ZEDJavaAPINativeLibrary;

import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.zed.ZEDTools.throwOnError;
import static us.ihmc.zed.global.zed.*;

public class ZEDImageSensor extends ImageSensor
{
   static
   {
      ZEDJavaAPINativeLibrary.load();
   }

   private static final AtomicInteger nextStreamingPort = new AtomicInteger(30000);

   public static final int LEFT_COLOR_IMAGE_KEY = 0;
   public static final int RIGHT_COLOR_IMAGE_KEY = 1;
   public static final int DEPTH_IMAGE_KEY = 2;

   public static final int OUTPUT_IMAGE_COUNT = 3;

   private static final float MILLIMETER_TO_METERS = 0.001f;

   private final int cameraID;

   // sl_open_camera parameters
   private final SL_InitParameters zedInitParameters = new SL_InitParameters();
   private int serialNumber = 0;
   private String svoFilePath = "";
   private String remoteStreamingAddress = "";
   private int remoteStreamingPort = 0;

   // sl_enable_streaming related
   private int bitrate;
   private final int localStreamingPort = nextStreamingPort.getAndAdd(2);
   private int fps;

   private final SL_RuntimeParameters zedRuntimeParameters = new SL_RuntimeParameters();

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

   /** Local wall-clock time of the last successful grab, used only for {@link #isSensorRunning()}'s liveness check. */
   private Instant lastGrabReceivedTime;
   private boolean lastGrabFailed = false;
   private long lastGrabTimestamp;

   private boolean positionalTrackingEnabled = false;
   private final MutableReferenceFrame trackedSensorFrame;
   private final RigidBodyTransform trackedPoseOffset = new RigidBodyTransform();
   private final SL_Quaternion sensorRotation = new SL_Quaternion();
   private final SL_Vector3 sensorTranslation = new SL_Vector3();

   private final CUstream_st cudaStream;

   /**
    * Debug aid: when set (via {@link #enablePoseDebugLogging}), dumps every grabbed frame's sensor pose to a CSV,
    * to check offline whether the frame-to-robot sync is smooth (see {@code plot_zed_pose_debug.py}).
    */
   private final AtomicReference<TransformCsvLogger> poseDebugLogger = new AtomicReference<>();

   /**
    * Optional monitor to synchronize {@code sensorFrame}'s transform-to-world reads against, set via
    * {@link #setFrameTreeLock}. See that method's doc for why this exists.
    */
   private Object frameTreeLock;

   /**
    * The most basic constructor that sets parameters to some default value.
    *
    * @param cameraID    ID assigned to this camera when opening.
    * @param zedModel    Model of the ZED camera.
    * @param slInputType One of {@code SL_INPUT_TYPE_*} from {@link us.ihmc.zed.global.zed}.
    */
   public ZEDImageSensor(int cameraID, ZEDModelData zedModel, int slInputType)
   {
      super(zedModel.name());

      this.cameraID = cameraID;

      trackedSensorFrame = new MutableReferenceFrame(getSensorName() + "_tracked", ReferenceFrameTools.getWorldFrame());

      cudaStream = CUDAStreamManager.getStream();

      sensorCenterToCameraDistanceY = (float) zedModel.getCenterToCameraDistance();
      updateReferenceFrames();

      // Set runtime parameters
      zedRuntimeParameters.reference_frame(SL_REFERENCE_FRAME_CAMERA);
      zedRuntimeParameters.enable_depth(true);
      zedRuntimeParameters.enable_fill_mode(false);
      zedRuntimeParameters.confidence_threshold(50);
      zedRuntimeParameters.texture_confidence_threshold(100);
      zedRuntimeParameters.remove_saturated_areas(true);

      // Set init parameters
      zedInitParameters.camera_device_id(cameraID);
      zedInitParameters.input_type(slInputType);
      zedInitParameters.resolution(SL_RESOLUTION_AUTO);
      zedInitParameters.camera_fps(30);
      zedInitParameters.depth_mode(SL_DEPTH_MODE_NEURAL);
      zedInitParameters.depth_stabilization(1);
      zedInitParameters.coordinate_unit(SL_UNIT_METER);
      zedInitParameters.coordinate_system(SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD);
      zedInitParameters.sdk_verbose(0); // false
      zedInitParameters.camera_disable_self_calib(false);
      zedInitParameters.camera_image_flip(SL_FLIP_MODE_OFF);
      zedInitParameters.enable_right_side_measure(false);
      zedInitParameters.sensors_required(true);
      zedInitParameters.enable_image_enhancement(true);
      zedInitParameters.open_timeout_sec(5.0f);
      zedInitParameters.async_grab_camera_recovery(false);
      zedInitParameters.enable_image_validity_check(false);
   }

   /**
    * Constructor used to open a camera via physical connection.
    * <p>
    * See the documentation for the available resolutions and frame rates:
    * <ul>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_HD4K}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_QHDPLUS}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_HD2K}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_HD1536}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_HD1080}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_HD1200}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_HD720}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_SVGA}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_VGA}</li>
    *    <li>{@link us.ihmc.zed.global.zed#SL_RESOLUTION_AUTO}</li>
    * </ul>
    *
    * @param cameraID     ID assigned to this camera when opening.
    * @param serialNumber Serial number of camera to open.
    * @param zedModel     Model of the ZED camera.
    * @param slInputType  Either {@link us.ihmc.zed.global.zed#SL_INPUT_TYPE_USB} or {@link us.ihmc.zed.global.zed#SL_INPUT_TYPE_GMSL}.
    * @param slDepthMode  One of {@code SL_DEPTH_MODE_*} from {@link us.ihmc.zed.global.zed}.
    * @param resolution   One of {@code SL_RESOLUTION_*} from {@link us.ihmc.zed.global.zed}.
    * @param fps          Frame rate to run the camera at,
    */
   public ZEDImageSensor(int cameraID, int serialNumber, ZEDModelData zedModel, int slInputType, int slDepthMode, int resolution, int fps)
   {
      this(cameraID, zedModel, slInputType);

      this.serialNumber = serialNumber;
      this.fps = fps;

      // Set some more runtime and init parameters
      zedRuntimeParameters.enable_depth(slDepthMode != SL_DEPTH_MODE_NONE);
      zedInitParameters.resolution(resolution);
      zedInitParameters.camera_fps(fps);
      zedInitParameters.depth_mode(slDepthMode);
   }

   /**
    * Constructor to connect to a remote ZED SDK stream.
    *
    * @param cameraID               ID assigned to this camera when opening.
    * @param zedModel               Model of the ZED camera.
    * @param slDepthMode            One of {@code SL_DEPTH_MODE_*} from {@link us.ihmc.zed.global.zed}.
    * @param remoteStreamingAddress Address of the remote ZED SDK stream.
    * @param remoteStreamingPort    Port of the remote ZED SDK stream.
    */
   public ZEDImageSensor(int cameraID, ZEDModelData zedModel, int slDepthMode, String remoteStreamingAddress, int remoteStreamingPort)
   {
      this(cameraID, zedModel, SL_INPUT_TYPE_STREAM);

      this.remoteStreamingAddress = remoteStreamingAddress;
      this.remoteStreamingPort = remoteStreamingPort;

      // Set some more runtime and init parameters
      zedRuntimeParameters.enable_depth(slDepthMode != SL_DEPTH_MODE_NONE);
      zedInitParameters.depth_mode(slDepthMode);
   }

   protected ZEDImageSensor(int cameraID, ZEDModelData zedModel, int slDepthMode, String svoFilePath)
   {
      this(cameraID, zedModel, SL_INPUT_TYPE_SVO);

      if (!Files.exists(Path.of(svoFilePath)))
         throw new RuntimeException("SVO file does not exist");

      this.svoFilePath = svoFilePath;

      // Set some more runtime and init parameters
      zedRuntimeParameters.enable_depth(slDepthMode != SL_DEPTH_MODE_NONE);
      zedInitParameters.depth_mode(slDepthMode);
      zedInitParameters.svo_real_time_mode(true);
   }

   public void setTrackedPoseOffset(RigidBodyTransformReadOnly offset)
   {
      trackedPoseOffset.set(offset);
   }

   /**
    * If {@code sensorFrame} (see {@link #setSensorFrame}) is backed by a robot model that's mutated from another
    * thread - e.g. a {@code ROS2SyncedRobotModel} whose {@code update()} is applied on its own thread as new robot
    * state messages arrive - that update thread can race the grab thread's {@code getTransformToWorldFrame()} read
    * of the same frame tree, since {@code ReferenceFrame}'s transform cache isn't otherwise thread-safe. Pass the
    * same object that update is synchronized on (e.g. the {@code ROS2SyncedRobotModel} instance itself) here, and
    * the grab thread will synchronize its frame reads on it too. Leave unset (default {@code null}) if
    * {@code sensorFrame} isn't concurrently mutated by anything else, e.g. a fixed gizmo frame in a demo.
    */
   public void setFrameTreeLock(Object frameTreeLock)
   {
      this.frameTreeLock = frameTreeLock;
   }

   /**
    * Starts logging every grabbed frame's sensor pose to a CSV at {@code outputFile}, for debugging frame-sync
    * smoothness offline. Overwrites {@code outputFile} if it exists. Call {@link #disablePoseDebugLogging()} when
    * done to close the file.
    */
   public void enablePoseDebugLogging(Path outputFile)
   {
      TransformCsvLogger newLogger = new TransformCsvLogger(outputFile, "grabSequenceNumber", "imageAgeNanos");
      TransformCsvLogger oldLogger = poseDebugLogger.getAndSet(newLogger);
      if (oldLogger != null)
         oldLogger.close();
   }

   public void disablePoseDebugLogging()
   {
      TransformCsvLogger oldLogger = poseDebugLogger.getAndSet(null);
      if (oldLogger != null)
         oldLogger.close();
   }

   public SL_InitParameters getInitParameters()
   {
      return zedInitParameters;
   }

   public SL_RuntimeParameters getRuntimeParameters()
   {
      return zedRuntimeParameters;
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

         if (zedInitParameters.depth_mode() == SL_DEPTH_MODE_NEURAL || zedInitParameters.depth_mode() == SL_DEPTH_MODE_NEURAL_PLUS)
            LogTools.info("ZED SDK will use neural depth mode. This uses significant GPU resources.");

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

         if (zedInitParameters.input_type() != SL_INPUT_TYPE_STREAM)
         {
            int defaultGOPSize = -1;
            int disableAdaptiveBitrate = 0;
            int chunkSize = 16084;
            sl_enable_streaming(cameraID, SL_STREAMING_CODEC_H265, bitrate, (short) localStreamingPort, defaultGOPSize, disableAdaptiveBitrate, chunkSize, fps);
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

   private static long instantToEpochNanos(Instant instant)
   {
      return instant.getEpochSecond() * 1_000_000_000L + instant.getNano();
   }

   protected int openCamera()
   {
      return sl_open_camera(cameraID, zedInitParameters, serialNumber, svoFilePath, remoteStreamingAddress, remoteStreamingPort, -1, "", "", "");
   }

   @Override
   public boolean isSensorRunning()
   {
      boolean recentlyGrabbed = lastGrabReceivedTime != null && lastGrabReceivedTime.isAfter(Instant.now().minusSeconds(1));
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
               trackedSensorFrame.update(transform -> transform.set(trackedPoseOffset));
            }

            return false;
         }

         throwOnError(returnCode);

         // ReferenceFrame's transform-to-root cache is a plain, unsynchronized mutable field, recomputed in place
         // on read. sensorFrame (and its ancestors up to world) can be mutated from another thread - e.g. a
         // ROS2SyncedRobotModel applying newly-received robot state on its own update thread - so reading it here
         // unguarded can return a torn, transiently wrong transform. See setFrameTreeLock().
         RigidBodyTransform leftSensorTransformAtGrab;
         RigidBodyTransform rightSensorTransformAtGrab;
         if (frameTreeLock != null)
         {
            synchronized (frameTreeLock)
            {
               leftSensorTransformAtGrab = leftSensorFrame.getTransformToWorldFrame();
               rightSensorTransformAtGrab = rightSensorFrame.getTransformToWorldFrame();
            }
         }
         else
         {
            leftSensorTransformAtGrab = leftSensorFrame.getTransformToWorldFrame();
            rightSensorTransformAtGrab = rightSensorFrame.getTransformToWorldFrame();
         }

         // Don't assume the ZED SDK's clock domain lines up with Instant.now() (SL_TIMESTAMP_CLOCK_SYSTEM_CLOCK
         // doesn't reliably produce Unix-epoch nanoseconds on all hardware/SDK builds). Instead, use
         // sl_get_current_timestamp() the way it's documented to be used: diffed against sl_get_image_timestamp()
         // to get the frame's age in nanoseconds, on the ZED SDK's own clock. That age is then subtracted from a
         // local Instant.now() taken right alongside it, translating the real capture time into our local clock
         // domain, without needing the two clocks' epochs to agree.
         long imageTimestampZed = sl_get_image_timestamp(cameraID);
         long currentTimestampZed = sl_get_current_timestamp(cameraID);
         Instant localNow = Instant.now();

         long imageAgeNanos = Math.max(0L, currentTimestampZed - imageTimestampZed);
         Instant grabTime = localNow.minusNanos(imageAgeNanos);
         lastGrabTimestamp = instantToEpochNanos(grabTime);

         lastGrabTime = grabTime;
         lastGrabReceivedTime = Instant.now();
         ++grabSequenceNumber;

         TransformCsvLogger logger = poseDebugLogger.get();
         if (logger != null)
            logger.log("grab", lastGrabTimestamp, leftSensorTransformAtGrab, grabSequenceNumber, imageAgeNanos);

         // Update tracked position if tracking enabled
         if (positionalTrackingEnabled)
         {
            sl_get_position(cameraID, sensorRotation, sensorTranslation, SL_REFERENCE_FRAME_WORLD);

            Quaternion euclidRotation = new Quaternion(sensorRotation.x(), sensorRotation.y(), sensorRotation.z(), sensorRotation.w());
            Vector3D euclidTranslation = new Vector3D(sensorTranslation.x(), sensorTranslation.y(), sensorTranslation.z());
            if (!euclidRotation.containsNaN() && !euclidTranslation.containsNaN())
               trackedSensorFrame.update(transformToWorld ->
                                         {
                                            transformToWorld.set(euclidRotation, euclidTranslation);
                                            transformToWorld.prependOrientation(trackedPoseOffset.getRotation());
                                            transformToWorld.prependTranslation(trackedPoseOffset.getTranslation());
                                         });
         }

         // Retrieve the grabbed depth image
         Pointer depthImagePointer = slMatPointers[DEPTH_IMAGE_KEY];
         returnCode = sl_retrieve_measure(cameraID, depthImagePointer, SL_MEASURE_DEPTH_U16_MM, SL_MEM_GPU, imageWidth, imageHeight, cudaStream);
         throwOnError(returnCode);

         // Retrieve the grabbed left color image
         Pointer leftColorImagePointer = slMatPointers[LEFT_COLOR_IMAGE_KEY];
         returnCode = sl_retrieve_image(cameraID, leftColorImagePointer, SL_VIEW_LEFT, SL_MEM_GPU, imageWidth, imageHeight, cudaStream);
         throwOnError(returnCode);

         // Retrieve the grabbed right color image
         Pointer rightColorImagePointer = slMatPointers[RIGHT_COLOR_IMAGE_KEY];
         returnCode = sl_retrieve_image(cameraID, rightColorImagePointer, SL_VIEW_RIGHT, SL_MEM_GPU, imageWidth, imageHeight, cudaStream);
         throwOnError(returnCode);

         synchronized (grabbedImages)
         {  // Create RawImages from the grabbed retrieved slMats
            if (grabbedImages[LEFT_COLOR_IMAGE_KEY] != null)
               grabbedImages[LEFT_COLOR_IMAGE_KEY].release();
            grabbedImages[LEFT_COLOR_IMAGE_KEY] = slMatToRawImage(leftColorImagePointer, PixelFormat.BGRA8, leftSensorIntrinsics, leftSensorTransformAtGrab);

            if (grabbedImages[RIGHT_COLOR_IMAGE_KEY] != null)
               grabbedImages[RIGHT_COLOR_IMAGE_KEY].release();
            grabbedImages[RIGHT_COLOR_IMAGE_KEY] = slMatToRawImage(rightColorImagePointer,
                                                                   PixelFormat.BGRA8,
                                                                   rightSensorIntrinsics,
                                                                   rightSensorTransformAtGrab);

            if (grabbedImages[DEPTH_IMAGE_KEY] != null)
               grabbedImages[DEPTH_IMAGE_KEY].release();
            grabbedImages[DEPTH_IMAGE_KEY] = slMatToRawImage(depthImagePointer, PixelFormat.GRAY16, leftSensorIntrinsics, leftSensorTransformAtGrab);
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

      disablePoseDebugLogging();

      for (Pointer slMat : slMatPointers)
      {
         if (slMat != null && !slMat.isNull())
         {
            sl_mat_free(slMat, SL_MEM_GPU);
            slMat.close();
         }
      }

      synchronized (grabbedImages)
      {
         for (RawImage image : grabbedImages)
            if (image != null)
               image.release();
      }

      sl_close_camera(cameraID);

      CUDAStreamManager.releaseStream(cudaStream);

      System.out.println("Closed " + getClass().getSimpleName());
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
