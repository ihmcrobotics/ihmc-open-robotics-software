package us.ihmc.sensors.zed;

import org.bytedeco.javacpp.IntPointer;
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
import us.ihmc.sensors.CameraIntrinsics;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.zed.SL_CalibrationParameters;
import us.ihmc.zed.SL_InitParameters;
import us.ihmc.zed.SL_PositionalTrackingParameters;
import us.ihmc.zed.SL_Quaternion;
import us.ihmc.zed.SL_RuntimeParameters;
import us.ihmc.zed.SL_SpatialMappingParameters;
import us.ihmc.zed.SL_Vector3;
import us.ihmc.zed.ZEDException;
import us.ihmc.zed.library.ZEDJavaAPINativeLibrary;

import java.time.Instant;
import java.util.concurrent.atomic.AtomicInteger;

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

   private static final int DEFAULT_RESOLUTION = SL_RESOLUTION_HD720;
   private static final int DEFAULT_FPS = 15;
   private static final float MILLIMETER_TO_METERS = 0.001f;

   private final int cameraID;
   private final int serialNumber;
   private final ZEDModelData zedModel;
   private final int slInputType;
   private final int slDepthMode;
   private final int resolution;
   private int fps;
   private int bitrate;
   private String remoteStreamingAddress;
   private int remoteStreamingPort;
   private final int localStreamingPort = nextStreamingPort.getAndAdd(2);

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
   private boolean spatialMappingEnabled = false;
   private final MutableReferenceFrame trackedSensorFrame;
   private final RigidBodyTransform trackedPoseOffset = new RigidBodyTransform();
   private final SL_Quaternion sensorRotation = new SL_Quaternion();
   private final SL_Vector3 sensorTranslation = new SL_Vector3();

   // Spatial mapping mesh data
   private float[] meshVertices = null;
   private int[] meshTriangles = null;
   private byte[] meshColors = null;
   private int meshNumVertices = 0;
   private int meshNumTriangles = 0;

   public ZEDImageSensor(int cameraID, ZEDModelData zedModel, int slInputType, int slDepthMode)
   {
      this(cameraID, zedModel, slInputType, slDepthMode, DEFAULT_RESOLUTION, DEFAULT_FPS);
   }

   public ZEDImageSensor(int cameraID, ZEDModelData zedModel, int slInputType, int slDepthMode, int resolution, int fps)
   {
      this(cameraID, 0, zedModel, slInputType, slDepthMode, resolution, fps);
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
   public ZEDImageSensor(int cameraID, int serialNumber, ZEDModelData zedModel, int slInputType, int slDepthMode, int resolution, int fps)
   {
      super(zedModel.name());

      this.cameraID = cameraID;
      this.serialNumber = serialNumber;
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
      zedRuntimeParameters.confidence_threshold(50);
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

   public void setTrackedPoseOffset(RigidBodyTransformReadOnly offset)
   {
      trackedPoseOffset.set(offset);
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
            positionalTrackingParameters.enable_area_memory(true);
            positionalTrackingParameters.enable_imu_fusion(true);
            positionalTrackingParameters.enable_pose_smoothing(false);
            positionalTrackingParameters.set_floor_as_origin(false);
            sl_enable_positional_tracking(cameraID, positionalTrackingParameters, "");
         }

         if (spatialMappingEnabled)
         {
            // Positional tracking is required for spatial mapping
            if (!positionalTrackingEnabled)
            {
               LogTools.warn("Spatial mapping requires positional tracking. Enabling positional tracking automatically.");
               SL_PositionalTrackingParameters positionalTrackingParameters = sl_get_positional_tracking_parameters(cameraID);
               positionalTrackingParameters.enable_area_memory(true);
               positionalTrackingParameters.enable_imu_fusion(true);
               sl_enable_positional_tracking(cameraID, positionalTrackingParameters, "");
               positionalTrackingEnabled = true;
            }

            SL_SpatialMappingParameters spatialMappingParameters = new SL_SpatialMappingParameters();
            spatialMappingParameters.map_type(SL_SPATIAL_MAP_TYPE_MESH); // Mesh, not point cloud
            spatialMappingParameters.resolution_meter(0.05f); // 5cm resolution
            spatialMappingParameters.range_meter(0.0f); // Automatic range detection
            spatialMappingParameters.save_texture(false); // Enable if RGB texture needed
            spatialMappingParameters.use_chunk_only(true); // Use chunked mesh format
            spatialMappingParameters.max_memory_usage(2048); // MB
            spatialMappingParameters.reverse_vertex_order(false);
            sl_enable_spatial_mapping(cameraID, spatialMappingParameters);
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
         return sl_open_camera(cameraID, zedInitParameters, serialNumber, "", remoteStreamingAddress, remoteStreamingPort, "", "", "");
      else
         return sl_open_camera(cameraID, zedInitParameters, serialNumber, "", "", 0, "", "", "");
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
         RigidBodyTransform leftSensorTransformAtGrab = leftSensorFrame.getTransformToWorldFrame();
         RigidBodyTransform rightSensorTransformAtGrab = rightSensorFrame.getTransformToWorldFrame();
         Instant grabTime = Instant.now();
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
         lastGrabTime = grabTime;
         ++grabSequenceNumber;

         lastGrabTimestamp = sl_get_current_timestamp(cameraID);

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

         if (spatialMappingEnabled)
         {
            // Monitor mapping state
            int mappingState = sl_get_spatial_mapping_state(cameraID);
            switch (mappingState)
            {
               case SL_SPATIAL_MAPPING_STATE_INITIALIZING ->
                  LogTools.debug("Spatial mapping initializing...");
               case SL_SPATIAL_MAPPING_STATE_OK -> { /* Normal operation */ }
               case SL_SPATIAL_MAPPING_STATE_NOT_ENOUGH_MEMORY ->
                  LogTools.warn("Spatial mapping: Not enough memory");
               case SL_SPATIAL_MAPPING_STATE_NOT_ENABLED ->
                  LogTools.warn("Spatial mapping not enabled");
               case SL_SPATIAL_MAPPING_STATE_FPS_TOO_LOW ->
                  LogTools.warn("Spatial mapping: FPS too low");
            }

            // Extract and update mesh data periodically
            if (grabSequenceNumber % 30 == 0) // Update mesh every 30 frames
            {
               updateMeshData();
            }
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

   public void enableSpatialMapping(boolean enable)
   {
      spatialMappingEnabled = enable;
   }

   /**
    * Updates the mesh data from the spatial mapping system.
    * Extracts vertices, triangles, and colors from the ZED spatial map.
    */
   private void updateMeshData()
   {
      // Use IntPointers for output parameters to ensure we read C-side writes
      try (IntPointer nbVerticesPerSubmesh = new IntPointer(1000);
           IntPointer nbTrianglesPerSubmesh = new IntPointer(1000);
           IntPointer nbSubmeshes = new IntPointer(1);
           IntPointer updatedIndices = new IntPointer(1000);
           IntPointer nbVerticesTot = new IntPointer(1);
           IntPointer nbTrianglesTot = new IntPointer(1))
      {
         int maxSubmesh = 1000;

         int mappingState = sl_get_spatial_mapping_state(cameraID);
         LogTools.debug("Spatial mapping state: {}", mappingState);
         switch (mappingState)
         {
            case SL_SPATIAL_MAPPING_STATE_INITIALIZING:
               LogTools.debug("Spatial mapping initializing...");
               break;
            case SL_SPATIAL_MAPPING_STATE_OK:
               LogTools.debug("Spatial mapping OK");
               break;
            case SL_SPATIAL_MAPPING_STATE_NOT_ENOUGH_MEMORY:
               LogTools.debug("Not enough memory for spatial mapping");
               return;
            case SL_SPATIAL_MAPPING_STATE_NOT_ENABLED:
               LogTools.debug("Spatial mapping not enabled");
               return;
            case SL_SPATIAL_MAPPING_STATE_FPS_TOO_LOW:
               LogTools.debug("FPS too low for spatial mapping");
               return;
         }

         sl_request_mesh_async(cameraID);

         // Check if mesh is ready
         int meshReadyStatus = sl_get_mesh_request_status_async(cameraID);
         if (meshReadyStatus != SL_ERROR_CODE_SUCCESS)
         {
            LogTools.debug("Mesh not ready yet: {}", getZEDErrorName(meshReadyStatus));
            return;
         }

         // Call update_mesh with Pointers
         int updateResult = sl_update_mesh(cameraID,
                                           nbVerticesPerSubmesh,
                                           nbTrianglesPerSubmesh,
                                           nbSubmeshes,
                                           updatedIndices,
                                           nbVerticesTot,
                                           nbTrianglesTot,
                                           maxSubmesh);

         // Check result code
         if (updateResult != SL_ERROR_CODE_SUCCESS)
         {
            LogTools.warn("sl_update_mesh failed with error: {} ({})",
                          getZEDErrorName(updateResult), updateResult);
            return;
         }

         // Read values from Pointers
         int numVertices = nbVerticesTot.get();
         int numTriangles = nbTrianglesTot.get();
         if (numVertices > 0 && numTriangles > 0)
         {
            // Allocate arrays for mesh data
            float[] vertices = new float[numVertices * 3];
            int[] triangles = new int[numTriangles * 3];
            byte[] colors = new byte[numVertices * 4];
            float[] uvs = null;
            byte[] texturePtr = null;

            // Retrieve using the calculated sizes
            int retrieveResult = sl_retrieve_mesh(cameraID, vertices, triangles, colors, uvs, texturePtr, maxSubmesh);

            if (retrieveResult == SL_ERROR_CODE_SUCCESS)
            {
               synchronized (this)
               {
                  meshVertices = vertices;
                  meshTriangles = triangles;
                  meshColors = colors;
                  meshNumVertices = numVertices;
                  meshNumTriangles = numTriangles;
               }
            }
            else
            {
               LogTools.error("Failed to retrieve mesh: " + getZEDErrorName(retrieveResult));
            }
         }
      }
      catch (Exception e)
      {
         LogTools.error("Error updating mesh data: {}", e.getMessage());
      }
   }

   /**
    * Saves the current spatial mesh to a file.
    * @param filePath Path to save the mesh (supports .obj, .ply, .bin formats)
    */
   public void saveMesh(String filePath)
   {
      if (!spatialMappingEnabled)
      {
         LogTools.warn("Spatial mapping is not enabled. Cannot save mesh.");
         return;
      }

      sl_extract_whole_spatial_map(cameraID);
      boolean result = sl_save_mesh(cameraID, filePath, SL_MESH_FILE_FORMAT_OBJ);

      if (result)
         LogTools.info("Mesh saved to: {}", filePath);
      else
         LogTools.error("Failed to save mesh to: {}", filePath);
   }

   /**
    * @return Current mesh vertices as float array [x1,y1,z1, x2,y2,z2, ...], or null if no mesh available
    */
   public synchronized float[] getMeshVertices()
   {
      return meshVertices;
   }

   /**
    * @return Current mesh triangles as int array [v1,v2,v3, v1,v2,v3, ...], or null if no mesh available
    */
   public synchronized int[] getMeshTriangles()
   {
      return meshTriangles;
   }

   /**
    * @return Current mesh colors as byte array [r,g,b,a, r,g,b,a, ...], or null if no mesh available
    */
   public synchronized byte[] getMeshColors()
   {
      return meshColors;
   }

   /**
    * @return Number of vertices in the current mesh
    */
   public synchronized int getMeshNumVertices()
   {
      return meshNumVertices;
   }

   /**
    * @return Number of triangles in the current mesh
    */
   public synchronized int getMeshNumTriangles()
   {
      return meshNumTriangles;
   }

   @Override
   public void close()
   {
      if (spatialMappingEnabled)
      {
         sl_disable_spatial_mapping(cameraID);
      }

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

      synchronized (grabbedImages)
      {
         for (RawImage image : grabbedImages)
            if (image != null)
               image.release();
      }

      sl_close_camera(cameraID);

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
