package us.ihmc.perception.headless;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.*;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;
import java.util.function.Supplier;

/**
 * TerrainPerceptionProcessWithDriver is a headless process that runs the perception frontend for terrain-specific measurements such as planar regions.
 * color, depth, and point cloud data using the depth data obtained from the terrain sensor on the robot. (L515 currently). This class may be extended
 * in the future to support height map extraction, iterative-closest point based registration, LidarScanMessage publisher, and more.
 * <p>
 * Primary responsibilities include (but are not limited to):
 * 1. Loads depth data from the sensor.
 * 2. Loads color data from the sensor.
 * 3. Extracts planar regions from the depth data.
 * 4. Publishes color images on the color topic
 * 5. Publishes depth images on the depth topic
 * 6. Publishes planar regions on the planar regions topic
 */
public class TerrainPerceptionProcessWithDriver
{
   private final Notification destroyedNotification = new Notification();
   private final BytePointer compressedColorPointer = new BytePointer();
   private final BytePointer compressedDepthPointer = new BytePointer();
   private final FramePose3D colorPoseInDepthFrame = new FramePose3D();
   private final ImageMessage depthImageMessage = new ImageMessage();
   private final ImageMessage colorImageMessage = new ImageMessage();
   private final FramePose3D cameraPose = new FramePose3D();
   private final Throttler throttler = new Throttler();

   private final PerceptionConfigurationParameters parameters = new PerceptionConfigurationParameters();
   private final ROS2Topic<FramePlanarRegionsListMessage> frameRegionsTopic;
   private final ROS2Topic<PlanarRegionsListMessage> regionsTopic;
   private final Supplier<ReferenceFrame> sensorFrameUpdater;
   private final ROS2Topic<ImageMessage> colorTopic;
   private final ROS2Topic<ImageMessage> depthTopic;

   private final RapidPlanarRegionsExtractor rapidRegionsExtractor;
   private final OpenCLManager openCLManager;
   private final ROS2Helper ros2Helper;
   private final double outputPeriod;

   private RealSenseHardwareManager realSenseHardwareManager;
   private ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private RealsenseConfiguration realsenseConfiguration;
   private BytedecoImage debugExtractionImage;
   private BytedecoImage depthBytedecoImage;
   private _cl_program openCLProgram;
   private BytedecoRealsense sensor;
   private Mat depth16UC1Image;
   private Mat color8UC3Image;
   private Mat yuvColorImage;

   private volatile boolean running = true;

   private String serialNumber;
   private int depthWidth;
   private int depthHeight;
   private int colorWidth;
   private int colorHeight;
   private long depthSequenceNumber = 0;
   private long colorSequenceNumber = 0;

   public TerrainPerceptionProcessWithDriver(String serialNumber,
                                             RealsenseConfiguration realsenseConfiguration,
                                             ROS2Topic<ImageMessage> depthTopic,
                                             ROS2Topic<ImageMessage> colorTopic,
                                             ROS2Topic<FramePlanarRegionsListMessage> frameRegionsTopic,
                                             ROS2Topic<PlanarRegionsListMessage> regionsTopic,
                                             Supplier<ReferenceFrame> sensorFrameUpdater)
   {
      this.serialNumber = serialNumber;
      this.realsenseConfiguration = realsenseConfiguration;
      this.depthTopic = depthTopic;
      this.colorTopic = colorTopic;
      this.frameRegionsTopic = frameRegionsTopic;
      this.sensorFrameUpdater = sensorFrameUpdater;
      this.regionsTopic = regionsTopic;

      outputPeriod = UnitConversions.hertzToSeconds(20.0);

      BytedecoTools.loadOpenCV();

      openCLManager = new OpenCLManager();
      rapidRegionsExtractor = new RapidPlanarRegionsExtractor();

      realSenseHardwareManager = new RealSenseHardwareManager();
      sensor = realSenseHardwareManager.createBytedecoRealsenseDevice(serialNumber, realsenseConfiguration);
      if (sensor.getDevice() == null)
      {
         destroy();
         throw new RuntimeException("Realsense device not found. Set -D<model>.serial.number=00000000000");
      }
      sensor.enableColor(realsenseConfiguration);
      sensor.initialize();

      depthWidth = sensor.getDepthWidth();
      depthHeight = sensor.getDepthHeight();
      colorWidth = sensor.getColorWidth();
      colorHeight = sensor.getColorHeight();

      LogTools.info("Depth width: " + depthWidth + ", height: " + depthHeight);
      LogTools.info("Color width: " + colorWidth + ", height: " + colorHeight);

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "realsense_color_and_depth_publisher");
      ros2Helper = new ROS2Helper(ros2Node);

      LogTools.info(String.format("Sensor Fx: %.2f, Sensor Fy: %.2f, Sensor Cx: %.2f, Sensor Cy: %.2f", sensor.getDepthFocalLengthPixelsX(),
                    sensor.getDepthFocalLengthPixelsY(), sensor.getDepthPrincipalOffsetXPixels(), sensor.getDepthPrincipalOffsetYPixels()));

      openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
   }

   /**
    * Must be called from the sensor-specific calling class, after the sensor and logger initialization have succeeded.
    * We run in a daemon thread, because otherwise it will get killed on Ctrl+C before the shutdown hooks are finished running.
    * See {@link Runtime#addShutdownHook(Thread)} for details.
    */
   public void run()
   {
      ThreadTools.startAsDaemon(this::updateThread, getClass().getSimpleName() + "UpdateThread");
   }

   private void updateThread()
   {
      while (running)
      {
         update();
         throttler.waitAndRun(outputPeriod); // do the waiting after we send to remove unnecessary latency
      }

      // Make sure the Realsense
      ThreadTools.sleep(100);

      if (sensor != null)
         sensor.deleteDevice();
      realSenseHardwareManager.deleteContext();

      destroyedNotification.set();
   }

   /**
    * Update the sensor and logger, and publish the data to ROS2
    */
   private void update()
   {
      if (sensor.readFrameData())
      {
         sensor.updateDataBytePointers();

         Instant acquisitionTime = Instant.now();

         MutableBytePointer depthFrameData = sensor.getDepthFrameData();
         MutableBytePointer colorFrameData = sensor.getColorFrameData();

         if (depth16UC1Image == null)
         {
            LogTools.info(String.format("Creating images with dimensions: depth: %d x %d, color: %d x %d",
                                        sensor.getDepthHeight(),
                                        sensor.getDepthWidth(),
                                        sensor.getColorHeight(),
                                        sensor.getColorWidth()));

            depth16UC1Image = new Mat(sensor.getDepthHeight(), sensor.getDepthWidth(), opencv_core.CV_16UC1, depthFrameData);
            color8UC3Image = new Mat(sensor.getColorHeight(), sensor.getColorWidth(), opencv_core.CV_8UC3, colorFrameData);
            depthBytedecoImage = new BytedecoImage(sensor.getDepthWidth(), sensor.getDepthHeight(), opencv_core.CV_16UC1);
            // YUV I420 has 1.5 times the height of the image
            // YUV image must be preallocated or there will be a memory leak
            yuvColorImage = new Mat(sensor.getColorHeight() * 1.5, sensor.getColorWidth(), opencv_core.CV_8UC1);

            rapidRegionsExtractor.create(openCLManager,
                                         openCLProgram,
                                         depthHeight,
                                         depthWidth,
                                         sensor.getDepthFocalLengthPixelsX(),
                                         sensor.getDepthFocalLengthPixelsY(),
                                         sensor.getDepthPrincipalOffsetXPixels(),
                                         sensor.getDepthPrincipalOffsetYPixels());

            LogTools.info("Setting Up ROS2 Property Set Group");
            ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
            ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS, parameters);
            ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_RAPID_REGION_PARAMETERS, rapidRegionsExtractor.getParameters());
            ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_POLYGONIZER_PARAMETERS,
                                                           rapidRegionsExtractor.getRapidPlanarRegionsCustomizer().getPolygonizerParameters());
            ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_CONVEX_HULL_FACTORY_PARAMETERS,
                                                           rapidRegionsExtractor.getRapidPlanarRegionsCustomizer().getConcaveHullFactoryParameters());
         }
         else
         {
            depth16UC1Image.data(depthFrameData);
            color8UC3Image.data(colorFrameData);
         }

         // Important not to store as a field, as update() needs to be called each frame
         ReferenceFrame cameraFrame = sensorFrameUpdater.get();
         cameraPose.setToZero(cameraFrame);
         cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

         colorPoseInDepthFrame.set(sensor.getDepthToColorTranslation(), sensor.getDepthToColorRotation());

         BytedecoOpenCVTools.compressImagePNG(depth16UC1Image, compressedDepthPointer);
         if (parameters.getPublishColor())
            BytedecoOpenCVTools.compressRGBImageJPG(color8UC3Image, yuvColorImage, compressedColorPointer);

         if (parameters.getPublishDepth())
         {
            PerceptionMessageTools.setDepthIntrinsicsFromRealsense(sensor, depthImageMessage);
            CameraModel.PINHOLE.packMessageFormat(depthImageMessage);
            PerceptionMessageTools.publishCompressedDepthImage(compressedDepthPointer,
                                                               depthTopic,
                                                               depthImageMessage,
                                                               ros2Helper,
                                                               cameraPose,
                                                               acquisitionTime,
                                                               depthSequenceNumber++,
                                                               sensor.getDepthHeight(),
                                                               sensor.getDepthWidth(),
                                                               (float) sensor.getDepthDiscretization());
         }

         if (parameters.getPublishColor())
         {
            PerceptionMessageTools.setColorIntrinsicsFromRealsense(sensor, colorImageMessage);
            CameraModel.PINHOLE.packMessageFormat(colorImageMessage);
            PerceptionMessageTools.publishJPGCompressedColorImage(compressedColorPointer,
                                                                  colorTopic,
                                                                  colorImageMessage,
                                                                  ros2Helper,
                                                                  colorPoseInDepthFrame,
                                                                  acquisitionTime,
                                                                  colorSequenceNumber++,
                                                                  sensor.getColorHeight(),
                                                                  sensor.getColorWidth(),
                                                                  (float) sensor.getDepthDiscretization());
         }

         if (parameters.getRapidRegionsEnabled())
         {
            depth16UC1Image.convertTo(depthBytedecoImage.getBytedecoOpenCVMat(), opencv_core.CV_16UC1, 1, 0);
            FramePlanarRegionsList framePlanarRegionsList = new FramePlanarRegionsList();
            extractFramePlanarRegionsList(depthBytedecoImage, cameraFrame, framePlanarRegionsList);
            PerceptionMessageTools.publishPlanarRegionsList(framePlanarRegionsList.getPlanarRegionsList(), regionsTopic, ros2Helper);

            LogTools.info("Planar regions found: {}", framePlanarRegionsList.getPlanarRegionsList().getNumberOfPlanarRegions());
         }

         ros2PropertySetGroup.update();
      }
   }

   private void onPatchSizeResized()
   {
      int patchImageWidth = rapidRegionsExtractor.getPatchImageWidth();
      int patchImageHeight = rapidRegionsExtractor.getPatchImageHeight();
      debugExtractionImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_8UC4);
   }

   private void extractFramePlanarRegionsList(BytedecoImage depthImage, ReferenceFrame cameraFrame, FramePlanarRegionsList framePlanarRegionsList)
   {
      rapidRegionsExtractor.update(depthImage, cameraFrame, framePlanarRegionsList);
      rapidRegionsExtractor.setProcessing(false);
   }

   /**
    * Must be called in the shutdown hook from the sensor-specific calling class. Handles Ctrl + C based closing gracefully.
    */
   public void destroy()
   {
      running = false;
      rapidRegionsExtractor.destroy();
      depthBytedecoImage.destroy(openCLManager);
      openCLManager.destroy();
      destroyedNotification.blockingPoll();
   }

   public static void main(String[] args)
   {
      /*
         Color: [fx:901.3026, fy:901.8400, cx:635.2337, cy:350.9427, h:720, w:1280]
         Depth: [fx:730.7891, fy:731.0859, cx:528.6094, cy:408.1602, h:768, w:1024]
      */

      String l515SerialNumber = System.getProperty("l515.serial.number", "F1121365"); // Benchtop L515: F1120592, Tripod: F1121365, Local: F0245563
      new TerrainPerceptionProcessWithDriver(l515SerialNumber,
                                             RealsenseConfiguration.L515_COLOR_720P_DEPTH_768P_30HZ,
                                             ROS2Tools.L515_DEPTH_IMAGE,
                                             ROS2Tools.L515_COLOR_IMAGE,
                                             ROS2Tools.PERSPECTIVE_RAPID_REGIONS_WITH_POSE,
                                             ROS2Tools.PERSPECTIVE_RAPID_REGIONS,
                                             ReferenceFrame::getWorldFrame);
   }
}
