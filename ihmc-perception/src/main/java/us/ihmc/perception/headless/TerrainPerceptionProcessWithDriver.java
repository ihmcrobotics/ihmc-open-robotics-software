package us.ihmc.perception.headless;

import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.*;
import us.ihmc.perception.comms.PerceptionComms;
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
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.time.Instant;
import java.util.function.Supplier;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;

/**
 * TerrainPerceptionProcessWithDriver is a headless process that runs the perception frontend for terrain-specific measurements such as planar regions.
 * color, depth, and point cloud data using the depth data obtained from the terrain sensor on the robot. (L515 currently). This class may be extended
 * in the future to support height map extraction, iterative-closest point based registration, LidarScanMessage publisher, and more.
 *
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
   private final PausablePeriodicThread thread;
   private final Activator nativesLoadedActivator;
   private final RealtimeROS2Node realtimeROS2Node;

   private BytedecoImage debugExtractionImage;
   private final ImageMessage depthImageMessage = new ImageMessage();
   private final ImageMessage colorImageMessage = new ImageMessage();
   private final ROS2Helper ros2Helper;

   private ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense sensor;
   private Mat depthU16C1Image;
   private Mat color8UC3Image;
   private Mat yuvColorImage = new Mat();
   private BytedecoImage depthBytedecoImage;
   private final RapidPlanarRegionsExtractor rapidRegionsExtractor;

   private final OpenCLManager openCLManager;
   private _cl_program openCLProgram;

   private RealsenseConfiguration realsenseConfiguration;
   private final Supplier<ReferenceFrame> sensorFrameUpdater;
   private final FramePose3D cameraPose = new FramePose3D();
   private final ROS2Topic<ImageMessage> colorTopic;
   private final ROS2Topic<ImageMessage> depthTopic;
   private final ROS2Topic<FramePlanarRegionsListMessage> frameRegionsTopic;
   private final ROS2Topic<PlanarRegionsListMessage> regionsTopic;

   private String serialNumber;
   private int depthWidth;
   private int depthHeight;
   private int colorWidth;
   private int colorHeight;
   private long depthSequenceNumber = 0;
   private long colorSequenceNumber = 0;

   public TerrainPerceptionProcessWithDriver(String serialNumber, RealsenseConfiguration realsenseConfiguration,
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

      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_videopub");
      realtimeROS2Node.spin();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_node");
      ros2Helper = new ROS2Helper(ros2Node);

      openCLManager = new OpenCLManager();
      rapidRegionsExtractor = new RapidPlanarRegionsExtractor();

      thread = new PausablePeriodicThread("L515Node", UnitConversions.hertzToSeconds(31.0), 1, false, this::update);
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "L515Shutdown"));
      LogTools.info("Starting loop.");
      thread.start();
   }

   private void update()
   {
      if (nativesLoadedActivator.poll())
      {
         if (nativesLoadedActivator.isNewlyActivated())
         {
            LogTools.info("Natives loaded.");

            realSenseHardwareManager = new RealSenseHardwareManager();
            sensor = realSenseHardwareManager.createBytedecoRealsenseDevice(serialNumber,
                                                                            realsenseConfiguration.getDepthWidth(),
                                                                            realsenseConfiguration.getDepthHeight(),
                                                                            realsenseConfiguration.getDepthFPS());

            if (sensor.getDevice() == null)
            {
               thread.stop();
               throw new RuntimeException("Device not found. Set -Dl515.serial.number=F0000000");
            }
            sensor.enableColor(realsenseConfiguration.getColorWidth(), realsenseConfiguration.getColorHeight(), realsenseConfiguration.getColorFPS());
            sensor.initialize();

            depthWidth = sensor.getDepthWidth();
            depthHeight = sensor.getDepthHeight();
            colorWidth = sensor.getColorWidth();
            colorHeight = sensor.getColorHeight();

            LogTools.info("Depth width: " + depthWidth + ", height: " + depthHeight);
            LogTools.info("Color width: " + colorWidth + ", height: " + colorHeight);
         }

         if (sensor.readFrameData())
         {

            Instant now = Instant.now();
            double dataAquisitionTime = Conversions.nanosecondsToSeconds(System.nanoTime());

            sensor.updateDataBytePointers();

            if (depthU16C1Image == null)
            {
               LogTools.info("Is now reading frames.");

               MutableBytePointer depthFrameData = sensor.getDepthFrameData();
               MutableBytePointer colorFrameData = sensor.getColorFrameData();

               depthU16C1Image = new Mat(depthHeight, depthWidth, opencv_core.CV_16UC1, depthFrameData);
               depthBytedecoImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
               color8UC3Image = new Mat(colorHeight, colorWidth, opencv_core.CV_8UC3, colorFrameData);

               PerceptionMessageTools.setDepthIntrinsicsFromRealsense(sensor, depthImageMessage);
               PerceptionMessageTools.setColorIntrinsicsFromRealsense(sensor, colorImageMessage);

               // Important not to store as a field, as update() needs to be called each frame
               ReferenceFrame cameraFrame = sensorFrameUpdater.get();
               cameraPose.setToZero(cameraFrame);
               cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

               MessageTools.toMessage(now, depthImageMessage.getAcquisitionTime());
               MessageTools.toMessage(now, colorImageMessage.getAcquisitionTime());

               openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");
               rapidRegionsExtractor.create(openCLManager,
                                            openCLProgram,
                                            depthHeight,
                                            depthWidth,
                                            sensor.getDepthFocalLengthPixelsX(),
                                            sensor.getDepthFocalLengthPixelsY(),
                                            sensor.getDepthPrincipalOffsetXPixels(),
                                            sensor.getDepthPrincipalOffsetYPixels());

               ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
               ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_RAPID_REGION_PARAMETERS, rapidRegionsExtractor.getParameters());
               ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_POLYGONIZER_PARAMETERS,
                                                              rapidRegionsExtractor.getRapidPlanarRegionsCustomizer().getPolygonizerParameters());
               ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_CONVEX_HULL_FACTORY_PARAMETERS,
                                                              rapidRegionsExtractor.getRapidPlanarRegionsCustomizer().getConcaveHullFactoryParameters());

               onPatchSizeResized();

               LogTools.info("Initialized.");
            }

            //            LogTools.info("New Iteration: {}", dataAquisitionTime);
            ros2PropertySetGroup.update();

            depthU16C1Image.convertTo(depthBytedecoImage.getBytedecoOpenCVMat(), opencv_core.CV_16UC1, 1, 0);

            FramePlanarRegionsList framePlanarRegionsList = new FramePlanarRegionsList();
            extractFramePlanarRegionsList(depthBytedecoImage, ReferenceFrame.getWorldFrame(), framePlanarRegionsList);
            PlanarRegionsList planarRegionsList = framePlanarRegionsList.getPlanarRegionsList();

            //            LogTools.info("Planar regions: {}", planarRegionsList.getNumberOfPlanarRegions());

            // TODO:  Filter out regions that are colliding with the body before publishing
            PerceptionMessageTools.publishPlanarRegionsList(planarRegionsList, regionsTopic, ros2Helper);
            PerceptionMessageTools.compressAndPublishDepthImagePNG(depthU16C1Image,
                                                                   depthTopic,
                                                                   depthImageMessage,
                                                                   ros2Helper,
                                                                   cameraPose,
                                                                   now,
                                                                   depthSequenceNumber,
                                                                   depthHeight,
                                                                   depthWidth);
            PerceptionMessageTools.publishJPGCompressedColorImage(color8UC3Image,
                                                           yuvColorImage,
                                                           colorTopic,
                                                           colorImageMessage,
                                                           ros2Helper,
                                                           cameraPose,
                                                           now,
                                                           colorSequenceNumber,
                                                           colorHeight,
                                                           colorWidth);

            //            display(depthU16C1Image, color8UC3Image, 1);
         }
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

   private void destroy()
   {
      thread.destroy();
      realtimeROS2Node.destroy();
      rapidRegionsExtractor.destroy();
      sensor.deleteDevice();
      depthBytedecoImage.destroy(openCLManager);
      openCLManager.destroy();
      realSenseHardwareManager.deleteContext();
   }

   private void display(Mat depth, Mat color, int delay)
   {
      Mat depthDisplay = new Mat();
      BytedecoOpenCVTools.clampTo8BitUnsignedChar(depth, depthDisplay, 0.0, 255.0);
      BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(depthDisplay, depthDisplay);

      imshow("Depth", depthDisplay);
      imshow("Color", color);
      int code = waitKeyEx(delay);
      if (code == 113)
      {
         thread.stop();
         System.exit(0);
      }
   }

   public static void main(String[] args)
   {
      /*
         Color: [fx:901.3026, fy:901.8400, cx:635.2337, cy:350.9427, h:720, w:1280]
         Depth: [fx:730.7891, fy:731.0859, cx:528.6094, cy:408.1602, h:768, w:1024]
      */

      String l515SerialNumber = System.getProperty("l515.serial.number", "F1121365"); // Benchtop L515: F1120592, Tripod: F1121365, Local: F0245563
      new TerrainPerceptionProcessWithDriver(l515SerialNumber, RealsenseConfiguration.L515_COLOR_720P_DEPTH_768P_30HZ,
                                             ROS2Tools.L515_DEPTH_IMAGE,
                                             ROS2Tools.L515_COLOR_IMAGE,
                                             ROS2Tools.PERSPECTIVE_RAPID_REGIONS_WITH_POSE,
                                             ROS2Tools.PERSPECTIVE_RAPID_REGIONS,
                                             ReferenceFrame::getWorldFrame);
   }
}
