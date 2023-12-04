package us.ihmc;

import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.colorVision.BlackflyImagePublisher;
import us.ihmc.avatar.colorVision.BlackflyImageRetriever;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.ROS2HeartbeatDependencyNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.ouster.OusterDepthImagePublisher;
import us.ihmc.perception.ouster.OusterDepthImageRetriever;
import us.ihmc.perception.ouster.OusterNetServer;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectionUpdater;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeDetectionManager;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.RealsenseColorDepthImagePublisher;
import us.ihmc.sensors.RealsenseColorDepthImageRetriever;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;
import us.ihmc.tools.thread.RestartableThread;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.util.function.Supplier;

/**
 * <p>
 *    This class holds all sensor drivers and publishers, as well as some algorithms which consume the images
 *    produced by the sensors. In general, each sensor has "Retriever" and "Publisher" classes. The retriever
 *    is responsible for grabbing images from the sensor and providing them to this class. hen, andy kind of
 *    processing can be performed on the images, which are then given to the publisher or any algorithm which
 *    requires the images.
 * </p>
 * <p>
 *    In general, the retrievers have a thread in which images are grabbed from the sensors.
 *    The publishers have threads which wait for new images to arrive, and once given an image they publish it
 *    and wait again.
 *    Within this class, each sensor has a thread which takes the most recent image from a retriever, performs
 *    any processing, and passes the image on to the publisher.
 * </p>
 * <p>
 *    To launch the Nadia specific sensor configuration, see: {@code NadiaPerceptionAndAutonomyProcess}
 *    To launch the process with no specific configuration, uncomment the call to {@code forceEnableAllSensors}
 *    in the {@code main} method. When launching only one sensor, comment out the other sensor heartbeats in the
 *    {@code forceEnableAllSensors} method.
 * </p>
 *
 * TODO: Add HumanoidPerceptionModule, add depth image overlap removal.
 */
public class PerceptionAndAutonomyProcess
{
   private static final int ZED_CAMERA_ID = 0;
   private static final SideDependentList<ROS2Topic<ImageMessage>> ZED_COLOR_TOPICS = PerceptionAPI.ZED2_COLOR_IMAGES;
   private static final ROS2Topic<ImageMessage> ZED_DEPTH_TOPIC = PerceptionAPI.ZED2_DEPTH;

   private static final String REALSENSE_SERIAL_NUMBER = System.getProperty("d455.serial.number", "215122253249");
   private static final ROS2Topic<ImageMessage> REALSENSE_COLOR_TOPIC = PerceptionAPI.D455_COLOR_IMAGE;
   private static final ROS2Topic<ImageMessage> REALSENSE_DEPTH_TOPIC = PerceptionAPI.D455_DEPTH_IMAGE;

   private static final ROS2Topic<ImageMessage> OUSTER_DEPTH_TOPIC = PerceptionAPI.OUSTER_DEPTH_IMAGE;

   private static final String LEFT_BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.left.serial.number", "00000000");
   private static final String RIGHT_BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.right.serial.number", "17403057");
   private static final BlackflyLensProperties BLACKFLY_LENS = BlackflyLensProperties.BFS_U3_27S5C_FE185C086HA_1;
   private static final ROS2Topic<ImageMessage> BLACKFLY_IMAGE_TOPIC = PerceptionAPI.BLACKFLY_FISHEYE_COLOR_IMAGE.get(RobotSide.RIGHT);

   private static final double SCENE_GRAPH_UPDATE_FREQUENCY = 10.0;

   private ROS2HeartbeatDependencyNode zedPointCloudNode;
   private ROS2HeartbeatDependencyNode zedColorNode;
   private ROS2HeartbeatDependencyNode zedDepthNode;
   private RawImage zedDepthImage;
   private final SideDependentList<RawImage> zedColorImages = new SideDependentList<>();
   private ZEDColorDepthImageRetriever zedImageRetriever;
   private ZEDColorDepthImagePublisher zedImagePublisher;
   private RestartableThread zedProcessAndPublishThread;

   private ROS2HeartbeatDependencyNode realsensePublishNode;
   private RawImage realsenseDepthImage;
   private RawImage realsenseColorImage;
   private RealsenseColorDepthImageRetriever realsenseImageRetriever;
   private RealsenseColorDepthImagePublisher realsenseImagePublisher;
   private RestartableThread realsenseProcessAndPublishThread;

   private ROS2HeartbeatDependencyNode ousterDepthNode;
   private ROS2HeartbeatDependencyNode ousterLidarScanNode;
   private ROS2HeartbeatDependencyNode ousterHeightMapNode;
   private OusterNetServer ouster;
   private RawImage ousterDepthImage;
   private OusterDepthImageRetriever ousterDepthImageRetriever;
   private OusterDepthImagePublisher ousterDepthImagePublisher;
   private RestartableThread ousterProcessAndPublishThread;

   private final SideDependentList<Supplier<ReferenceFrame>> blackflyFrameSuppliers = new SideDependentList<>();
   private final SideDependentList<ROS2HeartbeatDependencyNode> blackflyImageNodes = new SideDependentList<>();
   private final SideDependentList<RawImage> blackflyImages = new SideDependentList<>();
   private final SideDependentList<BlackflyImageRetriever> blackflyImageRetrievers = new SideDependentList<>();
   private final SideDependentList<BlackflyImagePublisher> blackflyImagePublishers = new SideDependentList<>();
   private RestartableThread blackflyProcessAndPublishThread;

   private final Supplier<ReferenceFrame> robotPelvisFrameSupplier;
   private final ROS2SceneGraph sceneGraph;
   private RestartableThrottledThread sceneGraphUpdateThread;
   private ROS2HeartbeatDependencyNode arUcoDetectionNode;
   private final ArUcoDetectionUpdater arUcoUpdater;

   private final CenterposeDetectionManager centerposeDetectionManager;
   private ROS2HeartbeatDependencyNode centerposeUpdateNode;

   // Sensor heartbeats to run main method without UI
   private ROS2Heartbeat zedHeartbeat;
   private ROS2Heartbeat realsenseHeartbeat;
   private ROS2Heartbeat ousterHeartbeat;
   private ROS2Heartbeat leftBlackflyHeartbeat;
   private ROS2Heartbeat rightBlackflyHeartbeat;

   public PerceptionAndAutonomyProcess(ROS2PublishSubscribeAPI ros2,
                                       Supplier<ReferenceFrame> zedFrameSupplier,
                                       Supplier<ReferenceFrame> realsenseFrameSupplier,
                                       Supplier<ReferenceFrame> ousterFrameSupplier,
                                       Supplier<ReferenceFrame> leftBlackflyFrameSupplier,
                                       Supplier<ReferenceFrame> rightBlackflyFrameSupplier,
                                       Supplier<ReferenceFrame> robotPelvisFrameSupplier,
                                       ReferenceFrame zed2iLeftCameraFrame)
   {
      initializeDependencyGraph(ros2);

      zedImageRetriever = new ZEDColorDepthImageRetriever(ZED_CAMERA_ID, zedFrameSupplier, zedPointCloudNode, zedDepthNode, zedColorNode);
      zedImagePublisher = new ZEDColorDepthImagePublisher(ZED_COLOR_TOPICS, ZED_DEPTH_TOPIC);
      zedProcessAndPublishThread = new RestartableThread("ZEDImageProcessAndPublish", this::processAndPublishZED);
      zedProcessAndPublishThread.start();

      realsenseImageRetriever = new RealsenseColorDepthImageRetriever(new RealsenseDeviceManager(),
                                                                      REALSENSE_SERIAL_NUMBER,
                                                                      RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                                      realsenseFrameSupplier,
                                                                      realsensePublishNode);
      realsenseImagePublisher = new RealsenseColorDepthImagePublisher(REALSENSE_DEPTH_TOPIC, REALSENSE_COLOR_TOPIC);
      realsenseProcessAndPublishThread = new RestartableThread("RealsenseProcessAndPublish", this::processAndPublishRealsense);
      realsenseProcessAndPublishThread.start();

      ouster = new OusterNetServer();
      ouster.start();
      ousterDepthImageRetriever = new OusterDepthImageRetriever(ouster,
                                                                ousterFrameSupplier,
                                                                ousterLidarScanNode::checkIfDesired,
                                                                ousterHeightMapNode::checkIfDesired,
                                                                ousterDepthNode);
      ousterDepthImagePublisher = new OusterDepthImagePublisher(ouster, OUSTER_DEPTH_TOPIC);
      ousterProcessAndPublishThread = new RestartableThread("OusterProcessAndPublish", this::processAndPublishOuster);
      ousterProcessAndPublishThread.start();

      blackflyFrameSuppliers.put(RobotSide.LEFT, leftBlackflyFrameSupplier);
      blackflyFrameSuppliers.put(RobotSide.RIGHT, rightBlackflyFrameSupplier);
      blackflyProcessAndPublishThread = new RestartableThread("BlackflyProcessAndPublish", this::processAndPublishBlackfly);
      blackflyProcessAndPublishThread.start();

      this.robotPelvisFrameSupplier = robotPelvisFrameSupplier;
      sceneGraph = new ROS2SceneGraph(ros2);
      sceneGraphUpdateThread = new RestartableThrottledThread("SceneGraphUpdater", SCENE_GRAPH_UPDATE_FREQUENCY, this::updateSceneGraph);

      arUcoUpdater = new ArUcoDetectionUpdater(sceneGraph, BLACKFLY_LENS, blackflyFrameSuppliers.get(RobotSide.RIGHT));

      centerposeDetectionManager = new CenterposeDetectionManager(ros2, zed2iLeftCameraFrame);

      sceneGraphUpdateThread.start(); // scene graph runs at all times
   }

   public void destroy()
   {
      LogTools.info("Destroying {}", this.getClass().getSimpleName());
      if (zedImageRetriever != null)
      {
         zedProcessAndPublishThread.stop();
         zedImagePublisher.destroy();
         zedImageRetriever.destroy();
      }

      if (realsenseImageRetriever != null)
      {
         realsenseProcessAndPublishThread.stop();
         realsenseImagePublisher.destroy();
         realsenseImageRetriever.destroy();
      }

      if (ouster != null)
      {
         System.out.println("Destroying OusterNetServer...");
         ouster.destroy();
         System.out.println("Destroyed OusterNetServer...");
         ousterProcessAndPublishThread.stop();
         ousterDepthImagePublisher.destroy();
         ousterDepthImageRetriever.destroy();
      }

      sceneGraphUpdateThread.stop();
      arUcoUpdater.stopArUcoDetection();

      if (blackflyProcessAndPublishThread != null)
         blackflyProcessAndPublishThread.stop();
      for (RobotSide side : RobotSide.values)
      {
         if (blackflyImagePublishers.get(side) != null)
            blackflyImagePublishers.get(side).destroy();
         if (blackflyImageRetrievers.get(side) != null)
            blackflyImageRetrievers.get(side).destroy();
      }


      zedPointCloudNode.destroy();
      zedColorNode.destroy();
      zedDepthNode.destroy();
      realsensePublishNode.destroy();
      ousterDepthNode.destroy();
      ousterLidarScanNode.destroy();
      blackflyImageNodes.forEach(ROS2HeartbeatDependencyNode::destroy);
      arUcoDetectionNode.destroy();
      centerposeUpdateNode.destroy();

      if (zedHeartbeat != null)
         zedHeartbeat.destroy();
      if (realsenseHeartbeat != null)
         realsenseHeartbeat.destroy();
      if (ousterHeartbeat != null)
         ousterHeartbeat.destroy();
      if (leftBlackflyHeartbeat != null)
         leftBlackflyHeartbeat.destroy();
      if (rightBlackflyHeartbeat != null)
         rightBlackflyHeartbeat.destroy();

      LogTools.info("Destroyed {}", this.getClass().getSimpleName());
   }

   private void processAndPublishZED()
   {
      if (zedDepthNode.checkIfDesired() || zedColorNode.checkIfDesired())
      {
         zedDepthImage = zedImageRetriever.getLatestRawDepthImage();
         for (RobotSide side : RobotSide.values)
         {
            zedColorImages.put(side, zedImageRetriever.getLatestRawColorImage(side));
         }

         // Do processing on image

         zedImagePublisher.setNextGpuDepthImage(zedDepthImage.get());
         for (RobotSide side : RobotSide.values)
         {
            zedImagePublisher.setNextColorImage(zedColorImages.get(side).get(), side);
         }

         zedDepthImage.release();
         zedColorImages.forEach(RawImage::release);
      }
      else
         ThreadTools.sleep(500);
   }

   private void processAndPublishRealsense()
   {
      if (realsensePublishNode.checkIfDesired())
      {
         realsenseDepthImage = realsenseImageRetriever.getLatestRawDepthImage();
         realsenseColorImage = realsenseImageRetriever.getLatestRawColorImage();

         // Do processing on image

         realsenseImagePublisher.setNextDepthImage(realsenseDepthImage.get());
         realsenseImagePublisher.setNextColorImage(realsenseColorImage.get());

         realsenseDepthImage.release();
         realsenseColorImage.release();
      }
      else
         ThreadTools.sleep(500);
   }

   private void processAndPublishOuster()
   {
      if (ousterDepthNode.checkIfDesired())
      {
         ouster.start();
         ousterDepthImage = ousterDepthImageRetriever.getLatestRawDepthImage();
         if (ousterDepthImage == null)
            return;

         ousterDepthImagePublisher.setNextDepthImage(ousterDepthImage.get());

         ousterDepthImage.release();
      }
      else
      {
         ouster.stop();
         ThreadTools.sleep(500);
      }
   }

   private void processAndPublishBlackfly()
   {
      for (RobotSide side : RobotSide.values)
      {
         if (blackflyImageNodes.get(side).checkIfDesired())
         {
            if (blackflyImageRetrievers.get(side) != null && blackflyImagePublishers.get(side) != null)
            {
               blackflyImages.put(side, blackflyImageRetrievers.get(side).getLatestRawImage());

               blackflyImagePublishers.get(side).setNextDistortedImage(blackflyImages.get(side).get());

               if (side == RobotSide.RIGHT && blackflyImages.get(RobotSide.RIGHT).getImageWidth() != 0)
                  arUcoUpdater.setNextArUcoImage(blackflyImages.get(RobotSide.RIGHT).get());

               blackflyImages.get(side).release();
            }
            else
            {
               initializeBlackfly(side);
            }
         }
      }
   }

   private void updateSceneGraph()
   {
      sceneGraph.updateSubscription();
      if (arUcoDetectionNode.checkIfDesired() && arUcoUpdater.isInitialized())
         arUcoUpdater.updateArUcoDetection();
      if (centerposeUpdateNode.checkIfDesired())
         centerposeDetectionManager.updateSceneGraph(sceneGraph);
      sceneGraph.updateOnRobotOnly(robotPelvisFrameSupplier.get());
      sceneGraph.updatePublication();
   }

   private void initializeBlackfly(RobotSide side)
   {
      String serialNumber = side == RobotSide.LEFT ? LEFT_BLACKFLY_SERIAL_NUMBER : RIGHT_BLACKFLY_SERIAL_NUMBER;
      if (serialNumber.equals("00000000"))
      {
         LogTools.error("{} blackfly with serial number was not provided", side.getPascalCaseName());
         ThreadTools.sleep(3000);
         return;
      }

      blackflyImageRetrievers.put(side,
                                  new BlackflyImageRetriever(serialNumber,
                                                             BLACKFLY_LENS,
                                                             RobotSide.RIGHT,
                                                             blackflyFrameSuppliers.get(side),
                                                             blackflyImageNodes.get(side)));
      blackflyImagePublishers.put(side, new BlackflyImagePublisher(BLACKFLY_LENS, BLACKFLY_IMAGE_TOPIC));
   }

   private void initializeDependencyGraph(ROS2PublishSubscribeAPI ros2)
   {
      // Initialize all nodes
      zedPointCloudNode = new ROS2HeartbeatDependencyNode(ros2, PerceptionAPI.REQUEST_ZED_POINT_CLOUD);
      zedDepthNode = new ROS2HeartbeatDependencyNode(ros2, PerceptionAPI.REQUEST_ZED_DEPTH);
      zedColorNode = new ROS2HeartbeatDependencyNode(ros2, PerceptionAPI.REQUEST_ZED_COLOR);

      realsensePublishNode = new ROS2HeartbeatDependencyNode(ros2, PerceptionAPI.REQUEST_REALSENSE_POINT_CLOUD);

      ousterDepthNode = new ROS2HeartbeatDependencyNode(ros2, PerceptionAPI.REQUEST_OUSTER_DEPTH);
      ousterHeightMapNode = new ROS2HeartbeatDependencyNode(ros2, PerceptionAPI.REQUEST_HEIGHT_MAP);
      ousterLidarScanNode = new ROS2HeartbeatDependencyNode(ros2, PerceptionAPI.REQUEST_LIDAR_SCAN);

      for (RobotSide side : RobotSide.values)
         blackflyImageNodes.put(side, new ROS2HeartbeatDependencyNode(ros2, PerceptionAPI.REQUEST_BLACKFLY_COLOR_IMAGE.get(side)));

      arUcoDetectionNode = new ROS2HeartbeatDependencyNode(ros2, PerceptionAPI.REQUEST_ARUCO);

      centerposeUpdateNode = new ROS2HeartbeatDependencyNode(ros2, PerceptionAPI.REQUEST_CENTERPOSE);

      // build the graph
      zedDepthNode.addDependants(zedPointCloudNode);
      zedColorNode.addDependants(zedPointCloudNode, centerposeUpdateNode);

      blackflyImageNodes.get(RobotSide.RIGHT).addDependants(arUcoDetectionNode);
   }

   /*
    * This method is used to enable sensors without needing to run the UI.
    * Unneeded sensors can be commented out.
    */
   private void forceEnableAllSensors(ROS2PublishSubscribeAPI ros2)
   {
      zedHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_ZED_POINT_CLOUD);
      zedHeartbeat.setAlive(true);

      realsenseHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_REALSENSE_POINT_CLOUD);
      realsenseHeartbeat.setAlive(true);

      ousterHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_OUSTER_DEPTH);
      ousterHeartbeat.setAlive(true);

      leftBlackflyHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_BLACKFLY_COLOR_IMAGE.get(RobotSide.LEFT));
      leftBlackflyHeartbeat.setAlive(true);

      rightBlackflyHeartbeat = new ROS2Heartbeat(ros2, PerceptionAPI.REQUEST_BLACKFLY_COLOR_IMAGE.get(RobotSide.RIGHT));
      rightBlackflyHeartbeat.setAlive(true);
   }

   public static void main(String[] args)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "perception_autonomy_process");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      PerceptionAndAutonomyProcess perceptionAndAutonomyProcess = new PerceptionAndAutonomyProcess(ros2Helper,
                                                                                                   ReferenceFrame::getWorldFrame,
                                                                                                   ReferenceFrame::getWorldFrame,
                                                                                                   ReferenceFrame::getWorldFrame,
                                                                                                   ReferenceFrame::getWorldFrame,
                                                                                                   ReferenceFrame::getWorldFrame,
                                                                                                   ReferenceFrame::getWorldFrame,
                                                                                                   ReferenceFrame.getWorldFrame());

      // To run a sensor without the UI, uncomment the below line.
      // perceptionAndAutonomyProcess.forceEnableAllSensors(ros2Helper);
      Runtime.getRuntime().addShutdownHook(new Thread(perceptionAndAutonomyProcess::destroy, "PerceptionAutonomyProcessShutdown"));
   }
}
