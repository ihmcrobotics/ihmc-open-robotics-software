package us.ihmc;

import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.colorVision.BlackflyImagePublisher;
import us.ihmc.avatar.colorVision.BlackflyImageRetriever;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2HeartbeatMonitor;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.ouster.NettyOuster;
import us.ihmc.perception.ouster.OusterDepthImagePublisher;
import us.ihmc.perception.ouster.OusterDepthImageRetriever;
import us.ihmc.perception.realsense.RealsenseDevice;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.spinnaker.SpinnakerBlackflyManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.RealsenseColorDepthImagePublisher;
import us.ihmc.sensors.RealsenseColorDepthImageRetriever;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.util.function.Supplier;

public class PerceptionAndAutonomyProcess
{
   private static final int ZED_CAMERA_ID = 0;
   private static final double ZED_FPS = 30.0;
   private static final SideDependentList<ROS2Topic<ImageMessage>> ZED_COLOR_TOPICS = PerceptionAPI.ZED2_COLOR_IMAGES;
   private static final ROS2Topic<ImageMessage> ZED_DEPTH_TOPIC = PerceptionAPI.ZED2_DEPTH;

   private static final String REALSENSE_SERIAL_NUMBER = System.getProperty("d455.serial.number", "215122253249");
   private static final double REALSENSE_FPS = 20.0;
   private static final ROS2Topic<ImageMessage> REALSENSE_COLOR_TOPIC = PerceptionAPI.D455_COLOR_IMAGE;
   private static final ROS2Topic<ImageMessage> REALSENSE_DEPTH_TOPIC = PerceptionAPI.D455_DEPTH_IMAGE;

   private static final double OUSTER_FPS = 10.0;
   private static final ROS2Topic<ImageMessage> OUSTER_DEPTH_TOPIC = PerceptionAPI.OUSTER_DEPTH_IMAGE;

   private static final String LEFT_BLACKFLY_SERIAL_NUMBER = "17403057";
   private static final double BLACKFLY_FPS = 30.0;
   private static final BlackflyLensProperties BLACKFLY_LENS = BlackflyLensProperties.BFS_U3_27S5C_FE185C086HA_1;
   private static final ROS2Topic<ImageMessage> BLACKFLY_IMAGE_TOPIC = PerceptionAPI.BLACKFLY_FISHEYE_COLOR_IMAGE.get(RobotSide.RIGHT);

   private final Supplier<ReferenceFrame> zedFrameSupplier;
   private final ROS2HeartbeatMonitor zedPointCloudHeartbeat;
//   private final ROS2HeartbeatMonitor zedColorHeartbeat;
//   private final ROS2HeartbeatMonitor zedDepthHeartbeat;
   private RawImage zedDepthImage;
   private final SideDependentList<RawImage> zedColorImages = new SideDependentList<>();
   private ZEDColorDepthImageRetriever zedImageRetriever;
   private ZEDColorDepthImagePublisher zedImagePublisher;
   private RestartableThrottledThread zedProcessAndPublishThread;

   private final Supplier<ReferenceFrame> realsenseFrameSupplier;
   private final ROS2HeartbeatMonitor realsenseHeartbeat;
   private RealsenseDeviceManager realSenseManager;
   private RawImage realsenseDepthImage;
   private RawImage realsenseColorImage;
   private RealsenseColorDepthImageRetriever realsenseImageRetriever;
   private RealsenseColorDepthImagePublisher realsenseImagePublisher;
   private RestartableThrottledThread realsenseProcessAndPublishThread;

   private final Supplier<ReferenceFrame> ousterFrameSupplier;
   private final ROS2HeartbeatMonitor ousterDepthHeartbeat;
   private final ROS2HeartbeatMonitor lidarScanHeartbeat;
   private final ROS2HeartbeatMonitor heightMapHeartbeat;
   private NettyOuster ouster;
   private RawImage ousterDepthImage;
   private OusterDepthImageRetriever ousterDepthImageRetriever;
   private OusterDepthImagePublisher ousterDepthImagePublisher;
   private RestartableThrottledThread ousterProcessAndPublishThread;

   private final Supplier<ReferenceFrame> blackflyFrameSupplier;
   private final ROS2HeartbeatMonitor blackflyImageHeartbeat;
//   private final ROS2HeartbeatMonitor arUcoDetectionHeartbeat;
   private SpinnakerBlackflyManager blackflyManager;
   private RawImage blackflyImage;
   private BlackflyImageRetriever blackflyImageRetriever;
   private BlackflyImagePublisher blackflyImagePublisher;
   private RestartableThrottledThread blackflyProcessAndPublishThread;

   // temporary solution to keep process running when no sensors start on beginning
   private RestartableThrottledThread stayAliveThread;

   PerceptionAndAutonomyProcess(ROS2PublishSubscribeAPI ros2,
                                Supplier<ReferenceFrame> zedFrameSupplier,
                                Supplier<ReferenceFrame> realsenseFrameSupplier,
                                Supplier<ReferenceFrame> ousterFrameSupplier,
                                Supplier<ReferenceFrame> blackflyFrameSupplier)
   {
      this.zedFrameSupplier = zedFrameSupplier;
      zedPointCloudHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_ZED_POINT_CLOUD);
//      zedColorHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_ZED_COLOR);
//      zedDepthHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_ZED_DEPTH);
      initializeZEDHeartbeatCallbacks();

      this.realsenseFrameSupplier = realsenseFrameSupplier;
      realsenseHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_REALSENSE_POINT_CLOUD);
      initializeRealsenseHearbeatCallbacks();

      this.ousterFrameSupplier = ousterFrameSupplier;
      ousterDepthHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_OUSTER_DEPTH);
      lidarScanHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_LIDAR_SCAN);
      heightMapHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_HEIGHT_MAP);
      initializeOusterHeartbeatCallbacks();

      this.blackflyFrameSupplier = blackflyFrameSupplier;
      blackflyImageHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_BLACKFLY_COLOR_IMAGE.get(RobotSide.RIGHT));
//      arUcoDetectionHeartbeat = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.PUBLISH_ARUCO);
      initializeBlackflyHeartbeatCallbacks();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "PerceptionAndAutonomyShutdown"));
   }

   public void start()
   {
      if (zedPointCloudHeartbeat.isAlive())
      {
         initializeZED();
         zedImageRetriever.start();
         zedImagePublisher.startAll();
      }

      if (realsenseHeartbeat.isAlive())
      {
         initializeRealsense();
         realsenseImageRetriever.start();
         realsenseImagePublisher.startAll();
      }

      if (ousterDepthHeartbeat.isAlive())
      {
         initializeOuster();
         ousterDepthImageRetriever.start();
         ousterDepthImagePublisher.startDepth();
      }

      if (blackflyImageHeartbeat.isAlive())
      {
         initializeBlackfly();
         blackflyImageRetriever.start();
         blackflyImagePublisher.startAll();
      }

      stayAliveThread = new RestartableThrottledThread("KeepingPerceptionAlive", 1, () ->
      {
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException exception)
         {
            exception.printStackTrace();
         }
      });
      stayAliveThread.start();
   }

   public void destroy()
   {
      stayAliveThread.stop();

      if (zedImageRetriever != null)
      {
         zedProcessAndPublishThread.stop();
         zedImagePublisher.destroy();
         zedImageRetriever.destroy();
      }

      if (realSenseManager != null)
      {
         realsenseProcessAndPublishThread.stop();
         realsenseImagePublisher.destroy();
         realsenseImageRetriever.destroy();
      }

      if (ouster != null)
      {
         ouster.destroy();
         ousterProcessAndPublishThread.stop();
         ousterDepthImagePublisher.destroy();
         ousterDepthImageRetriever.destroy();
      }

      if (blackflyManager != null)
      {
         blackflyProcessAndPublishThread.stop();
         blackflyImagePublisher.destroy();
         blackflyImageRetriever.destroy();
         blackflyManager.destroy();
      }
   }

   private void processAndPublishZED()
   {
      zedDepthImage = zedImageRetriever.getLatestRawDepthImage();
      for (RobotSide side : RobotSide.values)
      {
         zedColorImages.put(side, zedImageRetriever.getLatestRawColorImage(side));
      }

      // Do processing on image

      zedImagePublisher.setNextGpuDepthImage(zedDepthImage);
      for (RobotSide side : RobotSide.values)
      {
         zedImagePublisher.setNextColorImage(zedColorImages.get(side), side);
      }
   }

   private void processAndPublishRealsense()
   {
      realsenseDepthImage = realsenseImageRetriever.getLatestRawDepthImage();
      realsenseColorImage = realsenseImageRetriever.getLatestRawColorImage();

      // Do processing on image

      realsenseImagePublisher.setNextDepthImage(realsenseDepthImage);
      realsenseImagePublisher.setNextColorImage(realsenseColorImage);
   }

   private void processAndPublishOuster()
   {
      ousterDepthImage = ousterDepthImageRetriever.getLatestRawDepthImage();

      ousterDepthImagePublisher.setNextDepthImage(ousterDepthImage);
   }

   private void processAndPublishBlackfly()
   {
      blackflyImage = blackflyImageRetriever.getLatestRawImage();

      blackflyImagePublisher.setNextDistortedImage(blackflyImage);
   }

   private void initializeZED()
   {
      zedImageRetriever = new ZEDColorDepthImageRetriever(ZED_CAMERA_ID, zedFrameSupplier);
      zedImagePublisher = new ZEDColorDepthImagePublisher(ZED_COLOR_TOPICS, ZED_DEPTH_TOPIC);
      zedProcessAndPublishThread = new RestartableThrottledThread("ZEDImageProcessAndPublish", ZED_FPS, this::processAndPublishZED);
      zedProcessAndPublishThread.start();
   }

   private void initializeZEDHeartbeatCallbacks()
   {
      zedPointCloudHeartbeat.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            if (zedImageRetriever == null)
               initializeZED();
            zedImageRetriever.start();
            zedImagePublisher.startAll();
         }
         else
         {
            zedImageRetriever.stop();
            zedImagePublisher.stopAll();
         }
      });
   }

   private void initializeRealsense()
   {
      realSenseManager = new RealsenseDeviceManager();
      RealsenseDevice realsense = realSenseManager.createBytedecoRealsenseDevice(REALSENSE_SERIAL_NUMBER,
                                                                                 RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ);
      realsenseImageRetriever = new RealsenseColorDepthImageRetriever(realsense,
                                                                      RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                                      realsenseFrameSupplier);
      realsenseImagePublisher = new RealsenseColorDepthImagePublisher(REALSENSE_DEPTH_TOPIC, REALSENSE_COLOR_TOPIC);
      realsenseProcessAndPublishThread = new RestartableThrottledThread("RealsenseProcessAndPublish", REALSENSE_FPS, this::processAndPublishRealsense);
      realsenseProcessAndPublishThread.start();
   }

   private void initializeRealsenseHearbeatCallbacks()
   {
      realsenseHeartbeat.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            if (realSenseManager == null)
               initializeRealsense();
            realsenseImageRetriever.start();
            realsenseImagePublisher.startAll();
         }
         else
         {
            realsenseImagePublisher.stopAll();
            realsenseImageRetriever.stop();
         }
      });
   }

   private void initializeOuster()
   {
      ouster = new NettyOuster();
      ouster.bind();
      ousterDepthImageRetriever = new OusterDepthImageRetriever(ouster, ousterFrameSupplier, lidarScanHeartbeat::isAlive, heightMapHeartbeat::isAlive);
      ousterDepthImagePublisher = new OusterDepthImagePublisher(ouster, OUSTER_DEPTH_TOPIC);
      ousterProcessAndPublishThread = new RestartableThrottledThread("OusterProcessAndPublish", OUSTER_FPS, this::processAndPublishOuster);
      ousterProcessAndPublishThread.start();
   }

   private void initializeOusterHeartbeatCallbacks()
   {
      ousterDepthHeartbeat.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            if (ouster == null)
               initializeOuster();
            ousterDepthImageRetriever.start();
            ousterDepthImagePublisher.startDepth();
         }
         else
         {
            ousterDepthImageRetriever.stop();
            ousterDepthImagePublisher.stopDepth();
         }
      });
   }


   private void initializeBlackfly()
   {
      blackflyManager = new SpinnakerBlackflyManager();
      SpinnakerBlackfly blackfly = blackflyManager.createSpinnakerBlackfly(LEFT_BLACKFLY_SERIAL_NUMBER);
      blackflyImageRetriever = new BlackflyImageRetriever(blackfly, BLACKFLY_LENS, RobotSide.RIGHT, blackflyFrameSupplier);
      blackflyImagePublisher = new BlackflyImagePublisher(BLACKFLY_LENS, blackflyFrameSupplier, BLACKFLY_IMAGE_TOPIC);
      blackflyProcessAndPublishThread = new RestartableThrottledThread("BlackflyProcessAndPublish", BLACKFLY_FPS, this::processAndPublishBlackfly);
      blackflyProcessAndPublishThread.start();
   }

   private void initializeBlackflyHeartbeatCallbacks()
   {
      blackflyImageHeartbeat.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
         {
            if (blackflyManager == null)
               initializeBlackfly();
            blackflyImageRetriever.start();
            blackflyImagePublisher.startImagePublishing();
         }
         else
         {
            blackflyImagePublisher.stopImagePublishing();
            blackflyImageRetriever.stop();
         }
      });
   }

   public static void main(String[] args)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "perception_autonomy_process");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      PerceptionAndAutonomyProcess publisher = new PerceptionAndAutonomyProcess(ros2Helper,
                                                                                ReferenceFrame::getWorldFrame,
                                                                                ReferenceFrame::getWorldFrame,
                                                                                ReferenceFrame::getWorldFrame,
                                                                                ReferenceFrame::getWorldFrame);
      publisher.start();
   }
}
