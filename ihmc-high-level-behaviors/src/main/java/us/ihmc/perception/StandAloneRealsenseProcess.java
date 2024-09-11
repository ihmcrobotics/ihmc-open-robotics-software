package us.ihmc.perception;

import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.RealsenseColorDepthImagePublisher;
import us.ihmc.sensors.RealsenseColorDepthImageRetriever;
import us.ihmc.tools.thread.RestartableThread;

import java.util.function.Supplier;

/**
 * This class handles publishing the color and depth of the realsense. Its meant to be a stand alone class that only touches the realsense.
 */
public class StandAloneRealsenseProcess
{
   private static ROS2Node ros2Node;

   private static final String REALSENSE_SERIAL_NUMBER = System.getProperty("d455.serial.number", "213522252883");
   private static final ROS2Topic<ImageMessage> REALSENSE_COLOR_TOPIC = PerceptionAPI.D455_COLOR_IMAGE;
   private static final ROS2Topic<ImageMessage> REALSENSE_DEPTH_TOPIC = PerceptionAPI.D455_DEPTH_IMAGE;

   private final RealsenseColorDepthImageRetriever realsenseImageRetriever;
   private final RealsenseColorDepthImagePublisher realsenseImagePublisher;

   private final ROS2DemandGraphNode realsenseDemandNode;
   private Supplier<ReferenceFrame> realsenseFrameSupplier = ReferenceFrame::getWorldFrame;

   public StandAloneRealsenseProcess(ROS2Helper ros2Helper, ROS2SyncedRobotModel syncedRobot)
   {
      if (syncedRobot != null)
      {
         realsenseFrameSupplier = syncedRobot.getReferenceFrames()::getSteppingCameraFrame;
      }
      realsenseDemandNode = new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_REALSENSE_POINT_CLOUD);

      realsenseImageRetriever = new RealsenseColorDepthImageRetriever(new RealsenseDeviceManager(),
                                                                      REALSENSE_SERIAL_NUMBER,
                                                                      RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                                      realsenseFrameSupplier,
                                                                      realsenseDemandNode::isDemanded);

      realsenseImagePublisher = new RealsenseColorDepthImagePublisher(REALSENSE_DEPTH_TOPIC, REALSENSE_COLOR_TOPIC);

      RestartableThread realsenseProcessAndPublishThread = new RestartableThread("RealsenseProcess", this::publishRealSense);
      realsenseProcessAndPublishThread.start();
   }

   private void publishRealSense()
   {
      if (realsenseDemandNode.isDemanded())
      {
         RawImage realsenseDepthImage = realsenseImageRetriever.getLatestRawDepthImage();
         RawImage realsenseColorImage = realsenseImageRetriever.getLatestRawColorImage();

         realsenseImagePublisher.setNextDepthImage(realsenseDepthImage.get());
         realsenseImagePublisher.setNextColorImage(realsenseColorImage.get());

         realsenseDepthImage.release();
         realsenseColorImage.release();
      }
      else
         ThreadTools.sleep(500);
   }

   public void destroy()
   {
      realsenseImagePublisher.destroy();
      realsenseImageRetriever.destroy();
      realsenseDemandNode.destroy();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "nadia_realsense_process");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      StandAloneRealsenseProcess standAloneRealsenseProcess = new StandAloneRealsenseProcess(ros2Helper, null);
      Runtime.getRuntime().addShutdownHook(new Thread(standAloneRealsenseProcess::destroy, "RealSenseProcess"));
   }
}
