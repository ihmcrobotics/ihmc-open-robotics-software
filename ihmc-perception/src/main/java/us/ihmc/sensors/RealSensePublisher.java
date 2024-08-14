package us.ihmc.sensors;

import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.tools.thread.RestartableThread;

import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Supplier;

public class RealSensePublisher
{
   private static final String REALSENSE_SERIAL_NUMBER = System.getProperty("d455.serial.number", "215122253249");
   private static final ROS2Topic<ImageMessage> REALSENSE_COLOR_TOPIC = PerceptionAPI.D455_COLOR_IMAGE;
   private static final ROS2Topic<ImageMessage> REALSENSE_DEPTH_TOPIC = PerceptionAPI.D455_DEPTH_IMAGE;

   private final RealsenseColorDepthImageRetriever realsenseImageRetriever;
   private final RealsenseColorDepthImagePublisher realsenseImagePublisher;

   private final ROS2DemandGraphNode realsenseDemandNode;

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   public RealSensePublisher()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "perception_autonomy_process");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      realsenseDemandNode = new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_REALSENSE_POINT_CLOUD);

      Supplier<ReferenceFrame> realsenseFrameSupplier = ReferenceFrame::getWorldFrame;
      realsenseImageRetriever = new RealsenseColorDepthImageRetriever(new RealsenseDeviceManager(),
                                                                      REALSENSE_SERIAL_NUMBER,
                                                                      RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                                      realsenseFrameSupplier,
                                                                      realsenseDemandNode);

      realsenseImagePublisher = new RealsenseColorDepthImagePublisher(REALSENSE_DEPTH_TOPIC, REALSENSE_COLOR_TOPIC);

      RestartableThread realsenseProcessAndPublishThread = new RestartableThread("RealsenseProcessAndPublish", this::processAndPublishRealsense);
      realsenseProcessAndPublishThread.start();
   }

   private void processAndPublishRealsense()
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

   public static void main(String[] args)
   {
      new RealSensePublisher();
   }
}
