package us.ihmc.atlas;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.atlas.sensors.AtlasRealsenseBasedREAStandaloneLauncher;
import us.ihmc.avatar.sensors.realsense.RealSenseL515CompressedImageROS1Bridge;
import us.ihmc.avatar.sensors.realsense.RealSenseL515PointCloudROS1Bridge;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;

import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;

public class AtlasLineDetectionDemo
{
//   private IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private String ROBOT_NAME = "Atlas";

   public AtlasLineDetectionDemo(RosMainNode rosMainNode, ROS2Node ros2Node)
   {

      new RealSenseL515CompressedImageROS1Bridge(rosMainNode, ros2Node);
      new RealSenseL515PointCloudROS1Bridge(rosMainNode, ros2Node);
      new AtlasRealsenseBasedREAStandaloneLauncher(false);
      new AtlasLineSegmentEstimator(rosMainNode, ros2Node);

//      execFootstepPlan();
   }

   public void execFootstepPlan()
   {
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      FootstepPlannerLogLoader.LoadResult loadresult = logLoader.load(new File("/home/quantum/.ihmc/logs/20201017_152926321_FootstepPlannerLog/"));
      if (!(loadresult == FootstepPlannerLogLoader.LoadResult.LOADED))
      {
         LogTools.error("Couldn't find file");
         return;
      }
      FootstepPlannerLog log = logLoader.getLog();
      FootstepPlanningToolboxOutputStatus statusPacket = log.getStatusPacket();
      FootstepDataListMessage footstepDataList = statusPacket.getFootstepDataList();

//      footstepDataListPublisher.publish(footstepDataList);
   }

   public static void main(String[] args) throws URISyntaxException
   {
      RosMainNode rosMainNode = new RosMainNode(new URI("http://localhost:11311"), "RealSenseL515DataPublisher", true);
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");
      new AtlasLineDetectionDemo(rosMainNode, ros2Node);
   }
}
