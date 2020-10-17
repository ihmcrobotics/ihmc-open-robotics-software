package us.ihmc.atlas;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeInterface;

import java.io.File;

public class AtlasLineDetectionDemo
{
   private IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private String ROBOT_NAME = "Atlas";

   public AtlasLineDetectionDemo(ROS2Node ros2Node){
      footstepDataListPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, FootstepDataListMessage.class, ROS2Tools.getControllerInputTopic(ROBOT_NAME));
      execFootstepPlan();
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

      footstepDataListPublisher.publish(footstepDataList);
   }

   public static void main(String[] args){
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");
      new AtlasLineDetectionDemo(ros2Node);
   }
}
