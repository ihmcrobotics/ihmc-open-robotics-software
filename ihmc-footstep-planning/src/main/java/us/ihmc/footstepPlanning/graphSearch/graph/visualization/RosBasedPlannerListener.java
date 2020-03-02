package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;

public class RosBasedPlannerListener extends MessageBasedPlannerListener
{
   private final IHMCROS2Publisher<FootstepNodeDataListMessage> plannerNodeDataPublisher;
   private final IHMCROS2Publisher<FootstepPlannerOccupancyMapMessage> occupancyMapPublisher;

   public RosBasedPlannerListener(IHMCROS2Publisher<FootstepNodeDataListMessage> plannerNodeDataPublisher, IHMCROS2Publisher<FootstepPlannerOccupancyMapMessage> occupancyMapPublisher, FootstepNodeSnapper snapper, long broadcastDtMillis)
   {
      super(snapper, broadcastDtMillis);
      this.plannerNodeDataPublisher = plannerNodeDataPublisher;
      this.occupancyMapPublisher = occupancyMapPublisher;
   }

   @Override
   void broadcastLowestCostNodeData(PlannerNodeDataList message)
   {
      plannerNodeDataPublisher.publish(message.getAsMessage());
   }

   @Override
   void broadcastOccupancyMap(PlannerOccupancyMap message)
   {
      occupancyMapPublisher.publish(message.getAsMessage());
   }
}
