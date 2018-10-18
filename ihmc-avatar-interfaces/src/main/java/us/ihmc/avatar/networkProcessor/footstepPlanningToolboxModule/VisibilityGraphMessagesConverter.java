package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import controller_msgs.msg.dds.BodyPathPlanStatisticsMessage;
import controller_msgs.msg.dds.NavigableRegionMessage;
import controller_msgs.msg.dds.VisibilityMapMessage;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

import java.util.List;

public class VisibilityGraphMessagesConverter
{
   public static BodyPathPlanStatisticsMessage convertToBodyPathPlanStatisticsMessage(VisibilityGraphStatistics statistics)
   {
      BodyPathPlanStatisticsMessage message = new BodyPathPlanStatisticsMessage();

      throw new RuntimeException("Not Yet Implemented.");
   }

   public static VisibilityMapHolder convertToVisibilityMapHolder(VisibilityMapMessage message)
   {
      throw new RuntimeException("Not Yet Implemented.");
   }

   public static List<NavigableRegion> convertToNavigableRegionsList(List<NavigableRegionMessage> message)
   {
      throw new RuntimeException("Not Yet Implemented.");
   }
}
