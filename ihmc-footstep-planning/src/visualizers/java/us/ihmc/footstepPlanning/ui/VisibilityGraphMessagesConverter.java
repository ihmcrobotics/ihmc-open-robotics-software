package us.ihmc.footstepPlanning.ui;

import controller_msgs.msg.dds.NavigableRegionMessage;
import controller_msgs.msg.dds.VisibilityMapMessage;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

import java.util.List;

public class VisibilityGraphMessagesConverter
{
   public static VisibilityMapHolder convertToVisibilityMapHolder(VisibilityMapMessage message)
   {
      return null;
   }

   public static List<NavigableRegion> convertToNavigableRegionsList(List<NavigableRegionMessage> message)
   {
      return null;
   }

}
