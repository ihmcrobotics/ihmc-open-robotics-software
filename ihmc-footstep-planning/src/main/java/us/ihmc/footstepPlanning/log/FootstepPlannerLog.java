package us.ihmc.footstepPlanning.log;

import controller_msgs.msg.dds.*;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphHolder;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FootstepPlannerLog
{
   private final String logName;

   private final FootstepPlanningRequestPacket requestPacket = new FootstepPlanningRequestPacket();
   private final FootstepPlannerParametersPacket footstepParametersPacket = new FootstepPlannerParametersPacket();
   private final VisibilityGraphsParametersPacket bodyPathParametersPacket = new VisibilityGraphsParametersPacket();
   private final FootstepPlanningToolboxOutputStatus statusPacket = new FootstepPlanningToolboxOutputStatus();

   private final VisibilityGraphHolder visibilityGraphHolder = new VisibilityGraphHolder();
   private final Map<GraphEdge<FootstepNode>, FootstepPlannerEdgeData> edgeDataMap = new HashMap<>();
   private final List<FootstepPlannerIterationData> iterationData = new ArrayList<>();

   public FootstepPlannerLog(String logName)
   {
      this.logName = logName;
   }

   public String getLogName()
   {
      return logName;
   }

   public FootstepPlanningRequestPacket getRequestPacket()
   {
      return requestPacket;
   }

   public FootstepPlannerParametersPacket getFootstepParametersPacket()
   {
      return footstepParametersPacket;
   }

   public VisibilityGraphsParametersPacket getBodyPathParametersPacket()
   {
      return bodyPathParametersPacket;
   }

   public FootstepPlanningToolboxOutputStatus getStatusPacket()
   {
      return statusPacket;
   }

   public VisibilityGraphHolder getVisibilityGraphHolder()
   {
      return visibilityGraphHolder;
   }

   public Map<GraphEdge<FootstepNode>, FootstepPlannerEdgeData> getEdgeDataMap()
   {
      return edgeDataMap;
   }

   public List<FootstepPlannerIterationData> getIterationData()
   {
      return iterationData;
   }
}
