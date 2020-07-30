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
   private final VisibilityGraphsParametersPacket bodyPathParametersPacket = new VisibilityGraphsParametersPacket();
   private final FootstepPlannerParametersPacket footstepParametersPacket = new FootstepPlannerParametersPacket();
   private final SwingPlannerParametersPacket swingPlannerParametersPacket = new SwingPlannerParametersPacket();
   private final SplitFractionCalculatorParametersPacket splitFractionParametersPacket = new SplitFractionCalculatorParametersPacket();
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

   public VisibilityGraphsParametersPacket getBodyPathParametersPacket()
   {
      return bodyPathParametersPacket;
   }

   public FootstepPlannerParametersPacket getFootstepParametersPacket()
   {
      return footstepParametersPacket;
   }

   public SwingPlannerParametersPacket getSwingPlannerParametersPacket()
   {
      return swingPlannerParametersPacket;
   }

   public SplitFractionCalculatorParametersPacket getSplitFractionParametersPacket()
   {
      return splitFractionParametersPacket;
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

   /**
    * Reconstructs the solution as a list of FootstepNodes from the log
    */
   public List<FootstepNode> getFootstepPlan()
   {
      List<FootstepPlannerIterationData> iterationData = getIterationData();
      Map<GraphEdge<FootstepNode>, FootstepPlannerEdgeData> edgeDataMap = getEdgeDataMap();

      int iterationIndex = 0;
      List<FootstepNode> footstepNodes = new ArrayList<>();
      footstepNodes.add(iterationData.get(0).getStanceNode());

      mainLoop:
      while (true)
      {
         FootstepNode stanceNode = iterationData.get(iterationIndex).getStanceNode();

         stepLoop:
         for (int i = 0; i < iterationData.get(iterationIndex).getChildNodes().size(); i++)
         {
            FootstepNode childNode = iterationData.get(iterationIndex).getChildNodes().get(i);
            GraphEdge<FootstepNode> edge = new GraphEdge<>(stanceNode, childNode);

            if (edgeDataMap.get(edge).getSolutionEdge())
            {
               footstepNodes.add(childNode);
               for (int j = 0; j < iterationData.size(); j++)
               {
                  if (iterationData.get(j).getStanceNode().equals(childNode))
                  {
                     iterationIndex = j;
                     continue mainLoop;
                  }
               }

               break mainLoop;
            }
         }
      }

      return footstepNodes;
   }
}
