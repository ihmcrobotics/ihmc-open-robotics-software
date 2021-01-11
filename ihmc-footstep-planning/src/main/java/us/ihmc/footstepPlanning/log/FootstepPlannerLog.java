package us.ihmc.footstepPlanning.log;

import controller_msgs.msg.dds.*;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphHolder;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FootstepPlannerLog
{
   private final String logName;

   // Packets
   private final FootstepPlanningRequestPacket requestPacket = new FootstepPlanningRequestPacket();
   private final VisibilityGraphsParametersPacket bodyPathParametersPacket = new VisibilityGraphsParametersPacket();
   private final FootstepPlannerParametersPacket footstepParametersPacket = new FootstepPlannerParametersPacket();
   private final SwingPlannerParametersPacket swingPlannerParametersPacket = new SwingPlannerParametersPacket();
   private final FootstepPlanningToolboxOutputStatus statusPacket = new FootstepPlanningToolboxOutputStatus();

   // Logged data
   private final VisibilityGraphHolder visibilityGraphHolder = new VisibilityGraphHolder();
   private final List<VariableDescriptor> variableDescriptors = new ArrayList<>();
   private final Map<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData> edgeDataMap = new HashMap<>();
   private final List<FootstepPlannerIterationData> iterationData = new ArrayList<>();
   private final SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();

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

   public FootstepPlanningToolboxOutputStatus getStatusPacket()
   {
      return statusPacket;
   }

   public VisibilityGraphHolder getVisibilityGraphHolder()
   {
      return visibilityGraphHolder;
   }

   public List<VariableDescriptor> getVariableDescriptors()
   {
      return variableDescriptors;
   }

   public Map<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData> getEdgeDataMap()
   {
      return edgeDataMap;
   }

   public List<FootstepPlannerIterationData> getIterationData()
   {
      return iterationData;
   }

   public SideDependentList<ConvexPolygon2D> getFootPolygons()
   {
      return footPolygons;
   }

   /**
    * Reconstructs the solution as a list of FootstepNodes from the log
    */
   public List<FootstepGraphNode> getFootstepPlan()
   {
      List<FootstepPlannerIterationData> iterationData = getIterationData();
      Map<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData> edgeDataMap = getEdgeDataMap();

      int iterationIndex = 0;
      List<FootstepGraphNode> footstepNodes = new ArrayList<>();
      footstepNodes.add(iterationData.get(0).getParentNode());

      mainLoop:
      while (true)
      {
         FootstepGraphNode parentNode = iterationData.get(iterationIndex).getParentNode();

         stepLoop:
         for (int i = 0; i < iterationData.get(iterationIndex).getChildNodes().size(); i++)
         {
            FootstepGraphNode childNode = iterationData.get(iterationIndex).getChildNodes().get(i);
            GraphEdge<FootstepGraphNode> edge = new GraphEdge<>(parentNode, childNode);

            // TODO
//            if (edgeDataMap.get(edge).getSolutionEdge())
            {
               footstepNodes.add(childNode);
               for (int j = 0; j < iterationData.size(); j++)
               {
                  if (iterationData.get(j).getParentNode().equals(childNode))
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
