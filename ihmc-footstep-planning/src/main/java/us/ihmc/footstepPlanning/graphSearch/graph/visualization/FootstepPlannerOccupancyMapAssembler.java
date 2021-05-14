package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.footstepPlanning.graphSearch.AStarIterationData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;

import java.util.function.Consumer;

public class FootstepPlannerOccupancyMapAssembler implements Consumer<AStarIterationData<FootstepGraphNode>>
{
   private final PlannerOccupancyMap occupancyMap = new PlannerOccupancyMap();

   public void reset()
   {
      occupancyMap.clear();
   }

   @Override
   public void accept(AStarIterationData<FootstepGraphNode> iterationData)
   {
      for (int i = 0; i < iterationData.getValidChildNodes().size(); i++)
      {
         FootstepGraphNode node = iterationData.getValidChildNodes().get(i);
         occupancyMap.addOccupiedCell(new PlannerCell(node.getSecondStep().getXIndex(), node.getSecondStep().getYIndex()));
      }

      for (int i = 0; i < iterationData.getInvalidChildNodes().size(); i++)
      {
         FootstepGraphNode node = iterationData.getInvalidChildNodes().get(i);
         occupancyMap.addOccupiedCell(new PlannerCell(node.getSecondStep().getXIndex(), node.getSecondStep().getYIndex()));
      }
   }

   public PlannerOccupancyMap getOccupancyMap()
   {
      return occupancyMap;
   }
}
