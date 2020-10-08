package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.footstepPlanning.graphSearch.AStarIterationData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;

import java.util.function.Consumer;

public class FootstepPlannerOccupancyMapAssembler implements Consumer<AStarIterationData<FootstanceNode>>
{
   private final PlannerOccupancyMap occupancyMap = new PlannerOccupancyMap();

   public void reset()
   {
      occupancyMap.clear();
   }

   @Override
   public void accept(AStarIterationData<FootstanceNode> iterationData)
   {
      for (int i = 0; i < iterationData.getValidChildNodes().size(); i++)
      {
         FootstanceNode node = iterationData.getValidChildNodes().get(i);
         occupancyMap.addOccupiedCell(new PlannerCell(node.getStanceNode().getXIndex(), node.getStanceNode().getYIndex()));
      }

      for (int i = 0; i < iterationData.getInvalidChildNodes().size(); i++)
      {
         FootstanceNode node = iterationData.getInvalidChildNodes().get(i);
         occupancyMap.addOccupiedCell(new PlannerCell(node.getStanceNode().getXIndex(), node.getStanceNode().getYIndex()));
      }
   }

   public PlannerOccupancyMap getOccupancyMap()
   {
      return occupancyMap;
   }
}
