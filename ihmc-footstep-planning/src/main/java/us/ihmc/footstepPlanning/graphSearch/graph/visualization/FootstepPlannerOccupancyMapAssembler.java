package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerIterationData;

import java.util.function.Consumer;

public class FootstepPlannerOccupancyMapAssembler implements Consumer<FootstepPlannerIterationData<FootstepNode>>
{
   private final PlannerOccupancyMap occupancyMap = new PlannerOccupancyMap();

   public void reset()
   {
      occupancyMap.clear();
   }

   @Override
   public void accept(FootstepPlannerIterationData<FootstepNode> iterationData)
   {
      for (int i = 0; i < iterationData.getValidChildNodes().size(); i++)
      {
         FootstepNode node = iterationData.getValidChildNodes().get(i);
         occupancyMap.addOccupiedCell(new PlannerCell(node.getXIndex(), node.getYIndex()));
      }

      for (int i = 0; i < iterationData.getInvalidChildNodes().size(); i++)
      {
         FootstepNode node = iterationData.getInvalidChildNodes().get(i);
         occupancyMap.addOccupiedCell(new PlannerCell(node.getXIndex(), node.getYIndex()));
      }
   }

   public PlannerOccupancyMap getOccupancyMap()
   {
      return occupancyMap;
   }
}
