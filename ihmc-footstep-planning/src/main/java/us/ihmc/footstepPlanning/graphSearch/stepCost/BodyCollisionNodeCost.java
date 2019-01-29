package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;

public class BodyCollisionNodeCost implements FootstepCost
{
   private final FootstepNodeBodyCollisionDetector collisionDetector;
   private final FootstepPlannerCostParameters costParameters;
   private final FootstepNodeSnapperReadOnly snapper;

   public BodyCollisionNodeCost(FootstepNodeBodyCollisionDetector collisionDetector, FootstepPlannerCostParameters costParameters, FootstepNodeSnapperReadOnly snapper)
   {
      this.collisionDetector = collisionDetector;
      this.costParameters = costParameters;
      this.snapper = snapper;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      if(costParameters.getBoundingBoxCost() <= 0.0 || costParameters.getMaximum2dDistanceFromBoundingBoxToPenalize() <= 0.0)
      {
         return 0.0;
      }

      double height = snapper.getSnapData(endNode).getSnapTransform().getTranslationZ();
      BodyCollisionData collisionData = collisionDetector.checkForCollision(endNode.getLatticeNode(), height);

      if(collisionData.isCollisionDetected())
      {
         throw new RuntimeException("Collision detector detected collision. This node should be invalid");
      }
      else if(Double.isNaN(collisionData.getDistanceFromBoundingBox()))
      {
         return 0.0;
      }
      else
      {
         return costParameters.getBoundingBoxCost() * (1.0 - collisionData.getDistanceFromBoundingBox() / costParameters.getMaximum2dDistanceFromBoundingBoxToPenalize());
      }
   }
}
