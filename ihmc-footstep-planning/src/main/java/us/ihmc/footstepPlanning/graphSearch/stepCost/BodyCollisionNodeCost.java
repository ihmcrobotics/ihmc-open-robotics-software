package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
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

      FootstepNodeSnapData snapData = snapper.getSnapData(endNode);
      if(snapData == null)
         return 0.0;

      RigidBodyTransform snapTransform = snapData.getSnapTransform();
      if(snapTransform == null)
         return 0.0;

      double height = snapTransform.getTranslationZ();
      BodyCollisionData collisionData = collisionDetector.checkForCollision(endNode, height);

      if(collisionData.isCollisionDetected())
      {
         return 0.0;
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
