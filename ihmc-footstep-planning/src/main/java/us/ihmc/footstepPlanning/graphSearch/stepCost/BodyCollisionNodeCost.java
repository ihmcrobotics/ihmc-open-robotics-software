package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;

public class BodyCollisionNodeCost implements FootstepCost
{
   private final FootstepNodeBodyCollisionDetector collisionDetector;
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeSnapperReadOnly snapper;

   public BodyCollisionNodeCost(FootstepNodeBodyCollisionDetector collisionDetector, FootstepPlannerParametersReadOnly costParameters, FootstepNodeSnapperReadOnly snapper)
   {
      this.collisionDetector = collisionDetector;
      this.parameters = costParameters;
      this.snapper = snapper;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      if(parameters.getBoundingBoxCost() <= 0.0 || parameters.getMaximum2dDistanceFromBoundingBoxToPenalize() <= 0.0)
      {
         return 0.0;
      }

      FootstepNodeSnapData snapData = snapper.getSnapData(endNode);
      if(snapData == null)
         return 0.0;

      RigidBodyTransform snapTransform = snapData.getSnapTransform();
      if(snapTransform == null)
         return 0.0;

      RigidBodyTransform snappedNodeTransform = snapData.getOrComputeSnappedNodeTransform(endNode);
      BodyCollisionData collisionData = collisionDetector.checkForCollision(endNode, snappedNodeTransform.getTranslationZ());

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
         return parameters.getBoundingBoxCost() * (1.0 - collisionData.getDistanceFromBoundingBox() / parameters.getMaximum2dDistanceFromBoundingBoxToPenalize());
      }
   }
}
