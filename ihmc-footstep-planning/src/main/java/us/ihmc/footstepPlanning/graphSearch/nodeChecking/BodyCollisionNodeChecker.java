package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class BodyCollisionNodeChecker extends FootstepNodeChecker
{
   private final FootstepNodeBodyCollisionDetector collisionDetector;
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeSnapperReadOnly snapper;

   public BodyCollisionNodeChecker(FootstepNodeBodyCollisionDetector collisionDetector, FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapperReadOnly snapper)
   {
      this.parameters = parameters;
      this.collisionDetector = collisionDetector;
      this.snapper = snapper;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      super.setPlanarRegions(planarRegions);
      collisionDetector.setPlanarRegionsList(planarRegionsList);
   }

   @Override
   public boolean isNodeValidInternal(FootstepNode node, FootstepNode previousNode)
   {
      if (previousNode == null || !parameters.checkForBodyBoxCollisions() || !hasPlanarRegions())
      {
         return true;
      }

      RigidBodyTransformReadOnly snappedNodeTransform = snapper.getSnapData(node).getOrComputeSnappedNodeTransform(node);
      RigidBodyTransformReadOnly previousSnappedNodeTransform = snapper.getSnapData(previousNode).getOrComputeSnappedNodeTransform(previousNode);

      double snapHeight = snappedNodeTransform.getTranslationZ();
      double previousSnapHeight = previousSnappedNodeTransform.getTranslationZ();

      int numberOfBoundingBoxChecks = Math.max(1, parameters.getNumberOfBoundingBoxChecks());
      List<BodyCollisionData> collisionData = collisionDetector.checkForCollision(node, previousNode, snapHeight, previousSnapHeight, numberOfBoundingBoxChecks);

      for (int i = 0; i < collisionData.size(); i++)
      {
         if(collisionData.get(i).isCollisionDetected())
         {
            rejectNode(node, previousNode, BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_HITTING_BODY);
            return false;
         }
      }

      return true;
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {
   }
}
