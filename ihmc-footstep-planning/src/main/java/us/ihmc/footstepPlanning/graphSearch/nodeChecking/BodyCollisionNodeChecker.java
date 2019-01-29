package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class BodyCollisionNodeChecker extends FootstepNodeChecker
{
   private final FootstepNodeBodyCollisionDetector collisionDetector;
   private final FootstepPlannerParameters parameters;
   private final FootstepNodeSnapperReadOnly snapper;

   public BodyCollisionNodeChecker(FootstepNodeBodyCollisionDetector collisionDetector, FootstepPlannerParameters parameters, FootstepNodeSnapperReadOnly snapper)
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

      LatticeNode midFootNode = FootstepNodeTools.computeAverage(node.getLatticeNode(), previousNode.getLatticeNode());
      double height = snapper.getSnapData(node).getSnapTransform().getTranslationZ();
      BodyCollisionData collisionData = collisionDetector.checkForCollision(midFootNode, height);
      return !collisionData.isCollisionDetected();
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {
   }
}
