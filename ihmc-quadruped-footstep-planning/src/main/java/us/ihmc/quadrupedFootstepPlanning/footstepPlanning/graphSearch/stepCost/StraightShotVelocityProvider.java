package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.CostTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;


public class StraightShotVelocityProvider implements NominalVelocityProvider
{
   private static final double finalTurnProximity = 1.0;
   private static final double finalSlowDownProximity = 0.5;
   private FootstepNode goalNode;

   public void setGoalNode(FootstepNode goalNode)
   {
      this.goalNode = goalNode;
   }

   public Vector2DReadOnly computeNominalNormalizedVelocityHeadingInWorld(FootstepNode node)
   {
      Vector2D heading = new Vector2D();

      Point2DReadOnly nodeCenter = node.getOrComputeXGaitCenterPoint();
      heading.set(goalNode.getOrComputeXGaitCenterPoint());
      heading.sub(nodeCenter);

      double distance = heading.length();

      double scaleFactor = InterpolationTools.linearInterpolate(0.25, 1.0, CostTools.computeDistanceToGoalScalar(nodeCenter.getX(), nodeCenter.getY(), goalNode,
                                                                                                                 finalSlowDownProximity));
      heading.scale(scaleFactor / distance);

      return heading;
   }



   public double computeNominalYaw(Point2DReadOnly nodeCenterPoint, double nodeYaw)
   {
      return CostTools.computeReferenceYaw(nodeCenterPoint, nodeYaw, goalNode, finalTurnProximity);
   }


}
