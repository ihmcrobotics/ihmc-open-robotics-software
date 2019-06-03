package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;

public class StraightShotVelocityProvider implements NominalVelocityProvider
{
   private FootstepNode goalNode;

   public void setGoalNode(FootstepNode goalNode)
   {
      this.goalNode = goalNode;
   }

   public Vector2DReadOnly computeNominalVelocityHeadingInWorld(FootstepNode node)
   {
      Vector2D heading = new Vector2D();

      heading.set(goalNode.getOrComputeXGaitCenterPoint());
      heading.sub(node.getOrComputeXGaitCenterPoint());
      heading.normalize();

      return heading;
   }

   public double computeNominalYaw(Point2DReadOnly node)
   {
      double distanceToGoal = node.distance(goalNode.getOrComputeXGaitCenterPoint());
      double finalTurnProximity = 1.0;

      double minimumBlendDistance = 0.75 * finalTurnProximity;
      double maximumBlendDistance = 1.25 * finalTurnProximity;

      double pathHeading = Math.atan2(goalNode.getOrComputeXGaitCenterPoint().getY() - node.getY(),
                                      goalNode.getOrComputeXGaitCenterPoint().getX() - node.getX());
      pathHeading = AngleTools.trimAngleMinusPiToPi(pathHeading);

      double yawMultiplier;
      if (distanceToGoal < minimumBlendDistance)
         yawMultiplier = 0.0;
      else if(distanceToGoal > maximumBlendDistance)
         yawMultiplier = 1.0;
      else
         yawMultiplier = (distanceToGoal - minimumBlendDistance) / (maximumBlendDistance - minimumBlendDistance);

      double referenceHeading = yawMultiplier * pathHeading;
      referenceHeading += (1.0 - yawMultiplier) * goalNode.getNominalYaw();
      return AngleTools.trimAngleMinusPiToPi(referenceHeading);
   }

}
