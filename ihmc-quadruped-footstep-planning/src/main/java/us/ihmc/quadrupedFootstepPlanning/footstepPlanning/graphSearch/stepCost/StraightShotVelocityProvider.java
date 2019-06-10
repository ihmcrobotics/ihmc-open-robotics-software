package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
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

   public Vector2DReadOnly computeNominalNormalizedVelocityHeadingInWorld(FootstepNode node)
   {
      Vector2D heading = new Vector2D();

      Point2DReadOnly nodeCenter = node.getOrComputeXGaitCenterPoint();
      heading.set(goalNode.getOrComputeXGaitCenterPoint());
      heading.sub(nodeCenter);
      heading.normalize();

      double proximityMultiplier = InterpolationTools.linearInterpolate(0.1, 1.0, computeDistanceToGoalScalar(nodeCenter.getX(), nodeCenter.getY(), 0.2));
      heading.scale(proximityMultiplier);

      return heading;
   }

   public double computeNominalYaw(Point2DReadOnly node)
   {
      double pathHeading = Math.atan2(goalNode.getOrComputeXGaitCenterPoint().getY() - node.getY(),
                                      goalNode.getOrComputeXGaitCenterPoint().getX() - node.getX());
      pathHeading = AngleTools.trimAngleMinusPiToPi(pathHeading);

      double yawMultiplier = computeDistanceToGoalScalar(node.getX(), node.getY(), 1.0);
      double referenceHeading = yawMultiplier * pathHeading;
      referenceHeading += (1.0 - yawMultiplier) * goalNode.getStepYaw();
      return AngleTools.trimAngleMinusPiToPi(referenceHeading);
   }

   private double computeDistanceToGoalScalar(double x, double y, double proximity)
   {
      Point2DReadOnly goalCenter = goalNode.getOrComputeXGaitCenterPoint();
      double distanceToGoal = EuclidCoreTools.norm(x - goalCenter.getX(), y - goalCenter.getY());

      double minimumBlendDistance = 0.75 * proximity;
      double maximumBlendDistance = 1.25 * proximity;

      double multiplier;
      if (distanceToGoal < minimumBlendDistance)
         multiplier = 0.0;
      else if(distanceToGoal > maximumBlendDistance)
         multiplier = 1.0;
      else
         multiplier = (distanceToGoal - minimumBlendDistance) / (maximumBlendDistance - minimumBlendDistance);

      return multiplier;
   }
}
