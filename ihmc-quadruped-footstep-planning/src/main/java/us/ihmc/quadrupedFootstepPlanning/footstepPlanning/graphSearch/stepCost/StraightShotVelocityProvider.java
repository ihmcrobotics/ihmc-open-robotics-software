package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

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

   public Vector2DReadOnly computeNominalVelocityHeadingInWorld(FootstepNode node)
   {
      Vector2D heading = new Vector2D();

      Point2DReadOnly nodeCenter = node.getOrComputeXGaitCenterPoint();
      heading.set(goalNode.getOrComputeXGaitCenterPoint());
      heading.sub(nodeCenter);
      heading.normalize();
      
      heading.scale(computeDistanceToGoalScalar(nodeCenter.getX(), nodeCenter.getY()));

      return heading;
   }

   public double computeNominalYaw(Point2DReadOnly node)
   {
      double pathHeading = Math.atan2(goalNode.getOrComputeXGaitCenterPoint().getY() - node.getY(),
                                      goalNode.getOrComputeXGaitCenterPoint().getX() - node.getX());
      pathHeading = AngleTools.trimAngleMinusPiToPi(pathHeading);

      double yawMultiplier = computeDistanceToGoalScalar(node.getX(), node.getY());
      double referenceHeading = yawMultiplier * pathHeading;
      referenceHeading += (1.0 - yawMultiplier) * goalNode.getNominalYaw();
      return AngleTools.trimAngleMinusPiToPi(referenceHeading);
   }
   
   private double computeDistanceToGoalScalar(double x, double y)
   {
      Point2DReadOnly goalCenter = goalNode.getOrComputeXGaitCenterPoint();
      double distanceToGoal = EuclidCoreTools.norm(x - goalCenter.getX(), y - goalCenter.getY());
      double finalTurnProximity = 1.0;

      double minimumBlendDistance = 0.75 * finalTurnProximity;
      double maximumBlendDistance = 1.25 * finalTurnProximity;
      
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
