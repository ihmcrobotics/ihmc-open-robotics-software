package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;

public class BodyPathBasedVelocityProvider implements NominalVelocityProvider
{
   private final BodyPathPlanner bodyPath;

   public BodyPathBasedVelocityProvider(BodyPathPlanner bodyPath)
   {
      this.bodyPath = bodyPath;
   }

   public void setGoalNode(FootstepNode goalNode)
   {
   }

   public Vector2DReadOnly computeNominalNormalizedVelocityHeadingInWorld(FootstepNode node)
   {
      Point2DReadOnly xGaitCenterPoint = node.getOrComputeXGaitCenterPoint();
      Pose2D closestPointOnPath = new Pose2D();

      bodyPath.getClosestPoint(xGaitCenterPoint, closestPointOnPath);
      double yaw = closestPointOnPath.getYaw();

      Vector2D heading = new Vector2D();
      heading.setX(Math.cos(yaw));
      heading.setY(Math.sin(yaw));

      return heading;
   }

   public double computeNominalYaw(Point2DReadOnly node, double yaw)
   {
      Pose2D closestPointOnPath = new Pose2D();

      bodyPath.getClosestPoint(node, closestPointOnPath);
      return closestPointOnPath.getYaw();
   }
}
