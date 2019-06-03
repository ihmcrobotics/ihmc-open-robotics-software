package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;

public class ForwardVelocityProvider implements NominalVelocityProvider
{
   public void setGoalNode(FootstepNode goalNode)
   {
   }

   public Vector2DReadOnly computeNominalVelocityHeadingInWorld(FootstepNode node)
   {
      Vector2D heading = new Vector2D();

      heading.setX(Math.cos(node.getNominalYaw()));
      heading.setY(Math.sin(node.getNominalYaw()));

      return heading;
   }

   public double computeNominalYaw(Point2DReadOnly node)
   {
      return Double.NaN;
   }
}
