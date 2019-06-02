package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;

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

}
