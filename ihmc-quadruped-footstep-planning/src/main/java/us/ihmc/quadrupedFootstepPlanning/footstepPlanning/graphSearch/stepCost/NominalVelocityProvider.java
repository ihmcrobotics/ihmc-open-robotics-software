package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;

public interface NominalVelocityProvider
{
   void setGoalNode(FootstepNode goalNode);

   Vector2DReadOnly computeNominalVelocityHeadingInWorld(FootstepNode node);
}
