package us.ihmc.footstepPlanning.aStar;

import java.util.HashSet;

public interface FootstepNodeExpansion
{
   public HashSet<FootstepNode> expandNode(FootstepNode node);
}
