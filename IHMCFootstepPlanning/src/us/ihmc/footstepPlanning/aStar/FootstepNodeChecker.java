package us.ihmc.footstepPlanning.aStar;

public interface FootstepNodeChecker
{
   public boolean isNodeValid(FootstepNode node, FootstepNode previosNode);
}
