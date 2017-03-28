package us.ihmc.footstepPlanning.aStar;

public interface FootstepCost
{
   public double compute(FootstepNode startNode, FootstepNode endNode);
}
