package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeChecker;

public class AlwaysValidNodeChecker implements FootstepNodeChecker
{
   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previosNode)
   {
      return true;
   }
}
