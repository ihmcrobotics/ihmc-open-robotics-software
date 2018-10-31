package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

/**
 * This class is used to define an action to take in the event that a new valid node is found in some way.
 * Typically used in combination with the {@link PlannerHeuristicNodeSearchPolicy}.
 */
public interface PlannerHeuristicNodeActionPolicy
{
   void addActionListener(PlannerHeuristicActionListener actionListener);

   /**
    *  Defines the action to be taken when a new valid node is found
    **/
   void performActionForNewValidNode(FootstepNode newValidNode, FootstepNode parentNode);
}
