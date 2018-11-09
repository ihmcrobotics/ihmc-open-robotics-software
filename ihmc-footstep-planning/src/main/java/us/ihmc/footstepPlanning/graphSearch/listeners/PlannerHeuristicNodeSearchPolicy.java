package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

/**
 * This interfaces defines a policy for searching for a new valid node using {@link #performSearchForValidNode(FootstepNode, FootstepNode)}.
 * Once this valid node is found, a set of actions defined by {@link PlannerHeuristicNodeActionPolicy} and added to this search policy using
 * {@link #attachActionPolicy(PlannerHeuristicNodeActionPolicy)} are executed.
 */
public interface PlannerHeuristicNodeSearchPolicy
{
   /**
    * Attaches an action policy to be performed if a valid node is found from the search.
    */
   void attachActionPolicy(PlannerHeuristicNodeActionPolicy actionPolicy);

   /**
    * If {@param rejectedNode}, which is the child of {@param parentNode}, performs a search based on this information
    * to try and find a new valid node.
    * @return whether the search was successful.
    */
   boolean performSearchForValidNode(FootstepNode rejectedNode, FootstepNode parentNode);

   /**
    * Returns the new valid node that results from {@link #performSearchForValidNode(FootstepNode, FootstepNode)}
    */
   FootstepNode pollNewValidNode();

   FootstepNode pollNewValidParentNode();

   /**
    * Using the new valid node, executes all the action policies defined by {@link PlannerHeuristicNodeActionPolicy}.
    */
   void executeActionPoliciesForNewValidNode();
}
