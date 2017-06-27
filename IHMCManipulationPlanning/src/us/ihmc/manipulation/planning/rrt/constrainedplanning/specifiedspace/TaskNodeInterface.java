package us.ihmc.manipulation.planning.rrt.constrainedplanning.specifiedspace;
/**
 * @method boolean isValidNode()
 * This method is for testing the node is valid.
 * 
 * @method TaskNode createNode()
 * This method is used for TaskNodeTree.
 * A class that inherits <TaskNode> must return a new constructor with overriding this method.
 * 
 * @author InhoLee 170626
 *
 */
public interface TaskNodeInterface
{
   public boolean isValidNode();
   public abstract TaskNode createNode();
}
