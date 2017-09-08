package us.ihmc.manipulation.planning.rrt;
/**
 * @method boolean isValidNode()
 * This method is for testing the node is valid.
 * 
 * @method RRTNode createNode()
 * This method is used for RRTTree, RRTPiecewisePath, RRTPlanner.
 * A class that inherits <RRTNode> must return a new constructor with overriding this method.
 * 
 * @author InhoLee 170224
 *
 */
public interface RRTInterface
{
   public abstract boolean isValidNode();
   public abstract RRTNode createNode();
}
