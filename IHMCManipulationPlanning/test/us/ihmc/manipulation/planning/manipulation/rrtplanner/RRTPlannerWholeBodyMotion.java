package us.ihmc.manipulation.planning.manipulation.rrtplanner;

import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.rrt.RRTPlanner;

public class RRTPlannerWholeBodyMotion extends RRTPlanner
{

   public RRTPlannerWholeBodyMotion(RRTNode root, RRTNode goal, double stepLength)
   {
      super(root, goal, stepLength);
   }

}
