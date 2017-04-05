package us.ihmc.manipulation.planning.rrttimedomain;

import us.ihmc.manipulation.planning.manipulation.solarpanelmotion.SolarPanelPath;
import us.ihmc.manipulation.planning.rrt.RRTNode;

public class RRTPlanner1DTimeDomain extends RRTPlannerTimeDomain
{
   private SolarPanelPath cleaningPath;

   public RRTPlanner1DTimeDomain(RRTNode root)
   {
      super(root);
   }
   
   public void getCleaningPath(SolarPanelPath cleaningPath)
   {
      this.cleaningPath = cleaningPath;
   }

}
