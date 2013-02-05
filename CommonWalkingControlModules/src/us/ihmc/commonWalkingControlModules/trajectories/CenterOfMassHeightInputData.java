package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotSide.RobotSide;

public class CenterOfMassHeightInputData
{
   private RobotSide supportLeg;
   
   public CenterOfMassHeightInputData()
   {
      
   }

   public RobotSide getSupportLeg()
   {
      return supportLeg;
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      this.supportLeg = supportLeg;
   }
}
