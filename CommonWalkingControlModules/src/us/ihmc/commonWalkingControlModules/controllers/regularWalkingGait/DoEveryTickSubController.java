package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.RobotSide;

public interface DoEveryTickSubController
{
   public void doEveryControlTick(RobotSide supportLeg);
}
