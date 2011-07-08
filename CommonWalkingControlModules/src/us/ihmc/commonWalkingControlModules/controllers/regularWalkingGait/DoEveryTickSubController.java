package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.robotSide.RobotSide;

public interface DoEveryTickSubController
{
   public abstract void doEveryControlTick(RobotSide supportLeg);
   
   public abstract void doFirstTick();
   
   public abstract void addUpdatable(Updatable updatable);

}
