package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.robotics.robotSide.RobotSide;

public interface DoEveryTickSubController
{
   public abstract void doEveryControlTick(RobotSide supportLeg);

   public abstract void addUpdatable(Updatable updatable);

   public abstract void initialize();
}
