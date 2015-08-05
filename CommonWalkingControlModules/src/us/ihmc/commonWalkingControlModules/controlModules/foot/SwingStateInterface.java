package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;

public interface SwingStateInterface
{

   public abstract void setFootstep(Footstep footstep);

   public abstract void replanTrajectory(Footstep footstep);

   public abstract void requestSwingSpeedUp(double speedUpFactor);

   public abstract void setInitialDesireds(FrameOrientation initialOrientation, FrameVector initialAngularVelocity);

}