package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Wrench;

// TODO Nuke that interface
public interface FootSwitchInterface
{
   public abstract boolean hasFootHitGround();
   
   public abstract double computeFootLoadPercentage();
   public abstract void computeAndPackCoP(FramePoint2d copToPack);

   public abstract void computeAndPackFootWrench(Wrench footWrenchToPack);

   public abstract ReferenceFrame getMeasurementFrame();

   public void reset();

   public abstract boolean getForceMagnitudePastThreshhold();
}
