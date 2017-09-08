package us.ihmc.robotics.sensors;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Wrench;

// TODO Nuke that interface
public interface FootSwitchInterface
{
   public abstract boolean hasFootHitGround();
   
   public abstract double computeFootLoadPercentage();

   public abstract void computeAndPackCoP(FramePoint2D copToPack);

   public abstract void updateCoP();

   public abstract void computeAndPackFootWrench(Wrench footWrenchToPack);

   public abstract ReferenceFrame getMeasurementFrame();

   public void reset();

   public abstract boolean getForceMagnitudePastThreshhold();

   public void setFootContactState(boolean hasFootHitGround);

   public void trustFootSwitch(boolean trustFootSwitch);
}
