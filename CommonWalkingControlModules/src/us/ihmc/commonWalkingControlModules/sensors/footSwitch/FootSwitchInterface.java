package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Wrench;

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
