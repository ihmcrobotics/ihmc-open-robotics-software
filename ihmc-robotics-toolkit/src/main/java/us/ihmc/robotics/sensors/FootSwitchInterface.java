package us.ihmc.robotics.sensors;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;

// TODO Nuke that interface
public interface FootSwitchInterface
{
   /**
    * This method is intended to be called once per control tick and should update the internal state of this foot switch. The other methods should avoid any extra computation.
    */
   default void updateMeasurement()
   {}

   public abstract boolean hasFootHitGround();
   
   public abstract double computeFootLoadPercentage();

   public abstract void computeAndPackCoP(FramePoint2D copToPack);

   public abstract void updateCoP();

   public abstract void computeAndPackFootWrench(Wrench footWrenchToPack);

   public abstract ReferenceFrame getMeasurementFrame();

   public void reset();

   public abstract boolean getForceMagnitudePastThreshhold();

   public void setFootContactState(boolean hasFootHitGround);

   public void trustFootSwitchInSwing(boolean trustFootSwitchInSwing);

   public void trustFootSwitchInSupport(boolean trustFootSwitchInSupport);
}
