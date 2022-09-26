package us.ihmc.robotics.sensors;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;

// TODO Nuke that interface
public interface FootSwitchInterface
{
   public void reset();

   /**
    * This method is intended to be called once per control tick and should update the internal state
    * of this foot switch. The other methods should avoid any extra computation.
    */
   default void update()
   {
   }

   boolean hasFootHitGround();

   double computeFootLoadPercentage();

   void computeAndPackCoP(FramePoint2D copToPack);

   void computeAndPackFootWrench(Wrench footWrenchToPack);

   ReferenceFrame getMeasurementFrame();

   boolean getForceMagnitudePastThreshhold();
}
