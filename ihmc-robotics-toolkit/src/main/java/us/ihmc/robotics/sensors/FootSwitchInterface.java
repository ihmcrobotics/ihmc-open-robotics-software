package us.ihmc.robotics.sensors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
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

   /**
    * Returns whether this foot switch estimates that the foot has made contact with the ground.
    * <p>
    * It typically relies on force measurement at the foot (either from a force/torque sensor or
    * estimate from joint torques) in addition to some filtering to prune out false positives.
    * </p>
    */
   boolean hasFootHitGround();

   /**
    * @return a value in [0, 1] representing the force magnitude on the foot in terms of robot weight.
    */
   double getFootLoadPercentage();

   /**
    * Gets the estimated center of pressure coordinates in the foot, or {@code null} if this foot
    * switch cannot estimate the center of pressure.
    */
   FramePoint2DReadOnly getCenterOfPressure();

   /**
    * Gets the estimated center of pressure coordinates in the foot, or {@code null} if this foot
    * switch cannot estimate the center of pressure.
    * <p>
    * If this foot switch cannot compute the center of pressure, {@code centerOfPressureToPack} is
    * filled with {@value Double#NaN}.
    * </p>
    */
   default void getCenterOfPressure(FramePoint2DBasics centerOfPressureToPack)
   {
      FramePoint2DReadOnly cop = getCenterOfPressure();
      if (cop == null)
      {
         centerOfPressureToPack.setToNaN(getMeasurementFrame());
      }

      centerOfPressureToPack.setIncludingFrame(cop);
   }

   void computeAndPackFootWrench(Wrench footWrenchToPack);

   ReferenceFrame getMeasurementFrame();

   boolean getForceMagnitudePastThreshhold();
}
