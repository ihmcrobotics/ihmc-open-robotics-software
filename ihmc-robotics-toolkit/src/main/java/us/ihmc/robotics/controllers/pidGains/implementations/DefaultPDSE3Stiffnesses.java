package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.robotics.controllers.pidGains.PDSE3Stiffnesses;
import us.ihmc.robotics.controllers.pidGains.PDSE3StiffnessesReadOnly;

/**
 * Provides a default implementation for PD gains for a SE3 PD controller.
 * <p>
 * This class uses two {@link DefaultPD3DStiffnesses}, one for position and one for orientation control,
 * internally.
 * </p>
 */
public class DefaultPDSE3Stiffnesses implements PDSE3Stiffnesses, Settable<DefaultPDSE3Stiffnesses>
{
   private final DefaultPD3DStiffnesses positionStiffnesses;
   private final DefaultPD3DStiffnesses orientationStiffnesses;

   public DefaultPDSE3Stiffnesses()
   {
      positionStiffnesses = new DefaultPD3DStiffnesses();
      orientationStiffnesses = new DefaultPD3DStiffnesses();
   }

   @Override
   public void set(DefaultPDSE3Stiffnesses other)
   {
      PDSE3Stiffnesses.super.set(other);
   }

   @Override
   public DefaultPD3DStiffnesses getPositionStiffnesses()
   {
      return positionStiffnesses;
   }

   @Override
   public DefaultPD3DStiffnesses getOrientationStiffnesses()
   {
      return orientationStiffnesses;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PDSE3StiffnessesReadOnly)
         return PDSE3Stiffnesses.super.equals((PDSE3StiffnessesReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return "Position: " + getPositionStiffnesses().toString() + "; Orientation: " + getOrientationStiffnesses().toString();
   }
}
