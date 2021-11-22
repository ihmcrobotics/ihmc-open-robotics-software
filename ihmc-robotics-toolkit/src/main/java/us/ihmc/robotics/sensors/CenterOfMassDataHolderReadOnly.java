package us.ihmc.robotics.sensors;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface CenterOfMassDataHolderReadOnly
{
   /**
    * Gets whether this data holder contains estimated position of the center of mass. If this method
    * returns {@code false}, the data contained in {@link #getCenterOfMassPosition()} should
    * <strong>not</strong> be consumed.
    * 
    * @return whether this data holder contains estimated position of the center of mass.
    */
   boolean hasCenterOfMassPosition();

   /**
    * Gets the estimated position of the center of mass if available.
    * 
    * @return the estimated position of the center of mass if available.
    */
   FramePoint3DReadOnly getCenterOfMassPosition();

   /**
    * Gets whether this data holder contains estimated velocity of the center of mass. If this method
    * returns {@code false}, the data contained in {@link #getCenterOfMassVelocity()} should
    * <strong>not</strong> be consumed.
    * 
    * @return whether this data holder contains estimated velocity of the center of mass.
    */
   boolean hasCenterOfMassVelocity();

   /**
    * Gets the estimated velocity of the center of mass if available.
    * 
    * @return the estimated velocity of the center of mass if available.
    */
   FrameVector3DReadOnly getCenterOfMassVelocity();
}
