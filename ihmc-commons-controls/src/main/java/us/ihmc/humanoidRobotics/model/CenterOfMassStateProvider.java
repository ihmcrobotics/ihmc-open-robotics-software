package us.ihmc.humanoidRobotics.model;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

/**
 * This interface can be used to wrap the source of the computation of the center of mass state.
 * <p>
 * It is typically used in the different modules of the humanoid controller and allows to swap the
 * source of the center of mass state. By default, the controller can use a kinematics based
 * calculator via {@link #createJacobianBasedStateCalculator(RigidBodyReadOnly, ReferenceFrame)}.
 * Alternatively, the center of mass state can be provided externally, for instance by a dedicated
 * state estimator using additional sensors.
 * </p>
 */
public interface CenterOfMassStateProvider
{
   /**
    * Indicates that the center of mass state should be updated.
    * <p>
    * Should be called once per control tick.
    * </p>
    */
   default void updateState()
   {

   }

   FramePoint3DReadOnly getCenterOfMassPosition();

   FrameVector3DReadOnly getCenterOfMassVelocity();

   /**
    * Factory to create a kinematic based center of mass state calculator using a
    * {@link CenterOfMassJacobian}.
    * <p>
    * This is the typical way to create a {@link CenterOfMassStateProvider} when no other alternative
    * is available.
    * </p>
    * 
    * @param elevator       the root body of the multi-body system to estimate the center of mass state
    *                       of.
    * @param referenceFrame the frame in which the center of mass state should be expressed.
    * @return the new center of mass state provider.
    */
   public static CenterOfMassStateProvider createJacobianBasedStateCalculator(RigidBodyReadOnly elevator, ReferenceFrame referenceFrame)
   {
      return new CenterOfMassStateProvider()
      {
         private final CenterOfMassJacobian jacobian = new CenterOfMassJacobian(elevator, referenceFrame);

         @Override
         public void updateState()
         {
            jacobian.reset();
         }

         @Override
         public FramePoint3DReadOnly getCenterOfMassPosition()
         {
            return jacobian.getCenterOfMass();
         }

         @Override
         public FrameVector3DReadOnly getCenterOfMassVelocity()
         {
            return jacobian.getCenterOfMassVelocity();
         }
      };
   }
}
