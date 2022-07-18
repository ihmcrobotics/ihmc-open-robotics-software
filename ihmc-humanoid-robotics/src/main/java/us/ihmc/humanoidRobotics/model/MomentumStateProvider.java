package us.ihmc.humanoidRobotics.model;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;

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
public interface MomentumStateProvider
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

   FrameVector3DReadOnly getModifiedCenterOfMassVelocity();

   /**
    * Factory to create a kinematic based center of mass state calculator using a
    * {@link CenterOfMassJacobian}.
    * <p>
    * This is the typical way to create a {@link MomentumStateProvider} when no other alternative
    * is available.
    * </p>
    * 
    * @param elevator       the root body of the multi-body system to estimate the center of mass state
    *                       of.
    * @param referenceFrame the frame in which the center of mass state should be expressed.
    * @return the new center of mass state provider.
    */
   public static MomentumStateProvider createMomentumBasedStateCalculator(RigidBodyReadOnly elevator, ReferenceFrame worldFrame, ReferenceFrame centerOfMassFrame, double omega0)
   {
      return new MomentumStateProvider()
      {
         private final CenterOfMassJacobian jacobian = new CenterOfMassJacobian(elevator, worldFrame);
         private final CentroidalMomentumCalculator centroidalMomentumCalculator = new CentroidalMomentumCalculator(elevator, centerOfMassFrame);
         private final double desiredHeight = 9.81 / omega0 / omega0;

         @Override
         public void updateState()
         {
            jacobian.reset();
            centroidalMomentumCalculator.reset();
         }

         @Override
         public FramePoint3DReadOnly getCenterOfMassPosition()
         {
            return jacobian.getCenterOfMass();
         }

         @Override
         public FrameVector3DReadOnly getModifiedCenterOfMassVelocity()
         {  
            FrameVector3DReadOnly comVel = jacobian.getCenterOfMassVelocity();
            MomentumReadOnly centroidalMomentum = centroidalMomentumCalculator.getMomentum();

            double modifiedComVelX = (centroidalMomentum.getLinearPart().getX() + centroidalMomentum.getAngularPart().getY() / desiredHeight) / centroidalMomentumCalculator.getTotalMass();
            double modifiedComVelY = (centroidalMomentum.getLinearPart().getY() - centroidalMomentum.getAngularPart().getX() / desiredHeight) / centroidalMomentumCalculator.getTotalMass();
            double modifiedComVelZ = comVel.getZ();
            
            return new FrameVector3D(worldFrame, modifiedComVelX, modifiedComVelY, modifiedComVelZ);
         }
      };
   }
}
