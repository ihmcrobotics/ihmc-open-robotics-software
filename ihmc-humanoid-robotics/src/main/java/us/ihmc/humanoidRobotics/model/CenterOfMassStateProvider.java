package us.ihmc.humanoidRobotics.model;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

public interface CenterOfMassStateProvider
{
   default void updateState()
   {

   }

   FramePoint3DReadOnly getCenterOfMassPosition();

   FrameVector3DReadOnly getCenterOfMassVelocity();

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
