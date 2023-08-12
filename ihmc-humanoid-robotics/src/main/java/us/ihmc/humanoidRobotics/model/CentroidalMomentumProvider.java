package us.ihmc.humanoidRobotics.model;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.interfaces.FixedFrameMomentumBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.MomentumCalculator;

public interface CentroidalMomentumProvider
{
   default void updateMomentumState()
   {}

   FrameVector3DReadOnly getCentroidalLinearMomentum();

   FrameVector3DReadOnly getCentroidalAngularMomentum();

   public static CentroidalMomentumProvider createJacobianBasedStateCalculator(RigidBodyReadOnly elevator)
   {
      return new CentroidalMomentumProvider()
      {
         private final MomentumCalculator momentumCalculator = new MomentumCalculator(MultiBodySystemTools.collectSuccessors(elevator.getParentJoint()));

         private final FixedFrameMomentumBasics robotMomentum = new Momentum();

         @Override
         public void updateMomentumState()
         {
            momentumCalculator.computeAndPack(robotMomentum);
         }

         @Override
         public FrameVector3DReadOnly getCentroidalLinearMomentum()
         {
            return robotMomentum.getLinearPart();
         }

         @Override
         public FrameVector3DReadOnly getCentroidalAngularMomentum()
         {
            return robotMomentum.getAngularPart();
         }
      };
   }
}
