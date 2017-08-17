package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CenterOfMassAccelerationCalculator
{
   private final FramePoint3D comLocation = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D linkLinearMomentumDot = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final RigidBody[] rigidBodies;
   private final RigidBody base;

   public CenterOfMassAccelerationCalculator(RigidBody rootBody, SpatialAccelerationCalculator spatialAccelerationCalculator)
   {
      this(rootBody, ScrewTools.computeSupportAndSubtreeSuccessors(rootBody), spatialAccelerationCalculator);
   }

   public CenterOfMassAccelerationCalculator(RigidBody base, RigidBody[] rigidBodies, SpatialAccelerationCalculator spatialAccelerationCalculator)
   {
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;
      this.rigidBodies = rigidBodies;
      this.base = base;
   }
   
   public void getCoMAcceleration(FrameVector3D comAccelerationToPack)
   {
      boolean firstIteration = true;
      double totalMass = 0.0;

      for (RigidBody rigidBody : rigidBodies)
      {
         double mass = rigidBody.getInertia().getMass();
         rigidBody.getCoMOffset(comLocation);

         spatialAccelerationCalculator.getLinearAccelerationOfBodyFixedPoint(base, rigidBody, comLocation, linkLinearMomentumDot);
         linkLinearMomentumDot.scale(mass);

         if (firstIteration)
            comAccelerationToPack.setIncludingFrame(linkLinearMomentumDot);
         else
            comAccelerationToPack.add(linkLinearMomentumDot);

         totalMass += mass;
         firstIteration = false;
      }

      comAccelerationToPack.scale(1.0 / totalMass);
   }
}
