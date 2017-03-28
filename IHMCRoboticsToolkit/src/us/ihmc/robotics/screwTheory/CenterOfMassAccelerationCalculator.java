package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CenterOfMassAccelerationCalculator
{
   private final FramePoint comLocation = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FrameVector linkLinearMomentumDot = new FrameVector(ReferenceFrame.getWorldFrame());
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
   
   public void getCoMAcceleration(FrameVector comAccelerationToPack)
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
