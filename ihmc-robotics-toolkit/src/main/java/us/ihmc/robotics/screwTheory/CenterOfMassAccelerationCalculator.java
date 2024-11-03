package us.ihmc.robotics.screwTheory;

import java.util.stream.Stream;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.SpatialAccelerationCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class CenterOfMassAccelerationCalculator
{
   private final FramePoint3D comLocation = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D linkLinearMomentumDot = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final RigidBodyBasics[] rigidBodies;
   private final RigidBodyBasics base;

   public CenterOfMassAccelerationCalculator(RigidBodyBasics rootBody, SpatialAccelerationCalculator spatialAccelerationCalculator)
   {
      this(rootBody, rootBody.subtreeArray(), spatialAccelerationCalculator);
   }

   public CenterOfMassAccelerationCalculator(RigidBodyBasics base, RigidBodyBasics[] rigidBodies, SpatialAccelerationCalculator spatialAccelerationCalculator)
   {
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;
      this.rigidBodies = Stream.of(rigidBodies).filter(body -> body.getInertia() != null).toArray(RigidBodyBasics[]::new);
      this.base = base;
   }
   
   public void getCoMAcceleration(FrameVector3D comAccelerationToPack)
   {
      boolean firstIteration = true;
      double totalMass = 0.0;

      for (RigidBodyBasics rigidBody : rigidBodies)
      {
         double mass = rigidBody.getInertia().getMass();
         rigidBody.getCenterOfMass(comLocation);

         linkLinearMomentumDot.setIncludingFrame(spatialAccelerationCalculator.getLinearAccelerationOfBodyFixedPoint(base, rigidBody, comLocation));
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
