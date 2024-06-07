package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Momentum;

public class WholeBodyAngularVelocityCalculator
{
   private final WholeBodyInertiaCalculator wholeBodyInertiaCalculator;
   private final MomentumCalculator momentumCalculator;

   private final Momentum robotMomentum = new Momentum();

   private final FrameVector3D linearMomentum = new FrameVector3D();
   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final FrameVector3D wholeBodyAngularVelocity = new FrameVector3D();

   public WholeBodyAngularVelocityCalculator(ReferenceFrame centerOfMassFrame, YoGraphicsListRegistry graphicsListRegistry, RigidBodyBasics... rigidBodies)
   {
      this.wholeBodyInertiaCalculator = new WholeBodyInertiaCalculator(centerOfMassFrame, null, rigidBodies);
      this.momentumCalculator = new MomentumCalculator(rigidBodies);
      robotMomentum.setReferenceFrame(centerOfMassFrame);
   }

   public WholeBodyAngularVelocityCalculator(ReferenceFrame centerOfMassFrame, RigidBodyBasics... rigidBodies)
   {
      this(centerOfMassFrame, null, rigidBodies);
   }

   public WholeBodyAngularVelocityCalculator(ReferenceFrame centerOfMassFrame, YoGraphicsListRegistry graphicsListRegistry, RigidBodyBasics rootBody)
   {
      this(centerOfMassFrame, graphicsListRegistry, rootBody.subtreeArray());
   }

   public FrameVector3DReadOnly getLinearMomentum()
   {
      return linearMomentum;
   }

   public FrameVector3DReadOnly getAngularMomentum()
   {
      return angularMomentum;
   }

   public FrameVector3DReadOnly getWholeBodyAngularVelocity()
   {
      return wholeBodyAngularVelocity;
   }

   public void compute()
   {
      wholeBodyInertiaCalculator.compute();
      momentumCalculator.computeAndPack(robotMomentum);

      linearMomentum.setIncludingFrame(robotMomentum.getLinearPart());
      angularMomentum.setIncludingFrame(robotMomentum.getAngularPart());
      wholeBodyAngularVelocity.setToZero(angularMomentum.getReferenceFrame());

      wholeBodyInertiaCalculator.getWholeBodyInertia().getMomentOfInertia().inverseTransform(angularMomentum, wholeBodyAngularVelocity);
   }

}
