package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.tuple3D.Vector3D;

public class MomentumCalculator
{
   private final TwistCalculator twistCalculator;
   private final Twist tempTwist = new Twist();
   private final Momentum tempMomentum = new Momentum();
   private final Vector3D zero = new Vector3D();
   private final RigidBody[] rigidBodiesInOrders;

   public MomentumCalculator(TwistCalculator twistCalculator, RigidBody... rigidBodies)
   {
      this.twistCalculator = twistCalculator;
      rigidBodiesInOrders = rigidBodies;
   }

   public MomentumCalculator(TwistCalculator twistCalculator)
   {
      this(twistCalculator, ScrewTools.computeSupportAndSubtreeSuccessors(twistCalculator.getRootBody()));
   }

   public void computeAndPack(Momentum momentum)
   {
      momentum.setAngularPart(zero);
      momentum.setLinearPart(zero);

      for (RigidBody rigidBody : rigidBodiesInOrders)
      {
         RigidBodyInertia inertia = rigidBody.getInertia();
         twistCalculator.getTwistOfBody(tempTwist, rigidBody);
         tempMomentum.compute(inertia, tempTwist);
         tempMomentum.changeFrame(momentum.getExpressedInFrame());
         momentum.add(tempMomentum);
      }
   }
}
