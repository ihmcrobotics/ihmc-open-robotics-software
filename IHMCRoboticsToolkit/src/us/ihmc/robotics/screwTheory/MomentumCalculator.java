package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.tuple3D.Vector3D;

public class MomentumCalculator
{
   private final Twist tempTwist = new Twist();
   private final Momentum tempMomentum = new Momentum();
   private final Vector3D zero = new Vector3D();
   private final RigidBody[] rigidBodiesInOrders;

   public MomentumCalculator(RigidBody... rigidBodies)
   {
      rigidBodiesInOrders = rigidBodies;
   }

   public MomentumCalculator(RigidBody rootBody)
   {
      this(ScrewTools.computeSupportAndSubtreeSuccessors(rootBody));
   }

   public void computeAndPack(Momentum momentum)
   {
      momentum.setAngularPart(zero);
      momentum.setLinearPart(zero);

      for (RigidBody rigidBody : rigidBodiesInOrders)
      {
         RigidBodyInertia inertia = rigidBody.getInertia();
         rigidBody.getBodyFixedFrame().getTwistOfFrame(tempTwist);
         tempMomentum.compute(inertia, tempTwist);
         tempMomentum.changeFrame(momentum.getExpressedInFrame());
         momentum.add(tempMomentum);
      }
   }
}
