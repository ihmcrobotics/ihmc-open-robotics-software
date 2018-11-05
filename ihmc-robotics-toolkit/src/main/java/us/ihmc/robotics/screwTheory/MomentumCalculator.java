package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;

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
      momentum.getAngularPart().set(zero);
      momentum.getLinearPart().set(zero);

      for (RigidBody rigidBody : rigidBodiesInOrders)
      {
         SpatialInertiaBasics inertia = rigidBody.getInertia();
         rigidBody.getBodyFixedFrame().getTwistOfFrame(tempTwist);
         tempMomentum.setReferenceFrame(inertia.getReferenceFrame());
         tempMomentum.compute(inertia, tempTwist);
         tempMomentum.changeFrame(momentum.getReferenceFrame());
         momentum.add(tempMomentum);
      }
   }
}
