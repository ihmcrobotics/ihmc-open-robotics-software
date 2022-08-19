package us.ihmc.robotics.screwTheory;

import java.util.stream.Stream;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.interfaces.FixedFrameMomentumBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

public class MomentumCalculator
{
   private final Momentum tempMomentum = new Momentum();
   private final RigidBodyReadOnly[] rigidBodiesInOrders;

   public MomentumCalculator(RigidBodyReadOnly... rigidBodies)
   {
      rigidBodiesInOrders = Stream.of(rigidBodies).filter(body -> body.getInertia() != null).toArray(RigidBodyReadOnly[]::new);
   }

   public MomentumCalculator(RigidBodyReadOnly rootBody)
   {
      this(rootBody.subtreeArray());
   }

   public MomentumCalculator(MultiBodySystemReadOnly input)
   {
      rigidBodiesInOrders = input.getJointsToConsider().stream().map(JointReadOnly::getSuccessor).filter(body -> body.getInertia() != null)
                                 .toArray(RigidBodyReadOnly[]::new);
   }

   public void computeAndPack(FixedFrameMomentumBasics momentum)
   {
      momentum.setToZero();

      for (RigidBodyReadOnly rigidBody : rigidBodiesInOrders)
      {
         SpatialInertiaReadOnly inertia = rigidBody.getInertia();
         tempMomentum.setReferenceFrame(inertia.getReferenceFrame());
         tempMomentum.compute(inertia, rigidBody.getBodyFixedFrame().getTwistOfFrame());
         tempMomentum.changeFrame(momentum.getReferenceFrame());
         momentum.add(tempMomentum);
      }
   }
}
