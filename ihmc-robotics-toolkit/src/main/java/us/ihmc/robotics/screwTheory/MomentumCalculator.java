package us.ihmc.robotics.screwTheory;

import java.util.stream.Stream;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Momentum;
<<<<<<< HEAD
import us.ihmc.mecano.spatial.interfaces.FixedFrameMomentumBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
=======
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.MomentumBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
>>>>>>> set up a whole-body inertia calculator

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
