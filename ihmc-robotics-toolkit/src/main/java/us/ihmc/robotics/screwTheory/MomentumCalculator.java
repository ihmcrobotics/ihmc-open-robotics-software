package us.ihmc.robotics.screwTheory;

import java.util.stream.Stream;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.FixedFrameMomentumBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

public class MomentumCalculator
{
   private final RigidBodyReadOnly[] rigidBodiesInOrders;

   private final Twist bodyTwist = new Twist();
   private final Momentum bodyMomentum = new Momentum();

   private ReferenceFrame baseFrame = null;

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

   public void setBaseFrame(ReferenceFrame baseFrame)
   {
      this.baseFrame = baseFrame;
   }

   public void computeAndPack(FixedFrameMomentumBasics momentum)
   {
      momentum.setToZero();

      if (baseFrame == null)
      {
         for (RigidBodyReadOnly rigidBody : rigidBodiesInOrders)
         {
            SpatialInertiaReadOnly inertia = rigidBody.getInertia();
            bodyMomentum.setReferenceFrame(inertia.getReferenceFrame());
            bodyMomentum.compute(inertia, rigidBody.getBodyFixedFrame().getTwistOfFrame());
            bodyMomentum.changeFrame(momentum.getReferenceFrame());
            momentum.add(bodyMomentum);
         }
      }
      else
      {
         for (RigidBodyReadOnly rigidBody : rigidBodiesInOrders)
         {
            SpatialInertiaReadOnly inertia = rigidBody.getInertia();
            bodyMomentum.setReferenceFrame(inertia.getReferenceFrame());
            rigidBody.getBodyFixedFrame().getTwistRelativeToOther(baseFrame, bodyTwist);
            bodyMomentum.compute(inertia, bodyTwist);
            bodyMomentum.changeFrame(momentum.getReferenceFrame());
            momentum.add(bodyMomentum);
         }
      }
   }
}
