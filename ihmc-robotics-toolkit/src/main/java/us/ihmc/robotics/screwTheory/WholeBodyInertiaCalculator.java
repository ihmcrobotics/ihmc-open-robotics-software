package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

import java.security.acl.AclEntry;
import java.util.stream.Stream;

public class WholeBodyInertiaCalculator
{
   private final RigidBodyBasics[] rigidBodiesInOrders;
   private final SpatialInertia tempInertia = new SpatialInertia();
   private final ReferenceFrame centerOfMassFrame;

   private final SpatialInertia wholeBodyInertia = new SpatialInertia();

   public WholeBodyInertiaCalculator(ReferenceFrame centerOfMassFrame, RigidBodyBasics... rigidBodies)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      rigidBodiesInOrders = Stream.of(rigidBodies).filter(body -> body.getInertia() != null).toArray(RigidBodyBasics[]::new);
   }

   public WholeBodyInertiaCalculator(ReferenceFrame centerOfMassFrame, RigidBodyBasics rootBody)
   {
      this(centerOfMassFrame, rootBody.subtreeArray());
   }

   public void computeAndPack(SpatialInertiaBasics wholeBodyInertia)
   {
      compute();
      wholeBodyInertia.setIncludingFrame(this.wholeBodyInertia);
   }

   public void compute()
   {
      wholeBodyInertia.setToZero(centerOfMassFrame, centerOfMassFrame);

      for (RigidBodyBasics rigidBody : rigidBodiesInOrders)
      {
         tempInertia.setIncludingFrame(rigidBody.getInertia());
         tempInertia.changeFrame(centerOfMassFrame);
         wholeBodyInertia.add(tempInertia);
      }
   }

   public SpatialInertiaReadOnly getWholeBodyInertia()
   {
      return wholeBodyInertia;
   }
}
