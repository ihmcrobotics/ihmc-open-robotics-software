package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collector;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;

public class CombinedRigidBodyTwistProviders implements RigidBodyTwistProvider
{
   private final ReferenceFrame inertialFrame;
   private final List<RigidBodyTwistProvider> rigidBodyTwistProviders = new ArrayList<>();

   public CombinedRigidBodyTwistProviders(ReferenceFrame inertialFrame)
   {
      this.inertialFrame = inertialFrame;
   }

   public CombinedRigidBodyTwistProviders(CombinedRigidBodyTwistProviders other)
   {
      inertialFrame = other.inertialFrame;
      rigidBodyTwistProviders.addAll(other.rigidBodyTwistProviders);
   }

   public void addAll(CombinedRigidBodyTwistProviders other)
   {
      inertialFrame.checkReferenceFrameMatch(other.inertialFrame);
      rigidBodyTwistProviders.addAll(other.rigidBodyTwistProviders);
   }

   public void addAll(Collection<? extends RigidBodyTwistProvider> rigidBodyTwistProviders)
   {
      rigidBodyTwistProviders.forEach(this::add);
   }

   public void add(RigidBodyTwistProvider rigidBodyTwistProvider)
   {
      if (rigidBodyTwistProvider != null)
         rigidBodyTwistProviders.add(rigidBodyTwistProvider);
   }

   public void removeAll(CombinedRigidBodyTwistProviders other)
   {
      rigidBodyTwistProviders.removeAll(other.rigidBodyTwistProviders);
   }

   public void removeAll(Collection<? extends RigidBodyTwistProvider> rigidBodyTwistProviders)
   {
      rigidBodyTwistProviders.forEach(this::remove);
   }

   public void remove(RigidBodyTwistProvider rigidBodyTwistProvider)
   {
      if (rigidBodyTwistProvider != null)
         rigidBodyTwistProviders.remove(rigidBodyTwistProvider);
   }

   private final Twist combinedTwist = new Twist();
   private final Twist twist = new Twist();
   private final FrameVector3D linearVelocity = new FrameVector3D();

   @Override
   public TwistReadOnly getTwistOfBody(RigidBodyReadOnly body)
   {
      combinedTwist.setToZero(body.getBodyFixedFrame(), getInertialFrame(), body.getBodyFixedFrame());

      for (RigidBodyTwistProvider provider : rigidBodyTwistProviders)
      {
         TwistReadOnly singleTwist = provider.getTwistOfBody(body);
         if (singleTwist != null)
            combinedTwist.add((SpatialVectorReadOnly) singleTwist);
      }
      return combinedTwist;
   }

   @Override
   public TwistReadOnly getRelativeTwist(RigidBodyReadOnly base, RigidBodyReadOnly body)
   {
      MovingReferenceFrame bodyFrame = body.getBodyFixedFrame();

      TwistReadOnly immutableBaseTwist = getTwistOfBody(base);
      if (immutableBaseTwist == null)
         return null;

      twist.setIncludingFrame(immutableBaseTwist);

      TwistReadOnly immutableBodyTwist = getTwistOfBody(body);
      if (immutableBodyTwist == null)
         return null;

      twist.changeFrame(bodyFrame);
      twist.sub(immutableBodyTwist);
      twist.invert();

      return twist;
   }

   @Override
   public FrameVector3DReadOnly getLinearVelocityOfBodyFixedPoint(RigidBodyReadOnly base, RigidBodyReadOnly body, FramePoint3DReadOnly bodyFixedPoint)
   {
      if (base != null)
         getRelativeTwist(base, body).getLinearVelocityAt(bodyFixedPoint, linearVelocity);
      else
         getTwistOfBody(body).getLinearVelocityAt(bodyFixedPoint, linearVelocity);
      return linearVelocity;
   }

   @Override
   public ReferenceFrame getInertialFrame()
   {
      return inertialFrame;
   }

   public static Collector<RigidBodyTwistProvider, CombinedRigidBodyTwistProviders, CombinedRigidBodyTwistProviders> collect(ReferenceFrame inertialFrame)
   {
      return Collector.of(() -> new CombinedRigidBodyTwistProviders(inertialFrame), CombinedRigidBodyTwistProviders::add, (left, right) ->
      {
         left.addAll(right);
         return left;
      }, Collector.Characteristics.IDENTITY_FINISH);
   }

   public static Collector<ImpulseBasedConstraintCalculator, CombinedRigidBodyTwistProviders, CombinedRigidBodyTwistProviders> collectFromCalculator(ReferenceFrame inertialFrame)
   {
      return Collector.of(() -> new CombinedRigidBodyTwistProviders(inertialFrame), (providers, calculator) ->
      {
         for (int i = 0; i < calculator.getNumberOfRobotsInvolved(); i++)
            providers.add(calculator.getRigidBodyTwistChangeProvider(i));
      }, (left, right) ->
      {
         left.addAll(right);
         return left;
      }, Collector.Characteristics.IDENTITY_FINISH);
   }
}
