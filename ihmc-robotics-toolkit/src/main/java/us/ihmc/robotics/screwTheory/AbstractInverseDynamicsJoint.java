package us.ihmc.robotics.screwTheory;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;

/**
 * Base implementation for any {@link InverseDynamicsJoint} that gathers all the basic setup for a
 * joint.
 * 
 */
public abstract class AbstractInverseDynamicsJoint implements InverseDynamicsJoint
{
   /**
    * The name of this joint. Each joint of a robot should have a unique name, but it is not
    * enforced here.
    */
   protected final String name;
   /**
    * The {@code RigidBody} directly connected to this joint and located between this joint and the
    * root body of the robot. The predecessor cannot be {@code null}.
    */
   protected final RigidBody predecessor;
   /**
    * The {@code RigidBody} directly connected to this joint and located between this joint and an
    * end-effector of the robot. The successor should not be {@code null}, but it is not enforced
    * here.
    */
   protected RigidBody successor;
   /**
    * Reference frame fixed to the predecessor and which origin is located at this joint's origin.
    */
   protected final MovingReferenceFrame beforeJointFrame;
   /**
    * Reference frame fixed to the successor and which origin is located at this joint's origin.
    */
   protected final MovingReferenceFrame afterJointFrame;
   /**
    * This is essentially the geometric Jacobian for this joint.
    */
   protected GeometricJacobian motionSubspace;

   /**
    * A secondary unique hash code representing this joint that is computed based on {@link #name}
    * and the predecessor name.
    * <p>
    * This hash code has the benefit of remaining the same when creating several instances of the
    * same robot, such that it can be used to serialize and deserialize joint information.
    * </p>
    */
   private final long nameBasedHashCode;

   /**
    * Creates the abstract layer for a new joint.
    * <p>
    * Note that the {@link #beforeJointFrame} is set to {@code predecessor.getBodyFixedFrame()}.
    * </p>
    * 
    * @param name the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    */
   public AbstractInverseDynamicsJoint(String name, RigidBody predecessor)
   {
      this(name, predecessor, null);
   }

   /**
    * Creates the abstract layer for a new joint.
    * 
    * @param name the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    * @param transformToParent the transform to the frame after the parent joint. Not modified.
    */
   public AbstractInverseDynamicsJoint(String name, RigidBody predecessor, RigidBodyTransform transformToParent)
   {
      nameBasedHashCode = NameBasedHashCodeTools.combineHashCodes(name, predecessor);

      this.name = name;
      this.predecessor = predecessor;
      beforeJointFrame = createBeforeJointFrame(name, predecessor, transformToParent);
      afterJointFrame = createAfterJointFrame(name, beforeJointFrame);
      predecessor.addChildJoint(this);
   }

   private static MovingReferenceFrame createBeforeJointFrame(String jointName, RigidBody parentBody, RigidBodyTransform transformToParent)
   {
      String beforeJointName = "before" + StringUtils.capitalize(jointName);

      MovingReferenceFrame parentFrame;
      if (parentBody.isRootBody())
      {
         parentFrame = parentBody.getBodyFixedFrame();

         /*
          * TODO Special case to keep the beforeJointFrame of the SixDoFJoint to be the
          * elevatorFrame. This should probably removed, might cause reference frame exceptions
          * though.
          */
         if (transformToParent == null)
            return parentFrame;
      }
      else
         parentFrame = parentBody.getParentJoint().getFrameAfterJoint();

      return MovingReferenceFrame.constructFrameFixedInParent(beforeJointName, parentFrame, transformToParent);
   }

   private MovingReferenceFrame createAfterJointFrame(String name, ReferenceFrame beforeJointFrame)
   {
      return new MovingReferenceFrame("after" + StringUtils.capitalize(name), beforeJointFrame)
      {

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            updateJointTransform(transformToParent);
         }

         @Override
         protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
         {
            getJointTwist(twistRelativeToParentToPack);
         }
      };
   }

   /**
    * Override this method to update the given {@code jointTransform} which is used to update
    * {@link #afterJointFrame}.
    * <p>
    * The {@code jointTransform} describes the transform from {@link #afterJointFrame} to
    * {@link #beforeJointFrame} knowing that when the joint is at its 'zero-configuration' these two
    * frames should coincide.
    * </p>
    * 
    * @param jointTransform the transform to update such that it defines the pose of
    *           {@link #afterJointFrame} expressed in {@link #beforeJointFrame}. Modified.
    */
   protected abstract void updateJointTransform(RigidBodyTransform jointTransform);

   /** {@inheritDoc} */
   @Override
   public final MovingReferenceFrame getFrameBeforeJoint()
   {
      return beforeJointFrame;
   }

   /** {@inheritDoc} */
   @Override
   public final MovingReferenceFrame getFrameAfterJoint()
   {
      return afterJointFrame;
   }

   /** {@inheritDoc} */
   @Override
   public final GeometricJacobian getMotionSubspace()
   {
      return motionSubspace;
   }

   /** {@inheritDoc} */
   @Override
   public final RigidBody getPredecessor()
   {
      return predecessor;
   }

   /** {@inheritDoc} */
   @Override
   public final RigidBody getSuccessor()
   {
      return successor;
   }

   /** {@inheritDoc} */
   @Override
   public final String getName()
   {
      return name;
   }

   /** {@inheritDoc} */
   @Override
   public final void updateFramesRecursively()
   {
      beforeJointFrame.update();
      afterJointFrame.update();

      if (successor != null)
      {
         successor.updateFramesRecursively();
      }
   }

   /** {@inheritDoc} */
   @Override
   public void getSuccessorTwist(Twist twistToPack)
   {
      getJointTwist(twistToPack);

      ReferenceFrame predecessorFrame = predecessor.getBodyFixedFrame();
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();

      twistToPack.changeBaseFrameNoRelativeTwist(predecessorFrame);
      twistToPack.changeBodyFrameNoRelativeTwist(successorFrame);
      twistToPack.changeFrame(successorFrame);
   }

   /** {@inheritDoc} */
   @Override
   public void getPredecessorTwist(Twist twistToPack)
   {
      getJointTwist(twistToPack);

      ReferenceFrame predecessorFrame = predecessor.getBodyFixedFrame();
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();

      twistToPack.changeBaseFrameNoRelativeTwist(predecessorFrame);
      twistToPack.changeBodyFrameNoRelativeTwist(successorFrame);
      twistToPack.invert();
      twistToPack.changeFrame(predecessorFrame);
   }

   /** {@inheritDoc} */
   @Override
   public void getSuccessorAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      getJointAcceleration(accelerationToPack);

      ReferenceFrame predecessorFrame = predecessor.getBodyFixedFrame();
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();

      accelerationToPack.changeBaseFrameNoRelativeAcceleration(predecessorFrame);
      accelerationToPack.changeBodyFrameNoRelativeAcceleration(successorFrame);
      accelerationToPack.changeFrameNoRelativeMotion(successorFrame);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredSuccessorAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      getDesiredJointAcceleration(accelerationToPack);

      ReferenceFrame predecessorFrame = predecessor.getBodyFixedFrame();
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();

      accelerationToPack.changeBaseFrameNoRelativeAcceleration(predecessorFrame);
      accelerationToPack.changeBodyFrameNoRelativeAcceleration(successorFrame);
      accelerationToPack.changeFrameNoRelativeMotion(successorFrame);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredPredecessorAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      getDesiredJointAcceleration(accelerationToPack);

      ReferenceFrame predecessorFrame = predecessor.getBodyFixedFrame();
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();

      accelerationToPack.changeBaseFrameNoRelativeAcceleration(predecessorFrame);
      accelerationToPack.changeBodyFrameNoRelativeAcceleration(successorFrame);
      accelerationToPack.invert();
      accelerationToPack.changeFrameNoRelativeMotion(predecessorFrame);
   }

   /**
    * Returns the offset from the frame before this joint to the frame after this parent joint.
    * <p>
    * The returned {@code RigidBodyTransform} should not be modified.
    * </p>
    * 
    * @return this joint offset with respect to its parent joint as a transform.
    */
   @Override
   public RigidBodyTransform getOffsetTransform3D()
   {
      return getFrameBeforeJoint().getTransformToParent();
   }

   /**
    * Returns the configuration of this joint as a transform.
    * <p>
    * The returned {@code RigidBodyTransform} should not be modified.
    * </p>
    * 
    * @return the pose of the frame after joint expressed in the frame before joint.
    */
   @Override
   public RigidBodyTransform getJointTransform3D()
   {
      return getFrameAfterJoint().getTransformToParent();
   }

   /**
    * Packs the offset from the frame before this joint to the frame after this parent joint.
    * 
    * @param jointOffsetTransformToPack the transform in which this joint's offset is stored.
    *           Modified.
    */
   public void getOffsetTransform3D(RigidBodyTransform jointOffsetTransformToPack)
   {
      getFrameBeforeJoint().getTransformToParent(jointOffsetTransformToPack);
   }

   /**
    * Packs the configuration of this joint as a transform.
    * 
    * @param jointConfigurationTransformToPack transform in which the pose of the frame after joint
    *           expressed in the frame before joint is stored. Modified.
    */
   public void getJointTransform3D(RigidBodyTransform jointConfigurationTransformToPack)
   {
      getFrameAfterJoint().getTransformToParent(jointConfigurationTransformToPack);
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + " " + getName();
   }

   @Override
   public long getNameBasedHashCode()
   {
      return nameBasedHashCode;
   }
}