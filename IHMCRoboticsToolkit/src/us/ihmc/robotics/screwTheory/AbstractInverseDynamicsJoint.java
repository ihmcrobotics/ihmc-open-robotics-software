package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class AbstractInverseDynamicsJoint implements InverseDynamicsJoint
{
   protected final String name;
   protected final RigidBody predecessor;
   protected RigidBody successor;
   protected final ReferenceFrame beforeJointFrame;
   protected GeometricJacobian motionSubspace;

   private final long nameBasedHashCode;

   public AbstractInverseDynamicsJoint(String name, RigidBody predecessor, ReferenceFrame beforeJointFrame)
   {
      nameBasedHashCode = NameBasedHashCodeTools.combineHashCodes(name, predecessor);

      this.name = name;
      this.predecessor = predecessor;
      this.beforeJointFrame = beforeJointFrame;
      predecessor.addChildJoint(this);
   }

   @Override
   public final ReferenceFrame getFrameBeforeJoint()
   {
      return beforeJointFrame;
   }

   @Override
   public final GeometricJacobian getMotionSubspace()
   {
      return motionSubspace;
   }

   @Override
   public final RigidBody getPredecessor()
   {
      return predecessor;
   }

   @Override
   public final RigidBody getSuccessor()
   {
      return successor;
   }

   @Override
   public final String getName()
   {
      return name;
   }

   @Override
   public final void updateFramesRecursively()
   {
      getFrameAfterJoint().update();

      if (successor != null)
      {
         successor.updateFramesRecursively();
      }
   }

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

   
   @Override
   public RigidBodyTransform getOffsetTransform3D()
   {
      return getFrameBeforeJoint().getTransformToParent();
   }
   
   @Override
   public RigidBodyTransform getJointTransform3D()
   {
      return getFrameAfterJoint().getTransformToParent();
   }
   
   public void getOffsetTransform3D(RigidBodyTransform rigidBodyTransformToPack)
   {
      getFrameBeforeJoint().getTransformToParent(rigidBodyTransformToPack);
   }
   
   public void getJointTransform3D(RigidBodyTransform rigidBodyTransformToPack)
   {
      getFrameAfterJoint().getTransformToParent(rigidBodyTransformToPack);
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