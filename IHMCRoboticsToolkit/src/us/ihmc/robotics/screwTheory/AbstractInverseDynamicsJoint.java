package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class AbstractInverseDynamicsJoint implements InverseDynamicsJoint
{
   protected final String name;
   protected final RigidBody predecessor;
   protected RigidBody successor;
   protected final ReferenceFrame beforeJointFrame;
   protected GeometricJacobian motionSubspace;

   public AbstractInverseDynamicsJoint(String name, RigidBody predecessor, ReferenceFrame beforeJointFrame)
   {
      this.name = name;
      this.predecessor = predecessor;
      this.beforeJointFrame = beforeJointFrame;
      predecessor.addChildJoint(this);
   }

   public final ReferenceFrame getFrameBeforeJoint()
   {
      return beforeJointFrame;
   }

   public final GeometricJacobian getMotionSubspace()
   {
      return motionSubspace;
   }

   public final RigidBody getPredecessor()
   {
      return predecessor;
   }

   public final RigidBody getSuccessor()
   {
      return successor;
   }

   public final String getName()
   {
      return name;
   }

   public final void updateFramesRecursively()
   {
      getFrameAfterJoint().update();

      if (successor != null)
      {
         successor.updateFramesRecursively();
      }
   }

   public void getSuccessorTwist(Twist twistToPack)
   {
      getJointTwist(twistToPack);
   
      ReferenceFrame predecessorFrame = predecessor.getBodyFixedFrame();
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
   
      twistToPack.changeBaseFrameNoRelativeTwist(predecessorFrame);
      twistToPack.changeBodyFrameNoRelativeTwist(successorFrame);
      twistToPack.changeFrame(successorFrame);
   }

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

   public void getSuccessorAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      getJointAcceleration(accelerationToPack);
   
      ReferenceFrame predecessorFrame = predecessor.getBodyFixedFrame();
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
   
      accelerationToPack.changeBaseFrameNoRelativeAcceleration(predecessorFrame);
      accelerationToPack.changeBodyFrameNoRelativeAcceleration(successorFrame);
      accelerationToPack.changeFrameNoRelativeMotion(successorFrame);
   }

   public void getDesiredSuccessorAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      getDesiredJointAcceleration(accelerationToPack);
   
      ReferenceFrame predecessorFrame = predecessor.getBodyFixedFrame();
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
   
      accelerationToPack.changeBaseFrameNoRelativeAcceleration(predecessorFrame);
      accelerationToPack.changeBodyFrameNoRelativeAcceleration(successorFrame);
      accelerationToPack.changeFrameNoRelativeMotion(successorFrame);
   }

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

   
   public RigidBodyTransform getOffsetTransform3D()
   {
      return getFrameBeforeJoint().getTransformToParent();
   }
   
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
}