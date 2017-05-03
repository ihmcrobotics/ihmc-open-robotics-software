package us.ihmc.robotics.screwTheory;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class AbstractInverseDynamicsJoint implements InverseDynamicsJoint
{
   protected final String name;
   protected final RigidBody predecessor;
   protected RigidBody successor;
   protected final MovingReferenceFrame beforeJointFrame;
   protected final MovingReferenceFrame afterJointFrame;
   protected GeometricJacobian motionSubspace;

   private final long nameBasedHashCode;

   public AbstractInverseDynamicsJoint(String name, RigidBody predecessor)
   {
      this(name, predecessor, null);
   }

   public AbstractInverseDynamicsJoint(String name, RigidBody predecessor, RigidBodyTransform transformToParent)
   {
      nameBasedHashCode = NameBasedHashCodeTools.combineHashCodes(name, predecessor);

      this.name = name;
      this.predecessor = predecessor;
      beforeJointFrame = createBeforeJointFrame(name, predecessor, transformToParent);
      afterJointFrame = createAfterJointFrame(name, beforeJointFrame);
      predecessor.addChildJoint(this);
   }

   public static MovingReferenceFrame createBeforeJointFrame(String jointName, RigidBody parentBody, RigidBodyTransform transformToParent)
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
         private static final long serialVersionUID = 4779423307372501426L;

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

   protected abstract void updateJointTransform(RigidBodyTransform jointTransform);

   @Override
   public final MovingReferenceFrame getFrameBeforeJoint()
   {
      return beforeJointFrame;
   }

   @Override
   public final MovingReferenceFrame getFrameAfterJoint()
   {
      return afterJointFrame;
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
      beforeJointFrame.update();
      afterJointFrame.update();

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