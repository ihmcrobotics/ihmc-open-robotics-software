package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class PrismaticJoint extends OneDoFJoint
{
   private final FrameVector3D jointAxis;
   private final Vector3D translation = new Vector3D();

   public PrismaticJoint(String name, RigidBody predecessor, RigidBodyTransform transformToParent, Vector3DReadOnly jointAxis)
   {
      super(name, predecessor, transformToParent);
      this.jointAxis = new FrameVector3D(beforeJointFrame, jointAxis);
      this.unitJointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame, jointAxis, new Vector3D());
   }

   @Override
   protected void updateJointTransform(RigidBodyTransform jointTransform)
   {
      translation.set(jointAxis);
      translation.scale(getQ());
      jointTransform.setTranslationAndIdentityRotation(translation);
   }

   @Override
   public void setSuccessor(RigidBody successor)
   {
      this.successor = successor;

      ReferenceFrame predecessorFrame = getPredecessor().getBodyFixedFrame();
      ReferenceFrame successorFrame = getSuccessor().getBodyFixedFrame();

      unitJointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame, jointAxis.getVector(), new Vector3D());

      unitSuccessorTwist = new Twist(unitJointTwist);
      unitSuccessorTwist.changeBaseFrameNoRelativeTwist(predecessorFrame);
      unitSuccessorTwist.changeBodyFrameNoRelativeTwist(successorFrame);
      unitSuccessorTwist.changeFrame(successorFrame);

      unitPredecessorTwist = new Twist(unitSuccessorTwist);
      unitPredecessorTwist.invert();
      unitPredecessorTwist.changeFrame(predecessorFrame);

      unitJointAcceleration = new SpatialAccelerationVector(afterJointFrame, beforeJointFrame, afterJointFrame, jointAxis.getVector(), new Vector3D());

      unitSuccessorAcceleration = new SpatialAccelerationVector(unitJointAcceleration);
      unitSuccessorAcceleration.changeBaseFrameNoRelativeAcceleration(predecessorFrame);
      unitSuccessorAcceleration.changeBodyFrameNoRelativeAcceleration(successorFrame);
      unitSuccessorAcceleration.changeFrameNoRelativeMotion(successorFrame);

      unitPredecessorAcceleration = new SpatialAccelerationVector(unitSuccessorAcceleration);
      unitPredecessorAcceleration.invert();
      unitPredecessorAcceleration.changeFrameNoRelativeMotion(predecessorFrame); // actually, there is relative motion, but not in the directions that matter

      setMotionSubspace();
   }

   @Override
   public FrameVector3D getJointAxis()
   {
      return new FrameVector3D(jointAxis);
   }

   @Override
   public void getJointAxis(FrameVector3D axisToPack)
   {
      axisToPack.setIncludingFrame(jointAxis);
   }

   @Override
   public boolean isPassiveJoint()
   {
      return false;
   }
}
