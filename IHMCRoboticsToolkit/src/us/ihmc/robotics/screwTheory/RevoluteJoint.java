package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class RevoluteJoint extends OneDoFJoint
{
   private final FrameVector3D jointAxis;
   private final AxisAngle axisAngle = new AxisAngle();

   public RevoluteJoint(String name, RigidBody predecessor, Vector3DReadOnly jointAxis)
   {
      this(name, predecessor, null, jointAxis);
   }

   public RevoluteJoint(String name, RigidBody predecessor, RigidBodyTransform transformToParent, Vector3DReadOnly jointAxis)
   {
      super(name, predecessor, transformToParent);
      this.jointAxis = new FrameVector3D(beforeJointFrame, jointAxis);
      this.unitJointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame, new Vector3D(), jointAxis);
   }

   @Override
   protected void updateJointTransform(RigidBodyTransform jointTransform)
   {
      axisAngle.set(jointAxis.getVector(), getQ());
      jointTransform.setRotationAndZeroTranslation(axisAngle);
   }

   @Override
   public void setSuccessor(RigidBody successor)
   {
      this.successor = successor;

      ReferenceFrame predecessorFrame = getPredecessor().getBodyFixedFrame();
      ReferenceFrame successorFrame = getSuccessor().getBodyFixedFrame();

      unitJointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame, new Vector3D(), jointAxis.getVector());

      unitSuccessorTwist = new Twist(unitJointTwist);
      unitSuccessorTwist.changeBaseFrameNoRelativeTwist(predecessorFrame);
      unitSuccessorTwist.changeBodyFrameNoRelativeTwist(successorFrame);
      unitSuccessorTwist.changeFrame(successorFrame);

      unitPredecessorTwist = new Twist(unitSuccessorTwist);
      unitPredecessorTwist.invert();
      unitPredecessorTwist.changeFrame(predecessorFrame);

      unitJointAcceleration = new SpatialAccelerationVector(afterJointFrame, beforeJointFrame, afterJointFrame, new Vector3D(), jointAxis.getVector());

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
