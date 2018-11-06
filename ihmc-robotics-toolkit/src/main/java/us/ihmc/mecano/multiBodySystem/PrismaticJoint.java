package us.ihmc.mecano.multiBodySystem;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;

public class PrismaticJoint extends OneDoFJoint
{
   private final FrameVector3D jointAxis;
   private final Vector3D translation = new Vector3D();

   public PrismaticJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent, Vector3DReadOnly jointAxis)
   {
      super(name, predecessor, transformToParent);
      this.jointAxis = new FrameVector3D(beforeJointFrame, jointAxis);
      this.unitJointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame, new Vector3D(), jointAxis);
   }

   @Override
   protected void updateJointTransform(RigidBodyTransform jointTransform)
   {
      translation.set(jointAxis);
      translation.scale(getQ());
      jointTransform.setTranslationAndIdentityRotation(translation);
   }

   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;

      ReferenceFrame predecessorFrame = getPredecessor().getBodyFixedFrame();
      ReferenceFrame successorFrame = getSuccessor().getBodyFixedFrame();

      unitJointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame, new Vector3D(), jointAxis);

      unitSuccessorTwist = new Twist(unitJointTwist);
      unitSuccessorTwist.setBaseFrame(predecessorFrame);
      unitSuccessorTwist.setBodyFrame(successorFrame);
      unitSuccessorTwist.changeFrame(successorFrame);

      unitPredecessorTwist = new Twist(unitSuccessorTwist);
      unitPredecessorTwist.invert();
      unitPredecessorTwist.changeFrame(predecessorFrame);

      unitJointAcceleration = new SpatialAcceleration(afterJointFrame, beforeJointFrame, afterJointFrame, new Vector3D(), jointAxis);

      unitSuccessorAcceleration = new SpatialAcceleration(unitJointAcceleration);
      unitSuccessorAcceleration.setBaseFrame(predecessorFrame);
      unitSuccessorAcceleration.setBodyFrame(successorFrame);
      unitSuccessorAcceleration.changeFrame(successorFrame);

      unitPredecessorAcceleration = new SpatialAcceleration(unitSuccessorAcceleration);
      unitPredecessorAcceleration.invert();
      unitPredecessorAcceleration.changeFrame(predecessorFrame); // actually, there is relative motion, but not in the directions that matter

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
}
