package us.ihmc.robotModels.description;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.screwTheory.InvertedFourBarJoint;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;

public class InvertedFourBarJointDefinition extends OneDoFJointDefinition
{
   private RevoluteJointDefinition[] fourBarJoints;
   private int masterJointIndex;

   public InvertedFourBarJointDefinition(String name)
   {
      super(name);
   }

   public void setMasterJointIndex(int masterJointIndex)
   {
      this.masterJointIndex = masterJointIndex;
   }

   public void setFourBarJoints(RevoluteJointDefinition[] fourBarJoints)
   {
      this.fourBarJoints = fourBarJoints;
   }

   public int getMasterJointIndex()
   {
      return masterJointIndex;
   }

   public RevoluteJointDefinition[] getFourBarJoints()
   {
      return fourBarJoints;
   }

   @Override
   public InvertedFourBarJoint toJoint(RigidBodyBasics predecessor)
   {
      RevoluteJointBasics jointA = null, jointB = null, jointC = null, jointD = null;
      RevoluteJointDefinition jointADefinition = null, jointBDefinition = null, jointCDefinition = null, jointDDefinition = null;

      RevoluteJointDefinition masterJointDefinition = fourBarJoints[masterJointIndex];
      RevoluteJointDefinition loopClosureJointDefinition = null;

      for (RevoluteJointDefinition revoluteJointDefinition : fourBarJoints)
      {
         if (revoluteJointDefinition.isLoopClosure())
         {
            loopClosureJointDefinition = revoluteJointDefinition;
            continue;
         }

         if (revoluteJointDefinition.getPredecessor().getName().equals(predecessor.getName()))
         {
            if (jointA == null)
            {
               jointADefinition = revoluteJointDefinition;
               jointA = jointADefinition.toJoint(predecessor);
            }
            else
            {
               jointBDefinition = revoluteJointDefinition;
               jointB = jointBDefinition.toJoint(predecessor);
            }
         }
      }

      if (loopClosureJointDefinition == null)
      {
         throw new IllegalStateException("Unexpected four-bar configuration, does not have a loop closure.");
      }

      for (RevoluteJointDefinition revoluteJointDefinition : fourBarJoints)
      {
         if (revoluteJointDefinition.isLoopClosure())
            continue;

         if (revoluteJointDefinition.getParentJoint() == jointADefinition)
            jointDDefinition = revoluteJointDefinition;
         else if (revoluteJointDefinition.getParentJoint() == jointBDefinition)
            jointCDefinition = revoluteJointDefinition;
      }

      RigidBodyBasics bodyAD = jointADefinition.getSuccessor().toRigidBody(jointA);
      RigidBodyBasics bodyCD = null;

      if (jointDDefinition != null)
      {
         jointD = jointDDefinition.toJoint(bodyAD);
         bodyCD = jointDDefinition.getSuccessor().toRigidBody(jointD);
      }

      RigidBodyBasics bodyBC = jointBDefinition.getSuccessor().toRigidBody(jointB);

      if (jointCDefinition != null)
      {
         jointC = jointCDefinition.toJoint(bodyBC);
         bodyCD = jointCDefinition.getSuccessor().toRigidBody(jointC);
      }

      if ((jointC == null) == (jointD == null))
         throw new IllegalStateException("Unexpected four-bar configuration");

      if (jointC == null)
      {
         jointCDefinition = loopClosureJointDefinition;
         if (jointCDefinition.getParentJoint() == jointDDefinition)
            jointC = createLoopClosureJoint(bodyCD, bodyBC, jointCDefinition);
         else
            jointC = createLoopClosureJoint(bodyBC, bodyCD, jointCDefinition);
      }
      else
      {
         jointDDefinition = loopClosureJointDefinition;
         if (jointDDefinition.getParentJoint() == jointCDefinition)
            jointD = createLoopClosureJoint(bodyCD, bodyAD, jointDDefinition);
         else
            jointD = createLoopClosureJoint(bodyAD, bodyCD, jointDDefinition);
      }

      RevoluteJointBasics[] fourBarRevoluteJoints = new RevoluteJointBasics[] {jointA, jointB, jointC, jointD};
      int masterJointIndex = -1;
      if (masterJointDefinition == jointADefinition)
         masterJointIndex = 0;
      else if (masterJointDefinition == jointBDefinition)
         masterJointIndex = 1;
      else if (masterJointDefinition == jointCDefinition)
         masterJointIndex = 2;
      else if (masterJointDefinition == jointDDefinition)
         masterJointIndex = 3;
      return new InvertedFourBarJoint(getName(), fourBarRevoluteJoints, masterJointIndex);
   }

   private RevoluteJointBasics createLoopClosureJoint(RigidBodyBasics predecessor, RigidBodyBasics successor, RevoluteJointDefinition jointDefinition)
   {
      if (!jointDefinition.isLoopClosure())
         throw new IllegalArgumentException("This is not a loop closure joint: " + jointDefinition);
      if (!jointDefinition.getSuccessor().getName().equals(successor.getName()))
         throw new IllegalArgumentException("Successor mismatch.");
      RevoluteJointBasics joint = jointDefinition.toJoint(predecessor);
      joint.setupLoopClosure(successor, new RigidBodyTransform(jointDefinition.getLoopClosureDefinition().getTransformToSuccessorParent()));
      return joint;
   }

   @Override
   public JointDefinition copy()
   {
      throw new UnsupportedOperationException();
   }
}
