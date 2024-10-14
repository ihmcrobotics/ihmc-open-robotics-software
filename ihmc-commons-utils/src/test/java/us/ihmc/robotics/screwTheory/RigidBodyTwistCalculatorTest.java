package us.ihmc.robotics.screwTheory;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.List;
import java.util.Random;

public class RigidBodyTwistCalculatorTest
{

   public static final int ITERATIONS = 1000;
   public static final double EPSILON = 1.0e-12;

   @Test
   public void testAgainstMovingReferenceFrame()
   {
      Random random = new Random(1776L);
      Twist expectedTwist = new Twist();

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, 10);
         RigidBodyBasics root = joints.get(0).getPredecessor();
         for (JointStateType stateType : JointStateType.values())
         {
            MultiBodySystemRandomTools.nextState(random, stateType, joints);
         }

         root.updateFramesRecursively();

         RigidBodyTwistCalculator twistCalculator = new RigidBodyTwistCalculator(MultiBodySystemReadOnly.toMultiBodySystemInput(root));
         twistCalculator.setJointVelocityAccessor((joint, matrixToPack) -> joint.getJointVelocity(0, matrixToPack));

         for (JointBasics joint : joints)
         {
            RigidBodyBasics body = joint.getSuccessor();
            MecanoTestTools.assertTwistEquals(body.getBodyFixedFrame().getTwistOfFrame(), twistCalculator.getTwistOfBody(body), EPSILON);

            for (JointBasics otherJoint : joints)
            {
               RigidBodyBasics otherBody = otherJoint.getSuccessor();
               body.getBodyFixedFrame().getTwistRelativeToOther(otherBody.getBodyFixedFrame(), expectedTwist);
               MecanoTestTools.assertTwistEquals(expectedTwist, twistCalculator.getRelativeTwist(otherBody, body), EPSILON);
            }
         }
      }
   }

   @Test
   public void testAgainstMovingReferenceFrameWithMatrixAccessor()
   {
      Random random = new Random(1776L);
      Twist expectedTwist = new Twist();

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, 10);
         RigidBodyBasics root = joints.get(0).getPredecessor();
         for (JointStateType stateType : JointStateType.values())
         {
            MultiBodySystemRandomTools.nextState(random, stateType, joints);
         }

         root.updateFramesRecursively();

         MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(root);
         DMatrixRMaj jointVelocities = new DMatrixRMaj(multiBodySystemInput.getNumberOfDoFs(), 1);
         MultiBodySystemTools.extractJointsState(multiBodySystemInput.getJointsToConsider(), JointStateType.VELOCITY, jointVelocities);
         RigidBodyTwistCalculator twistCalculator = new RigidBodyTwistCalculator(multiBodySystemInput);
         twistCalculator.useAllJointVelocityMatrix(jointVelocities);

         for (JointBasics joint : joints)
         {
            RigidBodyBasics body = joint.getSuccessor();
            MecanoTestTools.assertTwistEquals(body.getBodyFixedFrame().getTwistOfFrame(), twistCalculator.getTwistOfBody(body), EPSILON);

            for (JointBasics otherJoint : joints)
            {
               RigidBodyBasics otherBody = otherJoint.getSuccessor();
               body.getBodyFixedFrame().getTwistRelativeToOther(otherBody.getBodyFixedFrame(), expectedTwist);
               MecanoTestTools.assertTwistEquals(expectedTwist, twistCalculator.getRelativeTwist(otherBody, body), EPSILON);
            }
         }
      }
   }
}
