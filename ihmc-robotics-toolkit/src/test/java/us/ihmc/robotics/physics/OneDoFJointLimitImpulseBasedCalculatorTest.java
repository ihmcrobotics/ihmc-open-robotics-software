package us.ihmc.robotics.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class OneDoFJointLimitImpulseBasedCalculatorTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testPrismaticChain()
   {
      Random random = new Random(45436);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = random.nextDouble();
         int numberOfJoints = 20;
         List<PrismaticJoint> joints = MultiBodySystemRandomTools.nextPrismaticJointChain(random, numberOfJoints);
         for (JointStateType state : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, state, joints);

         int jointIndex = random.nextInt(numberOfJoints);
         PrismaticJoint joint = joints.get(jointIndex);
         RigidBodyReadOnly rootBody = MultiBodySystemTools.getRootBody(joint.getPredecessor());
         ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(rootBody);
         forwardDynamicsCalculator.compute();

         double jointLimitLower, jointLimitUpper;

         double q = joint.getQ();
         double qd = joint.getQd();
         double qdd = forwardDynamicsCalculator.getComputedJointAcceleration(joint).get(0);

         boolean isJointApproachingLimit;

         if (random.nextBoolean())
         {
            jointLimitLower = q + 1.0e-12;
            jointLimitUpper = q + EuclidCoreRandomTools.nextDouble(random, 0.1, 1.0);
            isJointApproachingLimit = qd + dt * qdd <= 0.0;
         }
         else
         {
            jointLimitLower = q - EuclidCoreRandomTools.nextDouble(random, 0.1, 1.0);
            jointLimitUpper = q - 1.0e-12;
            isJointApproachingLimit = qd + dt * qdd >= 0.0;
         }

         joint.setJointLimits(jointLimitLower, jointLimitUpper);


         OneDoFJointLimitImpulseBasedCalculator calculator = new OneDoFJointLimitImpulseBasedCalculator(dt, joint, forwardDynamicsCalculator);
         calculator.setSpringConstant(0.0);
         calculator.computeImpulse();

         String message = "Iteration " + i;
         assertEquals(isJointApproachingLimit, calculator.isConstraintActive(), message);

         if (calculator.isConstraintActive())
         {
            DenseMatrix64F jointVelocities = new DenseMatrix64F(joints.stream().mapToInt(JointReadOnly::getDegreesOfFreedom).sum(), 1);
            MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocities);
            CommonOps.addEquals(jointVelocities, dt, forwardDynamicsCalculator.getJointAccelerationMatrix());
            CommonOps.addEquals(jointVelocities, calculator.getJointVelocityChange(0));
            MultiBodySystemTools.insertJointsState(joints, JointStateType.VELOCITY, jointVelocities);
            assertEquals(0.0, joint.getQd(), 1.0e-12, message);
         }
      }
   }
}
