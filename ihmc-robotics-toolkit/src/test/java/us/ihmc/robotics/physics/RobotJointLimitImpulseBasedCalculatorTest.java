package us.ihmc.robotics.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.physics.RobotJointLimitImpulseBasedCalculator.ActiveLimit;

public class RobotJointLimitImpulseBasedCalculatorTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testRevoluteChainSingleLimit()
   {
      Random random = new Random(45436);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = random.nextDouble();
         int numberOfJoints = 20;
         List<RevoluteJoint> joints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
         for (JointStateType state : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, state, joints);

         int jointIndex = random.nextInt(numberOfJoints);
         RevoluteJoint joint = joints.get(jointIndex);
         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joint.getPredecessor());
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
            jointLimitUpper = q + EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
            isJointApproachingLimit = q + dt * qd + 0.5 * dt * dt * qdd <= jointLimitLower && qd + dt * qdd <= 0.0;
         }
         else
         {
            jointLimitLower = q - EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
            jointLimitUpper = q - 1.0e-12;
            isJointApproachingLimit = q + dt * qd + 0.5 * dt * dt * qdd >= jointLimitUpper && qd + dt * qdd >= 0.0;
         }

         joint.setJointLimits(jointLimitLower, jointLimitUpper);

         RobotJointLimitImpulseBasedCalculator calculator = new RobotJointLimitImpulseBasedCalculator(rootBody, forwardDynamicsCalculator);
         calculator.setConstraintParameters(new ConstraintParameters(0.0, 0.0, 0.0));
         calculator.initialize(dt);
         calculator.updateInertia(null, null);
         calculator.computeImpulse(dt);

         String message = "Iteration " + i;
         assertEquals(isJointApproachingLimit, calculator.isConstraintActive(), message);

         if (calculator.isConstraintActive())
         {
            DMatrixRMaj jointVelocities = new DMatrixRMaj(joints.stream().mapToInt(JointReadOnly::getDegreesOfFreedom).sum(), 1);
            MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocities);
            CommonOps_DDRM.addEquals(jointVelocities, dt, forwardDynamicsCalculator.getJointAccelerationMatrix());
            CommonOps_DDRM.addEquals(jointVelocities, calculator.getJointVelocityChange(0));
            MultiBodySystemTools.insertJointsState(joints, JointStateType.VELOCITY, jointVelocities);
            assertEquals(0.0, joint.getQd(), 1.0e-12, message);
         }
      }
   }

   @Test
   public void testRevoluteChainMultipleLimit()
   {
      Random random = new Random(45436);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = random.nextDouble();
         int numberOfJoints = 20;
         List<RevoluteJoint> joints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
         for (JointStateType state : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, state, joints);

         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
         ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(rootBody);
         forwardDynamicsCalculator.compute();

         int numberOfJointsToPutAtLimit = random.nextInt(numberOfJoints);

         List<RevoluteJoint> jointPool = new ArrayList<>(joints);

         for (int j = 0; j < numberOfJointsToPutAtLimit; j++)
         {
            int jointIndex = random.nextInt(jointPool.size());
            RevoluteJoint joint = jointPool.remove(jointIndex);
            double jointLimitLower, jointLimitUpper;

            double q = joint.getQ();

            if (random.nextBoolean())
            {
               jointLimitLower = q + 1.0e-12;
               jointLimitUpper = q + EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
            }
            else
            {
               jointLimitLower = q - EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
               jointLimitUpper = q - 1.0e-12;
            }

            joint.setJointLimits(jointLimitLower, jointLimitUpper);
         }

         RobotJointLimitImpulseBasedCalculator calculator = new RobotJointLimitImpulseBasedCalculator(rootBody, forwardDynamicsCalculator);
         calculator.setConstraintParameters(new ConstraintParameters(0.0, 0.0, 0.0));
         calculator.initialize(dt);
         calculator.updateInertia(null, null);
         calculator.computeImpulse(dt);

         String message = "Iteration " + i;

         if (calculator.isConstraintActive())
         {
            DMatrixRMaj jointVelocities = new DMatrixRMaj(joints.stream().mapToInt(JointReadOnly::getDegreesOfFreedom).sum(), 1);
            MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocities);
            CommonOps_DDRM.addEquals(jointVelocities, dt, forwardDynamicsCalculator.getJointAccelerationMatrix());
            CommonOps_DDRM.addEquals(jointVelocities, calculator.getJointVelocityChange(0));
            MultiBodySystemTools.insertJointsState(joints, JointStateType.VELOCITY, jointVelocities);

            List<ActiveLimit> activeLimits = calculator.getActiveLimits();
            List<OneDoFJointBasics> jointsAtLimit = calculator.getJointTargets();

            for (int j = 0; j < jointsAtLimit.size(); j++)
            {
               OneDoFJointBasics joint = jointsAtLimit.get(j);

               double jointImpulse = calculator.getImpulse().get(j);
               assertEquals(0.0, joint.getQd() * jointImpulse, 1.0e-12);

               if (activeLimits.get(j) == ActiveLimit.LOWER)
               {
                  assertTrue(jointImpulse >= -1.0e-12, message);
                  assertTrue(joint.getQd() >= -1.0e-3, message);
               }
               else
               {
                  assertTrue(jointImpulse <= 1.0e-12, message);
                  assertTrue(joint.getQd() <= 1.0e-3, message);
               }
            }
         }
      }
   }
}
