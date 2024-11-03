package us.ihmc.robotics.screwTheory;

import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.mecano.tools.MecanoRandomTools.nextWrench;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.stream.Collectors;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class GravityCoriolisExternalWrenchMatrixCalculatorTest
{
   private static final int ITERATIONS = 500;
   private static final double ONE_DOF_JOINT_EPSILON = 8.0e-12;
   private static final double FLOATING_JOINT_EPSILON = 2.0e-11;
   private static final double ALL_JOINT_EPSILON = 1.0e-4;

   @Test
   public void testPrismaticJointChain() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<PrismaticJoint> joints = MultiBodySystemRandomTools.nextPrismaticJointChain(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testPrismaticJointTree() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<PrismaticJoint> joints = MultiBodySystemRandomTools.nextPrismaticJointTree(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testRevoluteJointChain() throws Exception
   {
      Random random = new Random(2654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<RevoluteJoint> joints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testRevoluteJointTree() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<RevoluteJoint> joints = MultiBodySystemRandomTools.nextRevoluteJointTree(random, random.nextInt(50) + 1);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         joints = MultiBodySystemRandomTools.nextRevoluteJointTree(random, random.nextInt(40) + 1);
         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testOneDoFJointChain() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testOneDoFJointTree() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointTree(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testFloatingRevoluteJointChain() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(40) + 1;
         List<Joint> joints = new RandomFloatingRevoluteJointChain(random, numberOfJoints).getJoints();
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), FLOATING_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), FLOATING_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 2.0 * FLOATING_JOINT_EPSILON);
      }
   }

   @Test
   public void testJointChain() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(40) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ALL_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ALL_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ALL_JOINT_EPSILON);
      }
   }

   private void compareAgainstInverseDynamicsCalculator(Random random,
                                                        int iteration,
                                                        List<? extends JointBasics> joints,
                                                        Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                        List<? extends JointReadOnly> jointsToIgnore,
                                                        double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(multiBodySystemInput);
      inverseDynamicsCalculator.setConsiderCoriolisAndCentrifugalForces(true);
      inverseDynamicsCalculator.setConsiderJointAccelerations(false);
      inverseDynamicsCalculator.setGravitionalAcceleration(gravity);
      GravityCoriolisExternalWrenchMatrixCalculator gcewmCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(multiBodySystemInput);
      gcewmCalculator.setGravitionalAcceleration(gravity);

      int numberOfDoFs = joints.stream().mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

      externalWrenches.forEach(inverseDynamicsCalculator::setExternalWrench);
      inverseDynamicsCalculator.compute();
      DMatrixRMaj tau_expected = inverseDynamicsCalculator.getJointTauMatrix();

      externalWrenches.forEach(gcewmCalculator::setExternalWrench);
      gcewmCalculator.compute();
      DMatrixRMaj tau_actual = gcewmCalculator.getJointTauMatrix();

      boolean areEqual = MatrixFeatures_DDRM.isEquals(tau_expected, tau_actual, epsilon);
      if (!areEqual)
      {
         System.out.println("iteration: " + iteration);
         double maxError = 0.0;
         DMatrixRMaj output = new DMatrixRMaj(numberOfDoFs, 3);
         for (int row = 0; row < numberOfDoFs; row++)
         {
            output.set(row, 0, tau_expected.get(row, 0));
            output.set(row, 1, tau_actual.get(row, 0));
            double error = tau_expected.get(row, 0) - tau_actual.get(row, 0);
            output.set(row, 2, error);
            maxError = Math.max(maxError, Math.abs(error));
         }
         output.print(EuclidCoreIOTools.getStringFormat(9, 6));
         System.out.println("Max error: " + maxError);
      }
      assertTrue(areEqual);

      List<? extends RigidBodyBasics> allRigidBodies = rootBody.subtreeList();

      for (RigidBodyReadOnly rigidBody : allRigidBodies)
      {
         try
         {
            SpatialAccelerationReadOnly expectedAccelerationOfBody = inverseDynamicsCalculator.getAccelerationProvider().getAccelerationOfBody(rigidBody);

            if (expectedAccelerationOfBody == null)
            {
               assertNull(gcewmCalculator.getAccelerationProvider().getAccelerationOfBody(rigidBody));
            }
            else
            {
               SpatialAcceleration actualAccelerationOfBody = new SpatialAcceleration(gcewmCalculator.getAccelerationProvider()
                                                                                                     .getAccelerationOfBody(rigidBody));
               actualAccelerationOfBody.changeFrame(expectedAccelerationOfBody.getReferenceFrame());
               MecanoTestTools.assertSpatialAccelerationEquals(expectedAccelerationOfBody, actualAccelerationOfBody, epsilon);

               for (int i = 0; i < 5; i++)
               {
                  RigidBodyBasics otherRigidBody = allRigidBodies.get(random.nextInt(allRigidBodies.size()));
                  SpatialAccelerationReadOnly expectedRelativeAcceleration = inverseDynamicsCalculator.getAccelerationProvider()
                                                                                                      .getRelativeAcceleration(rigidBody, otherRigidBody);
                  if (expectedAccelerationOfBody == null)
                  {
                     assertNull(gcewmCalculator.getAccelerationProvider().getRelativeAcceleration(otherRigidBody, rigidBody));
                  }
                  else
                  {
                     SpatialAccelerationReadOnly actualRelativeAcceleration = gcewmCalculator.getAccelerationProvider()
                                                                                             .getRelativeAcceleration(rigidBody, otherRigidBody);
                     MecanoTestTools.assertSpatialAccelerationEquals(expectedRelativeAcceleration, actualRelativeAcceleration, epsilon);
                  }
               }
            }
         }
         catch (NullPointerException e)
         {
            // TODO Remove this try-catch block when switching to Mecano 0.0.22 or later.
         }
      }
   }

   public static Map<RigidBodyReadOnly, WrenchReadOnly> nextExternalWrenches(Random random, List<? extends JointReadOnly> joints)
   {
      return joints.stream()
                   .filter(j -> random.nextBoolean())
                   .map(j -> j.getSuccessor())
                   .collect(Collectors.toMap(b -> b, b -> nextWrench(random, b.getBodyFixedFrame(), b.getBodyFixedFrame())));
   }
}
