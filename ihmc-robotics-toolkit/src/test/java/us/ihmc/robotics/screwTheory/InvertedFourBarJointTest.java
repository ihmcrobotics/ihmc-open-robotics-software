package us.ihmc.robotics.screwTheory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.function.Function;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.Test;
import org.opentest4j.AssertionFailedError;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.algorithms.SpatialAccelerationCalculator;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoIOTools;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotics.geometry.GeometryTools;

public class InvertedFourBarJointTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 1000;
   private static final double SMALL_EPSILON = 1.0e-10;
   private static final double MID_EPSILON = 1.0e-7;
   private static final double LARGE_EPSILON = 1.0e-5;

   @Test
   public void testMotionSubspaceDot()
   {
      Random random = new Random(4577);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         double dt = 0.5e-6;
         DMatrixRMaj Sprev = new DMatrixRMaj(6, 1);
         DMatrixRMaj Scurr = new DMatrixRMaj(6, 1);
         DMatrixRMaj actualSPrime = new DMatrixRMaj(6, 1);
         DMatrixRMaj expectedSPrime = new DMatrixRMaj(6, 1);
         DMatrixRMaj errorSPrime = new DMatrixRMaj(6, 1);

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMin = fourBarJoint.getJointLimitLower();
            double qMax = fourBarJoint.getJointLimitUpper();
            double qRange = qMax - qMin;
            qMin += 0.05 * qRange;
            qMax -= 0.05 * qRange;
            double q = EuclidCoreRandomTools.nextDouble(random, qMin, qMax);
            double qd = EuclidCoreRandomTools.nextDouble(random, 10.0);

            fourBarJoint.setQ(q);
            fourBarJoint.setQd(qd);
            fourBarJoint.updateFramesRecursively();

            fourBarJoint.getMotionSubspace(Sprev);
            fourBarJoint.getMotionSubspaceDot(actualSPrime);

            q += qd * dt;
            fourBarJoint.setQ(q);
            fourBarJoint.updateFramesRecursively();
            fourBarJoint.getMotionSubspace(Scurr);

            MatrixTools.numericallyDifferentiate(expectedSPrime, Sprev, Scurr, dt);

            CommonOps_DDRM.subtract(expectedSPrime, actualSPrime, errorSPrime);
            assertTrue(MatrixFeatures_DDRM.isEquals(expectedSPrime, actualSPrime, 1.0e-4),
                       String.format("Iteration: %d\nExpected:\n%s\nwas:\n%s\nDifference:\n%s",
                                     i,
                                     expectedSPrime.toString(),
                                     actualSPrime.toString(),
                                     errorSPrime.toString()));
         }
      }
   }

   @Test
   public void testSetQ()
   {
      Random random = new Random(348975);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         FourBarKinematicLoopFunction function = generator.apply("copy").getFourBarFunction();

         for (int i = 0; i < ITERATIONS; i++)
         { // Test configuration
            double alpha = i / (ITERATIONS - 1.0);
            double expected_qMaster = EuclidCoreTools.interpolate(function.getMasterJoint().getJointLimitLower(),
                                                                  function.getMasterJoint().getJointLimitUpper(),
                                                                  alpha);
            function.getMasterJoint().setQ(expected_qMaster);
            function.updateState(false, false);
            double expected_qA = function.getJointA().getQ();
            double expected_qB = function.getJointB().getQ();
            double expected_qC = function.getJointC().getQ();
            double expected_qD = function.getJointD().getQ();

            // Because the FourBarKinematicLoopFunction re-order the joints to be predictable, theta = qA + qD = qB + qC
            double theta = expected_qA + expected_qD;
            assertEquals(theta, expected_qB + expected_qC, SMALL_EPSILON);

            // randomize the configuration first
            MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, function.getLoopJoints());

            fourBarJoint.setQ(theta);
            fourBarJoint.updateFramesRecursively();

            double actual_qA = fourBarJoint.getJointA().getQ();
            double actual_qB = fourBarJoint.getJointB().getQ();
            double actual_qC = fourBarJoint.getJointC().getQ();
            double actual_qD = fourBarJoint.getJointD().getQ();

            assertEquals(expected_qA, actual_qA, SMALL_EPSILON);
            assertEquals(expected_qB, actual_qB, SMALL_EPSILON);
            assertEquals(expected_qC, actual_qC, SMALL_EPSILON);
            assertEquals(expected_qD, actual_qD, SMALL_EPSILON);
            assertEquals(theta, fourBarJoint.getQ(), SMALL_EPSILON);
         }

         { // Assert that q is clamped to the four bar limits
            fourBarJoint.setQ(fourBarJoint.getJointLimitLower() - 0.1);
            fourBarJoint.updateFramesRecursively();
            assertEquals(function.getJointA().getJointLimitLower(), fourBarJoint.getJointA().getQ());
            assertEquals(function.getJointB().getJointLimitLower(), fourBarJoint.getJointB().getQ());
            assertEquals(function.getJointC().getJointLimitLower(), fourBarJoint.getJointC().getQ());
            assertEquals(function.getJointD().getJointLimitLower(), fourBarJoint.getJointD().getQ());
            assertEquals(fourBarJoint.getJointLimitLower(), fourBarJoint.getQ(), SMALL_EPSILON);

            fourBarJoint.setQ(fourBarJoint.getJointLimitUpper() + 0.1);
            fourBarJoint.updateFramesRecursively();
            assertEquals(function.getJointA().getJointLimitUpper(), fourBarJoint.getJointA().getQ());
            assertEquals(function.getJointB().getJointLimitUpper(), fourBarJoint.getJointB().getQ());
            assertEquals(function.getJointC().getJointLimitUpper(), fourBarJoint.getJointC().getQ());
            assertEquals(function.getJointD().getJointLimitUpper(), fourBarJoint.getJointD().getQ());
            assertEquals(fourBarJoint.getJointLimitUpper(), fourBarJoint.getQ(), SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testSetJointOrientation()
   {
      Random random = new Random(7523);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");

         for (int i = 0; i < ITERATIONS; i++)
         {
            double theta = EuclidCoreRandomTools.nextDouble(random, fourBarJoint.getJointLimitLower(), fourBarJoint.getJointLimitUpper());
            Vector3D jointAxis = new Vector3D(fourBarJoint.getJointAxis());
            Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, jointAxis, true);
            Vector3D rotationVector = new Vector3D();
            rotationVector.setAndScale(theta, jointAxis);
            rotationVector.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.5), orthogonalToAxis, rotationVector);
            fourBarJoint.setJointOrientation(new Quaternion(rotationVector));
            fourBarJoint.updateFramesRecursively();
            assertEquals(theta, fourBarJoint.getQ(), SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testSetJointPosition()
   {
      Random random = new Random(7523);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");

         for (int i = 0; i < ITERATIONS; i++)
         { // Assert that the method does nothing
            MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, fourBarJoint);
            fourBarJoint.updateFramesRecursively();
            double expected_q = fourBarJoint.getQ();
            fourBarJoint.setJointPosition(EuclidCoreRandomTools.nextPoint3D(random, 1.0));
            fourBarJoint.updateFramesRecursively();
            double actual_q = fourBarJoint.getQ();

            assertEquals(expected_q, actual_q, SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testGetJointConfiguration()
   {
      Random random = new Random(348975);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         FourBarKinematicLoopFunction function = generator.apply("copy").getFourBarFunction();

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMaster = EuclidCoreRandomTools.nextDouble(random,
                                                              function.getMasterJoint().getJointLimitLower(),
                                                              function.getMasterJoint().getJointLimitUpper());
            fourBarJoint.getMasterJoint().setQ(qMaster);
            function.getMasterJoint().setQ(qMaster);
            fourBarJoint.updateFramesRecursively();
            function.updateState(false, false);
            function.getJointA().getPredecessor().updateFramesRecursively();

            assertEquals(function.getJointA().getQ(), fourBarJoint.getJointA().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointB().getQ(), fourBarJoint.getJointB().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointC().getQ(), fourBarJoint.getJointC().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointD().getQ(), fourBarJoint.getJointD().getQ(), SMALL_EPSILON);

            RigidBodyTransform actualConfiguration = new RigidBodyTransform();
            fourBarJoint.getJointConfiguration(actualConfiguration);

            if (fourBarJoint.getFrameBeforeJoint() == fourBarJoint.getJointA().getFrameBeforeJoint())
            {
               RigidBodyTransform expectedConfiguration = function.getJointD().getFrameAfterJoint()
                                                                  .getTransformToDesiredFrame(function.getJointA().getFrameBeforeJoint());
               EuclidCoreTestTools.assertRigidBodyTransformEquals("Iteration: " + i, expectedConfiguration, actualConfiguration, SMALL_EPSILON);
            }
            else
            {
               RigidBodyTransform expectedConfiguration = function.getJointC().getFrameAfterJoint()
                                                                  .getTransformToDesiredFrame(function.getJointB().getFrameBeforeJoint());
               EuclidCoreTestTools.assertRigidBodyTransformEquals("Iteration: " + i, expectedConfiguration, actualConfiguration, SMALL_EPSILON);
            }
         }
      }
   }

   @Test
   public void testSetQd()
   {
      Random random = new Random(8623);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         FourBarKinematicLoopFunction function = generator.apply("copy").getFourBarFunction();

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMaster = EuclidCoreRandomTools.nextDouble(random,
                                                              function.getMasterJoint().getJointLimitLower(),
                                                              function.getMasterJoint().getJointLimitUpper());
            double expected_qdMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            function.getMasterJoint().setQ(qMaster);
            function.getMasterJoint().setQd(expected_qdMaster);
            function.updateState(true, false);

            double expected_qdA = function.getJointA().getQd();
            double expected_qdB = function.getJointB().getQd();
            double expected_qdC = function.getJointC().getQd();
            double expected_qdD = function.getJointD().getQd();
            double theta = function.getJointA().getQ() + function.getJointD().getQ();
            double thetaDot = expected_qdA + expected_qdD;
            assertEquals(thetaDot, expected_qdB + expected_qdC, SMALL_EPSILON);

            fourBarJoint.setQ(theta);
            fourBarJoint.setQd(thetaDot);
            fourBarJoint.updateFramesRecursively();

            double actual_qdA = fourBarJoint.getJointA().getQd();
            double actual_qdB = fourBarJoint.getJointB().getQd();
            double actual_qdC = fourBarJoint.getJointC().getQd();
            double actual_qdD = fourBarJoint.getJointD().getQd();

            assertEquals(theta, fourBarJoint.getQ(), SMALL_EPSILON);
            assertEquals(expected_qdA, actual_qdA, MID_EPSILON);
            assertEquals(expected_qdB, actual_qdB, MID_EPSILON);
            assertEquals(expected_qdC, actual_qdC, MID_EPSILON);
            assertEquals(expected_qdD, actual_qdD, MID_EPSILON);
            assertEquals(thetaDot, fourBarJoint.getQd(), SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testSetJointAngularVelocity()
   {
      Random random = new Random(67567);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");

         for (int i = 0; i < ITERATIONS; i++)
         {
            double theta = EuclidCoreRandomTools.nextDouble(random, fourBarJoint.getJointLimitLower(), fourBarJoint.getJointLimitUpper());
            double thetaDot = EuclidCoreRandomTools.nextDouble(random, 10.0);

            Vector3D angularVelocity = new Vector3D();
            angularVelocity.setAndScale(thetaDot, fourBarJoint.getJointAxis());
            Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, fourBarJoint.getJointAxis(), true);
            angularVelocity.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonalToAxis, angularVelocity);

            fourBarJoint.setQ(theta);
            fourBarJoint.setJointAngularVelocity(angularVelocity);
            fourBarJoint.updateFramesRecursively();

            assertEquals(theta, fourBarJoint.getQ(), SMALL_EPSILON);
            assertEquals(thetaDot, fourBarJoint.getQd(), MID_EPSILON);
         }
      }
   }

   @Test
   public void testSetJointLinearVelocity()
   {
      Random random = new Random(7523);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");

         for (int i = 0; i < ITERATIONS; i++)
         { // Assert that the method does nothing
            MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, fourBarJoint);
            MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, fourBarJoint);
            fourBarJoint.updateFramesRecursively();
            double expected_qd = fourBarJoint.getQd();
            fourBarJoint.setJointLinearVelocity(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
            fourBarJoint.updateFramesRecursively();
            double actual_qd = fourBarJoint.getQd();

            assertEquals(expected_qd, actual_qd, SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testGetJointTwist()
   {
      Random random = new Random(4754756);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         FourBarKinematicLoopFunction function = generator.apply("copy").getFourBarFunction();

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMaster = EuclidCoreRandomTools.nextDouble(random,
                                                              fourBarJoint.getMasterJoint().getJointLimitLower(),
                                                              fourBarJoint.getMasterJoint().getJointLimitUpper());
            double qdMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            fourBarJoint.getMasterJoint().setQ(qMaster);
            fourBarJoint.getMasterJoint().setQd(qdMaster);
            function.getMasterJoint().setQ(qMaster);
            function.getMasterJoint().setQd(qdMaster);

            fourBarJoint.updateFramesRecursively();
            function.updateState(true, false);
            function.getJointA().getPredecessor().updateFramesRecursively();

            RevoluteJointBasics topJoint, bottomJoint;
            if (fourBarJoint.getFrameBeforeJoint() == fourBarJoint.getJointA().getFrameBeforeJoint())
            {
               topJoint = fourBarJoint.getJointA();
               bottomJoint = fourBarJoint.getJointD();
            }
            else
            {
               topJoint = fourBarJoint.getJointB();
               bottomJoint = fourBarJoint.getJointC();
            }

            Twist expectedTwist = new Twist();
            bottomJoint.getFrameAfterJoint().getTwistRelativeToOther(topJoint.getFrameBeforeJoint(), expectedTwist);
            TwistReadOnly actualTwist = fourBarJoint.getJointTwist();

            EuclidCoreTestTools.assertRigidBodyTransformEquals(bottomJoint.getFrameAfterJoint().getTransformToDesiredFrame(topJoint.getFrameBeforeJoint()),
                                                               bottomJoint.getFrameAfterJoint().getTransformToDesiredFrame(topJoint.getFrameBeforeJoint()),
                                                               SMALL_EPSILON);

            assertEquals(function.getJointA().getQ(), fourBarJoint.getJointA().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointB().getQ(), fourBarJoint.getJointB().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointC().getQ(), fourBarJoint.getJointC().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointD().getQ(), fourBarJoint.getJointD().getQ(), SMALL_EPSILON);

            assertEquals(function.getJointA().getQd(), fourBarJoint.getJointA().getQd(), SMALL_EPSILON);
            assertEquals(function.getJointB().getQd(), fourBarJoint.getJointB().getQd(), SMALL_EPSILON);
            assertEquals(function.getJointC().getQd(), fourBarJoint.getJointC().getQd(), SMALL_EPSILON);
            assertEquals(function.getJointD().getQd(), fourBarJoint.getJointD().getQd(), SMALL_EPSILON);

            assertTwistEquals(null, function.getJointA().getJointTwist(), fourBarJoint.getJointA().getJointTwist(), SMALL_EPSILON);
            assertTwistEquals(null, function.getJointB().getJointTwist(), fourBarJoint.getJointB().getJointTwist(), SMALL_EPSILON);
            assertTwistEquals(null, function.getJointC().getJointTwist(), fourBarJoint.getJointC().getJointTwist(), SMALL_EPSILON);
            assertTwistEquals(null, function.getJointD().getJointTwist(), fourBarJoint.getJointD().getJointTwist(), SMALL_EPSILON);

            // Only checking the frames by name
            assertTwistEquals("Iteration " + i, expectedTwist, actualTwist, SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testGetSuccessorTwist()
   {
      Random random = new Random(4754756);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         FourBarKinematicLoopFunction function = generator.apply("copy").getFourBarFunction();

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMaster = EuclidCoreRandomTools.nextDouble(random,
                                                              fourBarJoint.getMasterJoint().getJointLimitLower(),
                                                              fourBarJoint.getMasterJoint().getJointLimitUpper());
            double qdMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            fourBarJoint.getMasterJoint().setQ(qMaster);
            fourBarJoint.getMasterJoint().setQd(qdMaster);
            function.getMasterJoint().setQ(qMaster);
            function.getMasterJoint().setQd(qdMaster);

            fourBarJoint.updateFramesRecursively();
            function.updateState(true, false);
            function.getJointA().getPredecessor().updateFramesRecursively();

            Twist expectedTwist = new Twist();
            function.getJointD().getSuccessor().getBodyFixedFrame().getTwistRelativeToOther(function.getJointA().getPredecessor().getBodyFixedFrame(),
                                                                                            expectedTwist);

            Twist actualTwist = new Twist();
            fourBarJoint.getSuccessorTwist(actualTwist);

            // Only checking the frames by name
            assertTwistEquals("Iteration " + i, expectedTwist, actualTwist, MID_EPSILON);
         }
      }
   }

   @Test
   public void testGetPredecessorTwist()
   {
      Random random = new Random(4754756);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         FourBarKinematicLoopFunction function = generator.apply("copy").getFourBarFunction();

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMaster = EuclidCoreRandomTools.nextDouble(random,
                                                              fourBarJoint.getMasterJoint().getJointLimitLower(),
                                                              fourBarJoint.getMasterJoint().getJointLimitUpper());
            double qdMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            fourBarJoint.getMasterJoint().setQ(qMaster);
            fourBarJoint.getMasterJoint().setQd(qdMaster);
            function.getMasterJoint().setQ(qMaster);
            function.getMasterJoint().setQd(qdMaster);

            fourBarJoint.updateFramesRecursively();
            function.updateState(true, false);
            function.getJointA().getPredecessor().updateFramesRecursively();

            Twist expectedTwist = new Twist();
            function.getJointA().getPredecessor().getBodyFixedFrame().getTwistRelativeToOther(function.getJointD().getSuccessor().getBodyFixedFrame(),
                                                                                              expectedTwist);

            Twist actualTwist = new Twist();
            fourBarJoint.getPredecessorTwist(actualTwist);

            // Only checking the frames by name
            assertTwistEquals("Iteration " + i, expectedTwist, actualTwist, MID_EPSILON);
         }
      }
   }

   @Test
   public void testGetUnitJointTwist()
   {
      Random random = new Random(8623435);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         FourBarKinematicLoopFunction function = generator.apply("copy").getFourBarFunction();

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMaster = EuclidCoreRandomTools.nextDouble(random,
                                                              fourBarJoint.getMasterJoint().getJointLimitLower(),
                                                              fourBarJoint.getMasterJoint().getJointLimitUpper());
            double qdMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            fourBarJoint.getMasterJoint().setQ(qMaster);
            fourBarJoint.getMasterJoint().setQd(qdMaster);
            function.getMasterJoint().setQ(qMaster);
            function.getMasterJoint().setQd(qdMaster);

            fourBarJoint.updateFramesRecursively();
            function.updateState(true, false);
            function.getJointA().getPredecessor().updateFramesRecursively();

            RevoluteJointBasics topJoint, bottomJoint;
            if (fourBarJoint.getFrameBeforeJoint() == fourBarJoint.getJointA().getFrameBeforeJoint())
            {
               topJoint = fourBarJoint.getJointA();
               bottomJoint = fourBarJoint.getJointD();
            }
            else
            {
               topJoint = fourBarJoint.getJointB();
               bottomJoint = fourBarJoint.getJointC();
            }

            Twist expectedTwist = new Twist();
            bottomJoint.getFrameAfterJoint().getTwistRelativeToOther(topJoint.getFrameBeforeJoint(), expectedTwist);
            expectedTwist.scale(1.0 / (topJoint.getQd() + bottomJoint.getQd()));

            TwistReadOnly actualTwist = fourBarJoint.getUnitJointTwist();

            assertTwistEquals("Iteration " + i, expectedTwist, actualTwist, SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testGetUnitSuccessorTwist()
   {
      Random random = new Random(8623435);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         FourBarKinematicLoopFunction function = generator.apply("copy").getFourBarFunction();

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMaster = EuclidCoreRandomTools.nextDouble(random,
                                                              fourBarJoint.getMasterJoint().getJointLimitLower(),
                                                              fourBarJoint.getMasterJoint().getJointLimitUpper());
            double qdMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            fourBarJoint.getMasterJoint().setQ(qMaster);
            fourBarJoint.getMasterJoint().setQd(qdMaster);
            function.getMasterJoint().setQ(qMaster);
            function.getMasterJoint().setQd(qdMaster);

            fourBarJoint.updateFramesRecursively();
            function.updateState(true, false);
            function.getJointA().getPredecessor().updateFramesRecursively();

            Twist expectedTwist = new Twist();
            function.getJointD().getSuccessor().getBodyFixedFrame().getTwistRelativeToOther(function.getJointA().getPredecessor().getBodyFixedFrame(),
                                                                                            expectedTwist);
            expectedTwist.scale(1.0 / (function.getJointA().getQd() + function.getJointD().getQd()));

            TwistReadOnly actualTwist = fourBarJoint.getUnitSuccessorTwist();

            assertTwistEquals("Iteration " + i, expectedTwist, actualTwist, SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testGetUnitPredecessorTwist()
   {
      Random random = new Random(8623435);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         FourBarKinematicLoopFunction function = generator.apply("copy").getFourBarFunction();

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMaster = EuclidCoreRandomTools.nextDouble(random,
                                                              fourBarJoint.getMasterJoint().getJointLimitLower(),
                                                              fourBarJoint.getMasterJoint().getJointLimitUpper());
            double qdMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            fourBarJoint.getMasterJoint().setQ(qMaster);
            fourBarJoint.getMasterJoint().setQd(qdMaster);
            function.getMasterJoint().setQ(qMaster);
            function.getMasterJoint().setQd(qdMaster);

            fourBarJoint.updateFramesRecursively();
            function.updateState(true, false);
            function.getJointA().getPredecessor().updateFramesRecursively();

            Twist expectedTwist = new Twist();
            function.getJointA().getPredecessor().getBodyFixedFrame().getTwistRelativeToOther(function.getJointD().getSuccessor().getBodyFixedFrame(),
                                                                                              expectedTwist);
            expectedTwist.scale(1.0 / (function.getJointA().getQd() + function.getJointD().getQd()));

            TwistReadOnly actualTwist = fourBarJoint.getUnitPredecessorTwist();

            assertTwistEquals("Iteration " + i, expectedTwist, actualTwist, SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testSetQdd()
   {
      Random random = new Random(8623);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         FourBarKinematicLoopFunction function = generator.apply("copy").getFourBarFunction();

         long totalTime = 0;

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMasterMin = function.getMasterJoint().getJointLimitLower();
            double qMasterMax = function.getMasterJoint().getJointLimitUpper();
            double qMasterMid = 0.5 * (qMasterMin + qMasterMax);
            double qMasterRange = 0.75 * (qMasterMax - qMasterMin); // Reduce the range little to avoid singularities causing large acceleration values.

            double qMaster = qMasterMid + 0.5 * EuclidCoreRandomTools.nextDouble(random, qMasterRange);
            double qdMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double expected_qddMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            function.getMasterJoint().setQ(qMaster);
            function.getMasterJoint().setQd(qdMaster);
            function.getMasterJoint().setQdd(expected_qddMaster);
            function.updateState(true, true);

            double expected_qddA = function.getJointA().getQdd();
            double expected_qddB = function.getJointB().getQdd();
            double expected_qddC = function.getJointC().getQdd();
            double expected_qddD = function.getJointD().getQdd();
            double theta = function.getJointA().getQ() + function.getJointD().getQ();
            double thetaDot = function.getJointA().getQd() + function.getJointD().getQd();
            double thetaDDot = expected_qddA + expected_qddD;
            assertEquals(thetaDDot, expected_qddB + expected_qddC, MID_EPSILON);

            long start = System.nanoTime();
            fourBarJoint.setQ(theta);
            fourBarJoint.setQd(thetaDot);
            fourBarJoint.setQdd(thetaDDot);
            fourBarJoint.updateFramesRecursively();
            long end = System.nanoTime();
            totalTime += end - start;

            double actual_qddA = fourBarJoint.getJointA().getQdd();
            double actual_qddB = fourBarJoint.getJointB().getQdd();
            double actual_qddC = fourBarJoint.getJointC().getQdd();
            double actual_qddD = fourBarJoint.getJointD().getQdd();

            assertEquals(theta, fourBarJoint.getQ(), SMALL_EPSILON);
            assertEquals(thetaDot, fourBarJoint.getQd(), SMALL_EPSILON);
            assertEqualsVarEpsilon(thetaDDot, fourBarJoint.getQdd(), MID_EPSILON);
            assertEqualsVarEpsilon(expected_qddA, actual_qddA, LARGE_EPSILON);
            assertEqualsVarEpsilon(expected_qddB, actual_qddB, LARGE_EPSILON);
            assertEqualsVarEpsilon(expected_qddC, actual_qddC, LARGE_EPSILON);
            assertEqualsVarEpsilon(expected_qddD, actual_qddD, LARGE_EPSILON);
         }

         System.out.println("Average time: " + (totalTime / 1.0e6 / ITERATIONS) + "millisec");
      }
   }

   @Test
   public void testSetJointAngularAcceleration()
   {
      Random random = new Random(67567);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");

         for (int i = 0; i < ITERATIONS; i++)
         {
            double theta = EuclidCoreRandomTools.nextDouble(random, fourBarJoint.getJointLimitLower(), fourBarJoint.getJointLimitUpper());
            double thetaDot = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double thetaDDot = EuclidCoreRandomTools.nextDouble(random, 10.0);

            Vector3D angularAcceleration = new Vector3D();
            angularAcceleration.setAndScale(thetaDDot, fourBarJoint.getJointAxis());
            Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, fourBarJoint.getJointAxis(), true);
            angularAcceleration.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonalToAxis, angularAcceleration);

            fourBarJoint.setQ(theta);
            fourBarJoint.setQd(thetaDot);
            fourBarJoint.setJointAngularAcceleration(angularAcceleration);
            fourBarJoint.updateFramesRecursively();

            assertEquals(theta, fourBarJoint.getQ(), SMALL_EPSILON);
            assertEquals(thetaDot, fourBarJoint.getQd(), MID_EPSILON);
            assertEquals(thetaDDot, fourBarJoint.getQdd(), MID_EPSILON);
         }
      }
   }

   @Test
   public void testSetJointLinearAcceleration()
   {
      Random random = new Random(7523);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");

         for (int i = 0; i < ITERATIONS; i++)
         { // Assert that the method does nothing
            MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, fourBarJoint);
            MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, fourBarJoint);
            fourBarJoint.updateFramesRecursively();
            double expected_qdd = fourBarJoint.getQdd();
            fourBarJoint.setJointLinearAcceleration(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
            fourBarJoint.updateFramesRecursively();
            double actual_qdd = fourBarJoint.getQdd();

            assertEquals(expected_qdd, actual_qdd, SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testGetJointAcceleration()
   {
      Random random = new Random(4754756);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         FourBarKinematicLoopFunction function = generator.apply("copy").getFourBarFunction();

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMaster = EuclidCoreRandomTools.nextDouble(random,
                                                              fourBarJoint.getMasterJoint().getJointLimitLower(),
                                                              fourBarJoint.getMasterJoint().getJointLimitUpper());
            double qdMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double qddMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            fourBarJoint.getMasterJoint().setQ(qMaster);
            fourBarJoint.getMasterJoint().setQd(qdMaster);
            fourBarJoint.getMasterJoint().setQdd(qddMaster);
            function.getMasterJoint().setQ(qMaster);
            function.getMasterJoint().setQd(qdMaster);
            function.getMasterJoint().setQdd(qddMaster);

            fourBarJoint.updateFramesRecursively();
            function.updateState(true, true);
            function.getJointA().getPredecessor().updateFramesRecursively();

            assertEquals(function.getJointA().getQ(), fourBarJoint.getJointA().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointB().getQ(), fourBarJoint.getJointB().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointC().getQ(), fourBarJoint.getJointC().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointD().getQ(), fourBarJoint.getJointD().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointA().getQd(), fourBarJoint.getJointA().getQd(), MID_EPSILON);
            assertEquals(function.getJointB().getQd(), fourBarJoint.getJointB().getQd(), MID_EPSILON);
            assertEquals(function.getJointC().getQd(), fourBarJoint.getJointC().getQd(), MID_EPSILON);
            assertEquals(function.getJointD().getQd(), fourBarJoint.getJointD().getQd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointA().getQdd(), fourBarJoint.getJointA().getQdd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointB().getQdd(), fourBarJoint.getJointB().getQdd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointC().getQdd(), fourBarJoint.getJointC().getQdd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointD().getQdd(), fourBarJoint.getJointD().getQdd(), MID_EPSILON);

            RevoluteJointBasics topJoint, bottomJoint;
            if (fourBarJoint.getFrameBeforeJoint() == fourBarJoint.getJointA().getFrameBeforeJoint())
            {
               topJoint = fourBarJoint.getJointA();
               bottomJoint = fourBarJoint.getJointD();
            }
            else
            {
               topJoint = fourBarJoint.getJointB();
               bottomJoint = fourBarJoint.getJointC();
            }

            SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(topJoint.getPredecessor(), worldFrame);
            SpatialAcceleration expectedAcceleration = new SpatialAcceleration(spatialAccelerationCalculator.getRelativeAcceleration(topJoint.getPredecessor(),
                                                                                                                                     bottomJoint.getSuccessor()));
            expectedAcceleration.setBaseFrame(topJoint.getFrameBeforeJoint());
            expectedAcceleration.setBodyFrame(bottomJoint.getFrameAfterJoint());
            expectedAcceleration.changeFrame(bottomJoint.getFrameAfterJoint());

            SpatialAccelerationReadOnly actualAcceleration = fourBarJoint.getJointAcceleration();

            assertSpatialAccelerationEquals("Iteration " + i,
                                            expectedAcceleration,
                                            actualAcceleration,
                                            Math.max(1.0, expectedAcceleration.length()) * MID_EPSILON);
         }
      }
   }

   @Test
   public void testGetSuccessorAcceleration()
   {
      Random random = new Random(4754756);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         FourBarKinematicLoopFunction function = generator.apply("copy").getFourBarFunction();

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMaster = EuclidCoreRandomTools.nextDouble(random,
                                                              fourBarJoint.getMasterJoint().getJointLimitLower(),
                                                              fourBarJoint.getMasterJoint().getJointLimitUpper());
            double qdMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double qddMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            fourBarJoint.getMasterJoint().setQ(qMaster);
            fourBarJoint.getMasterJoint().setQd(qdMaster);
            fourBarJoint.getMasterJoint().setQdd(qddMaster);
            function.getMasterJoint().setQ(qMaster);
            function.getMasterJoint().setQd(qdMaster);
            function.getMasterJoint().setQdd(qddMaster);

            fourBarJoint.updateFramesRecursively();
            function.updateState(true, true);
            function.getJointA().getPredecessor().updateFramesRecursively();

            assertEquals(function.getJointA().getQ(), fourBarJoint.getJointA().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointB().getQ(), fourBarJoint.getJointB().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointC().getQ(), fourBarJoint.getJointC().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointD().getQ(), fourBarJoint.getJointD().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointA().getQd(), fourBarJoint.getJointA().getQd(), MID_EPSILON);
            assertEquals(function.getJointB().getQd(), fourBarJoint.getJointB().getQd(), MID_EPSILON);
            assertEquals(function.getJointC().getQd(), fourBarJoint.getJointC().getQd(), MID_EPSILON);
            assertEquals(function.getJointD().getQd(), fourBarJoint.getJointD().getQd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointA().getQdd(), fourBarJoint.getJointA().getQdd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointB().getQdd(), fourBarJoint.getJointB().getQdd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointC().getQdd(), fourBarJoint.getJointC().getQdd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointD().getQdd(), fourBarJoint.getJointD().getQdd(), MID_EPSILON);

            SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(function.getJointA().getPredecessor(), worldFrame);
            SpatialAcceleration expectedAcceleration = new SpatialAcceleration(spatialAccelerationCalculator.getRelativeAcceleration(function.getJointA()
                                                                                                                                             .getPredecessor(),
                                                                                                                                     function.getJointD()
                                                                                                                                             .getSuccessor()));
            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            fourBarJoint.getSuccessorAcceleration(actualAcceleration);

            assertSpatialAccelerationEquals("Iteration " + i,
                                            expectedAcceleration,
                                            actualAcceleration,
                                            Math.max(1.0, expectedAcceleration.length()) * MID_EPSILON);
         }
      }
   }

   @Test
   public void testGetJointBiasAcceleration()
   {
      Random random = new Random(4754756);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");
         FourBarKinematicLoopFunction function = generator.apply("copy").getFourBarFunction();

         for (int i = 0; i < ITERATIONS; i++)
         { // Non-zero velocity and no-acceleration
            double q = EuclidCoreRandomTools.nextDouble(random, fourBarJoint.getJointLimitLower(), fourBarJoint.getJointLimitUpper());
            double qd = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double qdd = 0.0;

            fourBarJoint.setQ(q);
            fourBarJoint.setQd(qd);
            fourBarJoint.setQdd(qdd);
            fourBarJoint.updateFramesRecursively();

            assertEquals(qdd, fourBarJoint.getQdd(), SMALL_EPSILON);

            function.getMasterJoint().setQ(fourBarJoint.getMasterJoint().getQ());
            function.getMasterJoint().setQd(fourBarJoint.getMasterJoint().getQd());
            function.getMasterJoint().setQdd(fourBarJoint.getMasterJoint().getQdd());

            function.updateState(true, true);
            function.getJointA().getPredecessor().updateFramesRecursively();

            assertEquals(function.getJointA().getQ(), fourBarJoint.getJointA().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointB().getQ(), fourBarJoint.getJointB().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointC().getQ(), fourBarJoint.getJointC().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointD().getQ(), fourBarJoint.getJointD().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointA().getQd(), fourBarJoint.getJointA().getQd(), MID_EPSILON);
            assertEquals(function.getJointB().getQd(), fourBarJoint.getJointB().getQd(), MID_EPSILON);
            assertEquals(function.getJointC().getQd(), fourBarJoint.getJointC().getQd(), MID_EPSILON);
            assertEquals(function.getJointD().getQd(), fourBarJoint.getJointD().getQd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointA().getQdd(), fourBarJoint.getJointA().getQdd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointB().getQdd(), fourBarJoint.getJointB().getQdd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointC().getQdd(), fourBarJoint.getJointC().getQdd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointD().getQdd(), fourBarJoint.getJointD().getQdd(), MID_EPSILON);

            RevoluteJointBasics topJoint, bottomJoint;
            if (fourBarJoint.getFrameBeforeJoint() == fourBarJoint.getJointA().getFrameBeforeJoint())
            {
               topJoint = fourBarJoint.getJointA();
               bottomJoint = fourBarJoint.getJointD();
            }
            else
            {
               topJoint = fourBarJoint.getJointB();
               bottomJoint = fourBarJoint.getJointC();
            }

            SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(topJoint.getPredecessor(), worldFrame);
            SpatialAcceleration expectedAcceleration = new SpatialAcceleration(spatialAccelerationCalculator.getRelativeAcceleration(topJoint.getPredecessor(),
                                                                                                                                     bottomJoint.getSuccessor()));
            expectedAcceleration.setBaseFrame(topJoint.getFrameBeforeJoint());
            expectedAcceleration.setBodyFrame(bottomJoint.getFrameAfterJoint());
            expectedAcceleration.changeFrame(bottomJoint.getFrameAfterJoint());

            SpatialAccelerationReadOnly actualAcceleration = fourBarJoint.getJointBiasAcceleration();

            assertSpatialAccelerationEquals("Iteration " + i,
                                            expectedAcceleration,
                                            actualAcceleration,
                                            Math.max(1.0, expectedAcceleration.length()) * MID_EPSILON);
         }

         for (int i = 0; i < ITERATIONS; i++)
         { // No velocity and non-zero acceleration
            double q = EuclidCoreRandomTools.nextDouble(random, fourBarJoint.getJointLimitLower(), fourBarJoint.getJointLimitUpper());
            double qd = 0.0;
            double qdd = EuclidCoreRandomTools.nextDouble(random, 10.0);

            fourBarJoint.setQ(q);
            fourBarJoint.setQd(qd);
            fourBarJoint.setQdd(qdd);
            fourBarJoint.updateFramesRecursively();

            function.getMasterJoint().setQ(fourBarJoint.getMasterJoint().getQ());
            function.getMasterJoint().setQd(fourBarJoint.getMasterJoint().getQd());
            function.getMasterJoint().setQdd(fourBarJoint.getMasterJoint().getQdd());

            function.updateState(true, true);
            function.getJointA().getPredecessor().updateFramesRecursively();

            assertEquals(function.getJointA().getQ(), fourBarJoint.getJointA().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointB().getQ(), fourBarJoint.getJointB().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointC().getQ(), fourBarJoint.getJointC().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointD().getQ(), fourBarJoint.getJointD().getQ(), SMALL_EPSILON);
            assertEquals(function.getJointA().getQd(), fourBarJoint.getJointA().getQd(), MID_EPSILON);
            assertEquals(function.getJointB().getQd(), fourBarJoint.getJointB().getQd(), MID_EPSILON);
            assertEquals(function.getJointC().getQd(), fourBarJoint.getJointC().getQd(), MID_EPSILON);
            assertEquals(function.getJointD().getQd(), fourBarJoint.getJointD().getQd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointA().getQdd(), fourBarJoint.getJointA().getQdd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointB().getQdd(), fourBarJoint.getJointB().getQdd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointC().getQdd(), fourBarJoint.getJointC().getQdd(), MID_EPSILON);
            assertEqualsVarEpsilon(function.getJointD().getQdd(), fourBarJoint.getJointD().getQdd(), MID_EPSILON);

            RevoluteJointBasics topJoint, bottomJoint;
            if (fourBarJoint.getFrameBeforeJoint() == fourBarJoint.getJointA().getFrameBeforeJoint())
            {
               topJoint = fourBarJoint.getJointA();
               bottomJoint = fourBarJoint.getJointD();
            }
            else
            {
               topJoint = fourBarJoint.getJointB();
               bottomJoint = fourBarJoint.getJointC();
            }

            SpatialAcceleration expectedAcceleration = new SpatialAcceleration(bottomJoint.getFrameAfterJoint(),
                                                                               topJoint.getFrameBeforeJoint(),
                                                                               bottomJoint.getFrameAfterJoint());

            SpatialAccelerationReadOnly actualAcceleration = fourBarJoint.getJointBiasAcceleration();

            assertSpatialAccelerationEquals("Iteration " + i,
                                            expectedAcceleration,
                                            actualAcceleration,
                                            Math.max(1.0, expectedAcceleration.length()) * SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testGetUnitJointAcceleration()
   {
      Random random = new Random(8623435);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMaster = EuclidCoreRandomTools.nextDouble(random,
                                                              fourBarJoint.getMasterJoint().getJointLimitLower(),
                                                              fourBarJoint.getMasterJoint().getJointLimitUpper());
            double qdMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double qddMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            fourBarJoint.getMasterJoint().setQ(qMaster);
            fourBarJoint.getMasterJoint().setQd(0.0); // Cancel out the bias accelerations
            fourBarJoint.getMasterJoint().setQdd(qddMaster);
            fourBarJoint.updateFramesRecursively();

            SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(fourBarJoint.getJointA().getPredecessor(),
                                                                                                            worldFrame);
            SpatialAcceleration expectedAcceleration = new SpatialAcceleration(spatialAccelerationCalculator.getRelativeAcceleration(fourBarJoint.getJointA()
                                                                                                                                                 .getPredecessor(),
                                                                                                                                     fourBarJoint.getJointD()
                                                                                                                                                 .getSuccessor()));
            expectedAcceleration.changeFrame(fourBarJoint.getFrameAfterJoint());
            expectedAcceleration.setBaseFrame(fourBarJoint.getFrameBeforeJoint());
            expectedAcceleration.setBodyFrame(fourBarJoint.getFrameAfterJoint());
            expectedAcceleration.scale(1.0 / fourBarJoint.getQdd());

            MecanoTestTools.assertSpatialAccelerationEquals("Iteration " + i, expectedAcceleration, fourBarJoint.getUnitJointAcceleration(), SMALL_EPSILON);

            fourBarJoint.getMasterJoint().setQd(qdMaster); // Introduce bias acceleration, unit-acceleration shouldn't change.
            fourBarJoint.updateFramesRecursively();
            MecanoTestTools.assertSpatialAccelerationEquals("Iteration " + i, expectedAcceleration, fourBarJoint.getUnitJointAcceleration(), SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testGetUnitSuccessorAcceleration()
   {
      Random random = new Random(8623435);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMaster = EuclidCoreRandomTools.nextDouble(random,
                                                              fourBarJoint.getMasterJoint().getJointLimitLower(),
                                                              fourBarJoint.getMasterJoint().getJointLimitUpper());
            double qdMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double qddMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            fourBarJoint.getMasterJoint().setQ(qMaster);
            fourBarJoint.getMasterJoint().setQd(0.0); // Cancel out the bias accelerations
            fourBarJoint.getMasterJoint().setQdd(qddMaster);
            fourBarJoint.updateFramesRecursively();

            SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(fourBarJoint.getJointA().getPredecessor(),
                                                                                                            worldFrame);
            SpatialAcceleration expectedAcceleration = new SpatialAcceleration(spatialAccelerationCalculator.getRelativeAcceleration(fourBarJoint.getJointA()
                                                                                                                                                 .getPredecessor(),
                                                                                                                                     fourBarJoint.getJointD()
                                                                                                                                                 .getSuccessor()));
            expectedAcceleration.setBodyFrame(fourBarJoint.getSuccessor().getBodyFixedFrame());
            expectedAcceleration.changeFrame(fourBarJoint.getSuccessor().getBodyFixedFrame());
            expectedAcceleration.scale(1.0 / fourBarJoint.getQdd());

            MecanoTestTools.assertSpatialAccelerationEquals("Iteration " + i, expectedAcceleration, fourBarJoint.getUnitSuccessorAcceleration(), SMALL_EPSILON);

            fourBarJoint.getMasterJoint().setQd(qdMaster); // Introduce bias acceleration, unit-acceleration shouldn't change.
            fourBarJoint.updateFramesRecursively();
            MecanoTestTools.assertSpatialAccelerationEquals("Iteration " + i, expectedAcceleration, fourBarJoint.getUnitSuccessorAcceleration(), SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testGetUnitPredecessorAcceleration()
   {
      Random random = new Random(8623435);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");

         for (int i = 0; i < ITERATIONS; i++)
         {
            double qMaster = EuclidCoreRandomTools.nextDouble(random,
                                                              fourBarJoint.getMasterJoint().getJointLimitLower(),
                                                              fourBarJoint.getMasterJoint().getJointLimitUpper());
            double qdMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double qddMaster = EuclidCoreRandomTools.nextDouble(random, 10.0);
            fourBarJoint.getMasterJoint().setQ(qMaster);
            fourBarJoint.getMasterJoint().setQd(0.0); // Cancel out the bias accelerations
            fourBarJoint.getMasterJoint().setQdd(qddMaster);
            fourBarJoint.updateFramesRecursively();

            SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(fourBarJoint.getJointA().getPredecessor(),
                                                                                                            worldFrame);
            SpatialAcceleration expectedAcceleration = new SpatialAcceleration(spatialAccelerationCalculator.getRelativeAcceleration(fourBarJoint.getJointD()
                                                                                                                                                 .getSuccessor(),
                                                                                                                                     fourBarJoint.getJointA()
                                                                                                                                                 .getPredecessor()));
            expectedAcceleration.setBaseFrame(fourBarJoint.getSuccessor().getBodyFixedFrame());
            expectedAcceleration.scale(1.0 / fourBarJoint.getQdd());

            MecanoTestTools.assertSpatialAccelerationEquals("Iteration " + i,
                                                            expectedAcceleration,
                                                            fourBarJoint.getUnitPredecessorAcceleration(),
                                                            SMALL_EPSILON);

            fourBarJoint.getMasterJoint().setQd(qdMaster); // Introduce bias acceleration, unit-acceleration shouldn't change.
            fourBarJoint.updateFramesRecursively();
            MecanoTestTools.assertSpatialAccelerationEquals("Iteration " + i,
                                                            expectedAcceleration,
                                                            fourBarJoint.getUnitPredecessorAcceleration(),
                                                            SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testSetTau()
   {
      // TODO Should check against the InverseDynamicsCalculator.
      Random random = new Random(3453897);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");

         for (int i = 0; i < ITERATIONS; i++)
         {
            double q = EuclidCoreRandomTools.nextDouble(random, fourBarJoint.getJointLimitLower(), fourBarJoint.getJointLimitUpper());
            double qd = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double qdd = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double tau = EuclidCoreRandomTools.nextDouble(random, 10.0);

            fourBarJoint.setQ(q);
            fourBarJoint.setQd(qd);
            fourBarJoint.setQdd(qdd);
            fourBarJoint.setTau(tau);

            assertEquals(tau, fourBarJoint.getTau(), SMALL_EPSILON, "Iteration " + i);
         }
      }
   }

   @Test
   public void testSetJointTorque()
   {
      Random random = new Random(3453897);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");

         for (int i = 0; i < ITERATIONS; i++)
         {
            double q = EuclidCoreRandomTools.nextDouble(random, fourBarJoint.getJointLimitLower(), fourBarJoint.getJointLimitUpper());
            double qd = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double qdd = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double tau = EuclidCoreRandomTools.nextDouble(random, 10.0);
            Vector3D jointTorque = new Vector3D();
            jointTorque.setAndScale(tau, fourBarJoint.getJointAxis());
            Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, fourBarJoint.getJointAxis(), true);
            jointTorque.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonalToAxis, jointTorque);

            fourBarJoint.setQ(q);
            fourBarJoint.setQd(qd);
            fourBarJoint.setQdd(qdd);
            fourBarJoint.setJointTorque(jointTorque);

            assertEquals(tau, fourBarJoint.getTau(), SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testSetJointForce()
   {
      Random random = new Random(3453897);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");

         for (int i = 0; i < ITERATIONS; i++)
         {
            double q = EuclidCoreRandomTools.nextDouble(random, fourBarJoint.getJointLimitLower(), fourBarJoint.getJointLimitUpper());
            double qd = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double qdd = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double tau = EuclidCoreRandomTools.nextDouble(random, 10.0);
            Vector3D jointForce = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);

            fourBarJoint.setQ(q);
            fourBarJoint.setQd(qd);
            fourBarJoint.setQdd(qdd);
            fourBarJoint.setTau(tau);
            assertEquals(tau, fourBarJoint.getTau(), SMALL_EPSILON);

            // Assert this does nothing
            fourBarJoint.setJointForce(jointForce);

            assertEquals(tau, fourBarJoint.getTau(), SMALL_EPSILON);
         }
      }
   }

   @Test
   public void testGetJointWrench()
   {
      Random random = new Random(3453897);

      for (Function<String, InvertedFourBarJoint> generator : createInvertedFourBarExampleGenerators())
      { // Test with known four bar
         InvertedFourBarJoint fourBarJoint = generator.apply("fourBar1");

         GeometricJacobianCalculator jacobian = new GeometricJacobianCalculator();
         jacobian.setKinematicChain(new OneDoFJointBasics[] {fourBarJoint});
         jacobian.setJacobianFrame(fourBarJoint.getFrameAfterJoint());

         for (int i = 0; i < ITERATIONS; i++)
         {
            double q = EuclidCoreRandomTools.nextDouble(random, fourBarJoint.getJointLimitLower(), fourBarJoint.getJointLimitUpper());
            double qd = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double qdd = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double tau = EuclidCoreRandomTools.nextDouble(random, 100.0);

            fourBarJoint.setQ(q);
            fourBarJoint.setQd(qd);
            fourBarJoint.setQdd(qdd);
            fourBarJoint.setTau(tau);
            assertEquals(tau, fourBarJoint.getTau(), SMALL_EPSILON);
            fourBarJoint.updateFramesRecursively();

            Wrench jointWrench = new Wrench(fourBarJoint.getJointWrench());
            jointWrench.setBodyFrame(fourBarJoint.getFrameAfterJoint());
            double actualTau = fourBarJoint.getUnitJointTwist().dot(jointWrench);
            assertEquals(fourBarJoint.getTau(), actualTau, SMALL_EPSILON);

            // Testing with the regular Jacobian transpose approach. 
            jacobian.reset();
            DMatrixRMaj jointTorque = new DMatrixRMaj(1, 0);
            jacobian.getJointTorques(fourBarJoint.getJointWrench(), jointTorque);

            assertEquals(fourBarJoint.getTau(), jointTorque.get(0), SMALL_EPSILON);
         }
      }
   }

   private static void assertEqualsVarEpsilon(double expected, double actual, double epsilon)
   {
      double varEpsilon = Math.max(1.0, Math.abs(expected)) * epsilon;
      assertEquals(expected, actual, varEpsilon);
   }

   private static void assertTwistEquals(String messagePrefix, TwistReadOnly expectedTwist, TwistReadOnly actualTwist, double epsilon)
   {
      try
      {
         assertEquals(expectedTwist.getBodyFrame().getName(), actualTwist.getBodyFrame().getName());
         assertEquals(expectedTwist.getBaseFrame().getName(), actualTwist.getBaseFrame().getName());
         assertEquals(expectedTwist.getReferenceFrame().getName(), actualTwist.getReferenceFrame().getName());
         EuclidCoreTestTools.assertTuple3DEquals(expectedTwist.getAngularPart(), actualTwist.getAngularPart(), epsilon);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTwist.getLinearPart(), actualTwist.getLinearPart(), epsilon);
      }
      catch (AssertionError e)
      {
         Vector3D angularError = new Vector3D();
         Vector3D linearError = new Vector3D();
         angularError.sub(expectedTwist.getAngularPart(), actualTwist.getAngularPart());
         linearError.sub(expectedTwist.getLinearPart(), actualTwist.getLinearPart());
         String errorMessage = String.format("Expected:\n%s\nbut was:\n%s\nDifference: angular=%s, linear=%s",
                                             MecanoIOTools.getTwistString(EuclidCoreTestTools.DEFAULT_FORMAT, expectedTwist),
                                             MecanoIOTools.getTwistString(EuclidCoreTestTools.DEFAULT_FORMAT, actualTwist),
                                             Double.toString(angularError.length()),
                                             Double.toString(linearError.length()));
         throw new AssertionFailedError(EuclidCoreTestTools.addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   private static void assertSpatialAccelerationEquals(String messagePrefix, SpatialAccelerationReadOnly expectedAcceleration,
                                                       SpatialAccelerationReadOnly actualAcceleration, double epsilon)
   {
      try
      {
         assertEquals(expectedAcceleration.getBodyFrame().getName(), actualAcceleration.getBodyFrame().getName());
         assertEquals(expectedAcceleration.getBaseFrame().getName(), actualAcceleration.getBaseFrame().getName());
         assertEquals(expectedAcceleration.getReferenceFrame().getName(), actualAcceleration.getReferenceFrame().getName());
         EuclidCoreTestTools.assertTuple3DEquals(expectedAcceleration.getAngularPart(), actualAcceleration.getAngularPart(), epsilon);
         EuclidCoreTestTools.assertTuple3DEquals(expectedAcceleration.getLinearPart(), actualAcceleration.getLinearPart(), epsilon);
      }
      catch (AssertionError e)
      {
         Vector3D angularError = new Vector3D();
         Vector3D linearError = new Vector3D();
         angularError.sub(expectedAcceleration.getAngularPart(), actualAcceleration.getAngularPart());
         linearError.sub(expectedAcceleration.getLinearPart(), actualAcceleration.getLinearPart());
         String errorMessage = String.format("Expected:\n%s\nbut was:\n%s\nDifference: angular=%s, linear=%s",
                                             MecanoIOTools.getSpatialAccelerationString(EuclidCoreTestTools.DEFAULT_FORMAT, expectedAcceleration),
                                             MecanoIOTools.getSpatialAccelerationString(EuclidCoreTestTools.DEFAULT_FORMAT, actualAcceleration),
                                             Double.toString(angularError.length()),
                                             Double.toString(linearError.length()));
         throw new AssertionFailedError(EuclidCoreTestTools.addPrefixToMessage(messagePrefix, errorMessage));
      }
   }

   static RevoluteJoint[] nextInvertedFourBar(Random random, String prefix, Vector3DReadOnly axis, int masterJointIndex, boolean flipAxesRandomly)
   {
      return nextInvertedFourBar(random,
                                 prefix,
                                 new RigidBody(prefix + "Root", worldFrame),
                                 joint -> MultiBodySystemRandomTools.nextRigidBody(random, prefix + "BodyCD", joint),
                                 axis,
                                 masterJointIndex,
                                 flipAxesRandomly);
   }

   private static RevoluteJoint[] nextInvertedFourBar(Random random, String prefix, RigidBodyBasics predecessor,
                                                      Function<RevoluteJointBasics, RigidBodyBasics> successorFactory, Vector3DReadOnly axis,
                                                      int masterJointIndex, boolean flipAxesRandomly)
   {
      List<Point2D> vertices = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 5.0, 4);
      int flippedIndex = random.nextInt(4);
      Collections.swap(vertices, flippedIndex, (flippedIndex + 1) % 4);

      Point2D A = vertices.get(0);
      Point2D B = vertices.get(1);
      Point2D C = vertices.get(2);
      Point2D D = vertices.get(3);

      Vector2D AB = new Vector2D();
      Vector2D AC = new Vector2D();
      Vector2D AD = new Vector2D();
      AB.sub(B, A);
      AC.sub(C, A);
      AD.sub(D, A);

      Vector3D axisA = new Vector3D(axis);
      Vector3D axisB = new Vector3D(axis);
      Vector3D axisC = new Vector3D(axis);
      Vector3D axisD = new Vector3D(axis);

      FramePoint3D jointAPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame);
      ReferenceFrame fourBarLocalFrame = GeometryTools.constructReferenceFrameFromPointAndAxis("LocalFrame",
                                                                                               jointAPosition,
                                                                                               Axis3D.Z,
                                                                                               new FrameVector3D(worldFrame, axis));

      FramePoint3D jointBPosition = new FramePoint3D(fourBarLocalFrame, AB);
      jointBPosition.setZ(EuclidCoreRandomTools.nextDouble(random));
      jointBPosition.changeFrame(worldFrame);
      FramePoint3D jointCPosition = new FramePoint3D(fourBarLocalFrame, AC);
      jointCPosition.setZ(EuclidCoreRandomTools.nextDouble(random));
      jointCPosition.changeFrame(worldFrame);
      FramePoint3D jointDPosition = new FramePoint3D(fourBarLocalFrame, AD);
      jointDPosition.setZ(EuclidCoreRandomTools.nextDouble(random));
      jointDPosition.changeFrame(worldFrame);

      if (flipAxesRandomly)
      { // Flip any but master joint such that the clockwise order is respected.
         if (masterJointIndex != 0 && random.nextBoolean())
            axisA.negate();
         if (masterJointIndex != 1 && random.nextBoolean())
            axisB.negate();
         if (masterJointIndex != 2 && random.nextBoolean())
            axisC.negate();
         if (masterJointIndex != 3 && random.nextBoolean())
            axisD.negate();
      }

      RevoluteJoint jointA = new RevoluteJoint(prefix + "JointA", predecessor, jointAPosition, axisA);
      RevoluteJoint jointB = new RevoluteJoint(prefix + "JointB", predecessor, jointBPosition, axisB);
      RigidBody bodyDA = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "BodyDA", jointA);
      RigidBody bodyBC = MultiBodySystemRandomTools.nextRigidBody(random, prefix + "BodyBC", jointB);
      jointCPosition.changeFrame(jointB.getFrameAfterJoint());
      RevoluteJoint jointC = new RevoluteJoint(prefix + "JointC", bodyBC, jointCPosition, axisC);
      jointDPosition.changeFrame(jointA.getFrameAfterJoint());
      RevoluteJoint jointD = new RevoluteJoint(prefix + "JointD", bodyDA, jointDPosition, axisD);

      RigidBodyBasics bodyCD = successorFactory.apply(jointD);
      jointCPosition.changeFrame(jointD.getFrameAfterJoint());
      jointC.setupLoopClosure(bodyCD, new RigidBodyTransform(new Quaternion(), jointCPosition));
      return new RevoluteJoint[] {jointA, jointB, jointC, jointD};
   }

   private static List<Function<String, InvertedFourBarJoint>> createInvertedFourBarExampleGenerators()
   {
      List<Function<String, InvertedFourBarJoint>> generators = new ArrayList<>();
      generators.add(name -> createKnownInvertedFourBarJoint1(name, 0, SMALL_EPSILON));
      generators.add(name -> createKnownInvertedFourBarJoint2(name, 0, SMALL_EPSILON));
      return generators;
   }

   private static InvertedFourBarJoint createKnownInvertedFourBarJoint1(String name, int masterJointIndex, double solverTolerance)
   {
      Point2D A = new Point2D(0.002, -0.189);
      Point2D B = new Point2D(-0.019, -0.144);
      Point2D C = new Point2D(0.023, -0.245);
      Point2D D = new Point2D(-0.015, -0.278);
      return createInvertedFourBarJoint(name, A, B, C, D, masterJointIndex, solverTolerance);
   }

   private static InvertedFourBarJoint createKnownInvertedFourBarJoint2(String name, int masterJointIndex, double solverTolerance)
   {
      Point2D A = new Point2D(0.227, 0.1);
      Point2D B = new Point2D(0.227, -0.1);
      Point2D C = new Point2D(0.427, 0.1);
      Point2D D = new Point2D(0.427, -0.1);
      return createInvertedFourBarJoint(name, A, B, C, D, masterJointIndex, solverTolerance);
   }

   private static InvertedFourBarJoint createInvertedFourBarJoint(String name, Point2D A, Point2D B, Point2D C, Point2D D, int masterJointIndex,
                                                                  double solverTolerance)
   {
      RevoluteJoint[] fourBarJoints = createInvertedFourBarJoints(A, B, C, D);
      InvertedFourBarJoint joint = new InvertedFourBarJoint(name, fourBarJoints, masterJointIndex);

      FramePoint3D offset = new FramePoint3D();
      if (fourBarJoints[3] != joint.getJointD())
      {
         joint.updateFramesRecursively();
         offset.setToZero(joint.getJointC().getFrameAfterJoint());
         offset.changeFrame(joint.getFrameAfterJoint());
      }
      new RigidBody("bodyCD", joint, 0, 0, 0, 0, offset);
      joint.setIKSolver(new InvertedFourBarJointIKBinarySolver(solverTolerance));
      return joint;
   }

   private static RevoluteJoint[] createInvertedFourBarJoints(Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C, Point2DReadOnly D)
   {
      Vector2D AB = new Vector2D();
      Vector2D AC = new Vector2D();
      Vector2D AD = new Vector2D();
      AB.sub(B, A);
      AC.sub(C, A);
      AD.sub(D, A);

      FramePoint3D jointAPosition = new FramePoint3D(worldFrame, A);
      ReferenceFrame fourBarLocalFrame = GeometryTools.constructReferenceFrameFromPointAndAxis("LocalFrame",
                                                                                               jointAPosition,
                                                                                               Axis3D.Z,
                                                                                               new FrameVector3D(worldFrame, Axis3D.Y));

      FramePoint3D jointBPosition = new FramePoint3D(fourBarLocalFrame, AB);
      jointBPosition.changeFrame(worldFrame);
      FramePoint3D jointCPosition = new FramePoint3D(fourBarLocalFrame, AC);
      jointCPosition.changeFrame(worldFrame);
      FramePoint3D jointDPosition = new FramePoint3D(fourBarLocalFrame, AD);
      jointDPosition.changeFrame(worldFrame);

      RigidBody rootBody = new RigidBody("root", worldFrame);
      RevoluteJoint jointA = new RevoluteJoint("jointA", rootBody, jointAPosition, Axis3D.Y);
      RevoluteJoint jointB = new RevoluteJoint("jointB", rootBody, jointBPosition, Axis3D.Y);
      RigidBody bodyDA = new RigidBody("bodyDA", jointA, 0, 0, 0, 0, new Vector3D());
      RigidBody bodyBC = new RigidBody("bodyBC", jointB, 0, 0, 0, 0, new Vector3D());
      jointCPosition.changeFrame(jointB.getFrameAfterJoint());
      RevoluteJoint jointC = new RevoluteJoint("jointC", bodyBC, jointCPosition, Axis3D.Y);
      jointDPosition.changeFrame(jointA.getFrameAfterJoint());
      RevoluteJoint jointD = new RevoluteJoint("jointD", bodyDA, jointDPosition, Axis3D.Y);

      RigidBody bodyCD = new RigidBody("bodyCD", jointD, 0, 0, 0, 0, new Vector3D());
      jointCPosition.changeFrame(jointD.getFrameAfterJoint());
      jointC.setupLoopClosure(bodyCD, new RigidBodyTransform(new Quaternion(), jointCPosition));

      return new RevoluteJoint[] {jointA, jointB, jointC, jointD};
   }
}
