package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.*;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;

public class TwistCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @ContinuousIntegrationTest(estimatedDuration = 0.01)
   @Test(timeout = 30000)
   public void testWithChainComposedOfPrismaticJoints() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 20;
      List<PrismaticJoint> prismaticJoints = ScrewTestTools.createRandomChainRobotWithPrismaticJoints(numberOfJoints, random);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, prismaticJoints.get(random.nextInt(numberOfJoints)).getPredecessor());

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(prismaticJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomVelocities(prismaticJoints, random, -10.0, 10.0);
         twistCalculator.compute();

         FrameVector3D cumulatedLinearVelocity = new FrameVector3D(worldFrame);

         for (PrismaticJoint joint : prismaticJoints)
         {
            RigidBody body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

            FrameVector3D jointAxis = joint.getJointAxis();
            cumulatedLinearVelocity.changeFrame(jointAxis.getReferenceFrame());
            cumulatedLinearVelocity.scaleAdd(joint.getQd(), jointAxis, cumulatedLinearVelocity);
            cumulatedLinearVelocity.changeFrame(bodyFrame);
            expectedTwist.setLinearPart(cumulatedLinearVelocity);

            assertTwistEquals(expectedTwist, actualTwist, 1.0e-12);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.01)
   @Test(timeout = 30000)
   public void testWithChainComposedOfRevoluteJointsAssertAngularVelocityOnly() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 20;
      List<RevoluteJoint> revoluteJoints = ScrewTestTools.createRandomChainRobot(numberOfJoints, random);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, revoluteJoints.get(random.nextInt(numberOfJoints)).getPredecessor());

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(revoluteJoints, random);
         ScrewTestTools.setRandomVelocities(revoluteJoints, random, -10.0, 10.0);
         twistCalculator.compute();

         FrameVector3D cumulatedAngularVelocity = new FrameVector3D(worldFrame);

         for (RevoluteJoint joint : revoluteJoints)
         {
            RigidBody body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

            FrameVector3D jointAxis = joint.getJointAxis();
            cumulatedAngularVelocity.changeFrame(jointAxis.getReferenceFrame());
            cumulatedAngularVelocity.scaleAdd(joint.getQd(), jointAxis, cumulatedAngularVelocity);
            cumulatedAngularVelocity.changeFrame(bodyFrame);
            expectedTwist.setAngularPart(cumulatedAngularVelocity);

            expectedTwist.checkReferenceFramesMatch(actualTwist);

            assertTrue(expectedTwist.angularPart.epsilonEquals(actualTwist.angularPart, 1.0e-12));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.02)
   @Test(timeout = 30000)
   public void testWithTreeComposedOfPrismaticJoints() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 100;
      List<PrismaticJoint> prismaticJoints = ScrewTestTools.createRandomTreeRobotWithPrismaticJoints(numberOfJoints, random);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, prismaticJoints.get(random.nextInt(numberOfJoints)).getPredecessor());

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(prismaticJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomVelocities(prismaticJoints, random, -10.0, 10.0);
         twistCalculator.compute();

         for (PrismaticJoint joint : prismaticJoints)
         {
            RigidBody body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

            RigidBody currentBody = body;
            FrameVector3D cumulatedLinearVelocity = new FrameVector3D(worldFrame);

            while (currentBody.getParentJoint() != null)
            {
               PrismaticJoint parentJoint = (PrismaticJoint) currentBody.getParentJoint();
               FrameVector3D jointAxis = parentJoint.getJointAxis();
               cumulatedLinearVelocity.changeFrame(jointAxis.getReferenceFrame());
               cumulatedLinearVelocity.scaleAdd(parentJoint.getQd(), jointAxis, cumulatedLinearVelocity);
               currentBody = parentJoint.getPredecessor();
            }

            cumulatedLinearVelocity.changeFrame(bodyFrame);
            expectedTwist.setLinearPart(cumulatedLinearVelocity);

            assertTwistEquals(expectedTwist, actualTwist, 1.0e-12);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testWithTreeComposedOfRevoluteJointsAssertAngularVelocity() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 100;
      List<RevoluteJoint> revoluteJoints = ScrewTestTools.createRandomTreeRobot(numberOfJoints, random);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, revoluteJoints.get(random.nextInt(numberOfJoints)).getPredecessor());

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(revoluteJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomVelocities(revoluteJoints, random, -10.0, 10.0);
         twistCalculator.compute();

         for (RevoluteJoint joint : revoluteJoints)
         {
            RigidBody body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

            RigidBody currentBody = body;
            FrameVector3D cumulatedAngularVelocity = new FrameVector3D(worldFrame);

            while (currentBody.getParentJoint() != null)
            {
               RevoluteJoint parentJoint = (RevoluteJoint) currentBody.getParentJoint();
               FrameVector3D jointAxis = parentJoint.getJointAxis();
               cumulatedAngularVelocity.changeFrame(jointAxis.getReferenceFrame());
               cumulatedAngularVelocity.scaleAdd(parentJoint.getQd(), jointAxis, cumulatedAngularVelocity);
               currentBody = parentJoint.getPredecessor();
            }

            cumulatedAngularVelocity.changeFrame(bodyFrame);
            expectedTwist.setAngularPart(cumulatedAngularVelocity);

            expectedTwist.checkReferenceFramesMatch(actualTwist);

            assertTrue(expectedTwist.angularPart.epsilonEquals(actualTwist.angularPart, 1.0e-12));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.01)
   @Test(timeout = 30000)
   public void testWithChainRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(234234L);

      int numberOfJoints = 10;
      List<OneDoFJoint> joints = ScrewTestTools.createRandomChainRobotWithOneDoFJoints(numberOfJoints, random);
      List<OneDoFJoint> jointsInFuture = Arrays.asList(ScrewTools.cloneOneDoFJointPath(joints.toArray(new OneDoFJoint[numberOfJoints])));

      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, joints.get(0).getPredecessor());

      double dt = 1.0e-8;

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(joints, random, -1.0, 1.0);
         ScrewTestTools.setRandomVelocities(joints, random, -1.0, 1.0);

         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            double q = joints.get(jointIndex).getQ() + dt * joints.get(jointIndex).getQd();
            jointsInFuture.get(jointIndex).setQ(q);
         }

         joints.get(0).updateFramesRecursively();
         jointsInFuture.get(0).updateFramesRecursively();

         twistCalculator.compute();

         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            OneDoFJoint joint = joints.get(jointIndex);
            RigidBody body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            ReferenceFrame bodyFrameInFuture = jointsInFuture.get(jointIndex).getSuccessor().getBodyFixedFrame();
            Twist expectedTwist = computeExpectedTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);

            assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.03)
   @Test(timeout = 30000)
   public void testWithTreeRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(234234L);

      int numberOfJoints = 100;
      List<OneDoFJoint> joints = ScrewTestTools.createRandomTreeRobotWithOneDoFJoints(numberOfJoints, random);
      List<OneDoFJoint> jointsInFuture = Arrays.asList(ScrewTools.cloneOneDoFJointPath(joints.toArray(new OneDoFJoint[numberOfJoints])));

      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, joints.get(0).getPredecessor());

      double dt = 1.0e-8;

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(joints, random, -1.0, 1.0);
         ScrewTestTools.setRandomVelocities(joints, random, -1.0, 1.0);

         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            double q = joints.get(jointIndex).getQ() + dt * joints.get(jointIndex).getQd();
            jointsInFuture.get(jointIndex).setQ(q);
         }

         joints.get(0).updateFramesRecursively();
         jointsInFuture.get(0).updateFramesRecursively();

         twistCalculator.compute();

         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            OneDoFJoint joint = joints.get(jointIndex);
            RigidBody body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            ReferenceFrame bodyFrameInFuture = jointsInFuture.get(jointIndex).getSuccessor().getBodyFixedFrame();
            Twist expectedTwist = computeExpectedTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);

            assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.05)
   @Test(timeout = 30000)
   public void testWithFloatingJointRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(435345L);

      int numberOfRevoluteJoints = 100;
      RandomFloatingChain floatingChain = new RandomFloatingChain(random, numberOfRevoluteJoints);
      SixDoFJoint floatingJoint = floatingChain.getRootJoint();
      List<RevoluteJoint> revoluteJoints = floatingChain.getRevoluteJoints();
      List<InverseDynamicsJoint> joints = floatingChain.getInverseDynamicsJoints();
      List<InverseDynamicsJoint> jointsInFuture = Arrays.asList(ScrewTools.cloneJointPath(joints.toArray(new InverseDynamicsJoint[numberOfRevoluteJoints
            + 1])));
      SixDoFJoint floatingJointInFuture = (SixDoFJoint) jointsInFuture.get(0);
      List<RevoluteJoint> revoluteJointsInFuture = ScrewTools.filterJoints(jointsInFuture, RevoluteJoint.class);

      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, joints.get(0).getPredecessor());

      double dt = 1.0e-8;

      for (int i = 0; i < 100; i++)
      {
         floatingJoint.setRotation(RandomGeometry.nextQuaternion(random));
         floatingJoint.setPosition(RandomGeometry.nextPoint3D(random, -10.0, 10.0));
         Twist floatingJointTwist = Twist.generateRandomTwist(random, floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(),
                                                              floatingJoint.getFrameAfterJoint());
         floatingJoint.setJointTwist(floatingJointTwist);

         floatingJointInFuture.setJointPositionVelocityAndAcceleration(floatingJoint);
         ScrewTestTools.integrateVelocities(floatingJointInFuture, dt);

         ScrewTestTools.setRandomPositions(revoluteJoints, random, -1.0, 1.0);
         ScrewTestTools.setRandomVelocities(revoluteJoints, random, -1.0, 1.0);

         for (int jointIndex = 0; jointIndex < numberOfRevoluteJoints; jointIndex++)
         {
            double q = revoluteJoints.get(jointIndex).getQ() + dt * revoluteJoints.get(jointIndex).getQd();
            revoluteJointsInFuture.get(jointIndex).setQ(q);
         }

         floatingJoint.updateFramesRecursively();
         floatingJointInFuture.updateFramesRecursively();

         twistCalculator.compute();

         for (int jointIndex = 0; jointIndex < numberOfRevoluteJoints + 1; jointIndex++)
         {
            InverseDynamicsJoint joint = joints.get(jointIndex);
            RigidBody body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            ReferenceFrame bodyFrameInFuture = jointsInFuture.get(jointIndex).getSuccessor().getBodyFixedFrame();
            Twist expectedTwist = computeExpectedTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);

            assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);

            Point3D bodyFixedPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
            FramePoint3D frameBodyFixedPoint = new FramePoint3D(bodyFrame, bodyFixedPoint);
            FrameVector3D actualLinearVelocity = new FrameVector3D();
            twistCalculator.getLinearVelocityOfBodyFixedPoint(body, frameBodyFixedPoint, actualLinearVelocity);
            FrameVector3D expectedLinearVelocity = computeExpectedLinearVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture, bodyFixedPoint);

            expectedLinearVelocity.checkReferenceFrameMatch(actualLinearVelocity);
            EuclidCoreTestTools.assertTuple3DEquals(expectedLinearVelocity.getVector(), actualLinearVelocity.getVector(), 1.0e-5);

            FrameVector3D expectedAngularVelocity = computeAngularVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);
            FrameVector3D actualAngularVelocity = new FrameVector3D();
            twistCalculator.getAngularVelocityOfBody(body, actualAngularVelocity);

            expectedAngularVelocity.checkReferenceFrameMatch(actualAngularVelocity);
            EuclidCoreTestTools.assertTuple3DEquals(expectedAngularVelocity.getVector(), actualAngularVelocity.getVector(), 1.0e-5);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.4)
   @Test(timeout = 30000)
   public void testRelativeTwistWithFloatingJointRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(435345L);

      int numberOfRevoluteJoints = 100;
      RandomFloatingChain floatingChain = new RandomFloatingChain(random, numberOfRevoluteJoints);
      SixDoFJoint floatingJoint = floatingChain.getRootJoint();
      List<RevoluteJoint> revoluteJoints = floatingChain.getRevoluteJoints();
      List<InverseDynamicsJoint> joints = floatingChain.getInverseDynamicsJoints();
      List<InverseDynamicsJoint> jointsInFuture = Arrays.asList(ScrewTools.cloneJointPath(joints.toArray(new InverseDynamicsJoint[numberOfRevoluteJoints
            + 1])));
      SixDoFJoint floatingJointInFuture = (SixDoFJoint) jointsInFuture.get(0);
      List<RevoluteJoint> revoluteJointsInFuture = ScrewTools.filterJoints(jointsInFuture, RevoluteJoint.class);

      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, joints.get(random.nextInt(numberOfRevoluteJoints)).getPredecessor());

      double dt = 1.0e-8;

      for (int i = 0; i < 50; i++)
      {
         floatingJoint.setRotation(RandomGeometry.nextQuaternion(random));
         floatingJoint.setPosition(RandomGeometry.nextPoint3D(random, -10.0, 10.0));
         Twist floatingJointTwist = Twist.generateRandomTwist(random, floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(),
                                                              floatingJoint.getFrameAfterJoint());
         floatingJoint.setJointTwist(floatingJointTwist);

         floatingJointInFuture.setJointPositionVelocityAndAcceleration(floatingJoint);
         ScrewTestTools.integrateVelocities(floatingJointInFuture, dt);

         ScrewTestTools.setRandomPositions(revoluteJoints, random, -1.0, 1.0);
         ScrewTestTools.setRandomVelocities(revoluteJoints, random, -1.0, 1.0);

         for (int jointIndex = 0; jointIndex < numberOfRevoluteJoints; jointIndex++)
         {
            double q = revoluteJoints.get(jointIndex).getQ() + dt * revoluteJoints.get(jointIndex).getQd();
            revoluteJointsInFuture.get(jointIndex).setQ(q);
         }

         floatingJoint.updateFramesRecursively();
         floatingJointInFuture.updateFramesRecursively();

         twistCalculator.compute();

         for (int jointIndex = 0; jointIndex < numberOfRevoluteJoints + 1; jointIndex++)
         {
            InverseDynamicsJoint joint = joints.get(jointIndex);
            RigidBody body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            ReferenceFrame bodyFrameInFuture = jointsInFuture.get(jointIndex).getSuccessor().getBodyFixedFrame();
            Twist expectedTwist = computeExpectedTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);

            assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);

            // Assert relative twist
            for (int baseJointIndex = 0; baseJointIndex < numberOfRevoluteJoints + 1; baseJointIndex++)
            {
               RigidBody base = joints.get(baseJointIndex).getSuccessor();
               Twist actualRelativeTwist = new Twist();
               twistCalculator.getRelativeTwist(base, body, actualRelativeTwist);

               ReferenceFrame baseFrame = base.getBodyFixedFrame();
               ReferenceFrame baseFrameInFuture = jointsInFuture.get(baseJointIndex).getSuccessor().getBodyFixedFrame();
               Twist expectedRelativeTwist = computeExpectedRelativeTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture, baseFrame, baseFrameInFuture);

               assertTwistEquals(expectedRelativeTwist, actualRelativeTwist, 1.0e-5);

               Point3D bodyFixedPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               FramePoint3D frameBodyFixedPoint = new FramePoint3D(bodyFrame, bodyFixedPoint);
               FrameVector3D actualLinearVelocity = new FrameVector3D();
               twistCalculator.getLinearVelocityOfBodyFixedPoint(base, body, frameBodyFixedPoint, actualLinearVelocity);
               FrameVector3D expectedLinearVelocity = computeExpectedLinearVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture, baseFrame,
                                                                                                    baseFrameInFuture, bodyFixedPoint);

               expectedLinearVelocity.checkReferenceFrameMatch(actualLinearVelocity);
               EuclidCoreTestTools.assertTuple3DEquals(expectedLinearVelocity.getVector(), actualLinearVelocity.getVector(), 2.0e-5);

               FrameVector3D expectedAngularVelocity = new FrameVector3D();
               expectedRelativeTwist.getAngularPart(expectedAngularVelocity);
               FrameVector3D actualAngularVelocity = new FrameVector3D();
               twistCalculator.getRelativeAngularVelocity(base, body, actualAngularVelocity);

               expectedAngularVelocity.checkReferenceFrameMatch(actualAngularVelocity);
               EuclidCoreTestTools.assertTuple3DEquals(expectedAngularVelocity.getVector(), actualAngularVelocity.getVector(), 1.0e-5);
            }
         }
      }
   }

   public static void assertTwistEquals(Twist expectedTwist, Twist actualTwist, double epsilon) throws AssertionError
   {
      assertTwistEquals(null, expectedTwist, actualTwist, epsilon);
   }

   public static void assertTwistEquals(String messagePrefix, Twist expectedTwist, Twist actualTwist, double epsilon) throws AssertionError
   {
      try
      {
         assertTrue(expectedTwist.epsilonEquals(actualTwist, epsilon));
      }
      catch (AssertionError e)
      {
         Vector3D difference = new Vector3D();
         difference.sub(expectedTwist.getLinearPart(), actualTwist.getLinearPart());
         double linearPartDifference = difference.length();
         difference.sub(expectedTwist.getAngularPart(), actualTwist.getAngularPart());
         double angularPartDifference = difference.length();
         messagePrefix = messagePrefix != null ? messagePrefix + " " : "";
         throw new AssertionError(messagePrefix + "expected:\n<" + expectedTwist + ">\n but was:\n<" + actualTwist + ">\n difference: linear part: " + linearPartDifference
               + ", angular part: " + angularPartDifference);
      }
   }

   public static FrameVector3D computeExpectedLinearVelocityByFiniteDifference(double dt, ReferenceFrame bodyFrame, ReferenceFrame bodyFrameInFuture,
                                                                             Point3D bodyFixedPoint)
   {
      return computeExpectedLinearVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture, worldFrame, worldFrame, bodyFixedPoint);
   }

   public static FrameVector3D computeExpectedLinearVelocityByFiniteDifference(double dt, ReferenceFrame bodyFrame, ReferenceFrame bodyFrameInFuture,
                                                                             ReferenceFrame baseFrame, ReferenceFrame baseFrameInFuture, Point3D bodyFixedPoint)
   {
      FramePoint3D point = new FramePoint3D(bodyFrame, bodyFixedPoint);
      FramePoint3D pointInFuture = new FramePoint3D(bodyFrameInFuture, bodyFixedPoint);
      point.changeFrame(baseFrame);
      pointInFuture.changeFrame(baseFrameInFuture);

      FrameVector3D pointLinearVelocity = new FrameVector3D(baseFrame);
      pointLinearVelocity.sub(pointInFuture.getPoint(), point.getPoint());
      pointLinearVelocity.scale(1.0 / dt);
      return pointLinearVelocity;
   }

   public static Twist computeExpectedTwistByFiniteDifference(double dt, ReferenceFrame bodyFrame, ReferenceFrame bodyFrameInFuture)
   {
      Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

      FrameVector3D bodyLinearVelocity = computeLinearVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);
      expectedTwist.setLinearPart(bodyLinearVelocity);

      FrameVector3D bodyAngularVelocity = computeAngularVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);
      expectedTwist.setAngularPart(bodyAngularVelocity);
      return expectedTwist;
   }

   public static Twist computeExpectedRelativeTwistByFiniteDifference(double dt, ReferenceFrame bodyFrame, ReferenceFrame bodyFrameInFuture,
                                                                      ReferenceFrame baseFrame, ReferenceFrame baseFrameInFuture)
   {
      Twist bodyTwist = computeExpectedTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);
      bodyTwist.changeFrame(bodyFrame);
      Twist baseTwist = computeExpectedTwistByFiniteDifference(dt, baseFrame, baseFrameInFuture);
      baseTwist.changeFrame(bodyFrame);

      Twist relativeTwist = new Twist(bodyFrame, baseFrame, bodyFrame);
      relativeTwist.set(bodyTwist);
      relativeTwist.sub(baseTwist);
      return relativeTwist;
   }

   public static FrameVector3D computeAngularVelocityByFiniteDifference(double dt, ReferenceFrame bodyFrame, ReferenceFrame bodyFrameInFuture)
   {
      FrameQuaternion bodyOrientation = new FrameQuaternion(bodyFrame);
      bodyOrientation.changeFrame(worldFrame);
      FrameQuaternion bodyOrientationInFuture = new FrameQuaternion(bodyFrameInFuture);
      bodyOrientationInFuture.changeFrame(worldFrame);

      FrameVector3D bodyAngularVelocity = new FrameVector3D(worldFrame);
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      Vector4D qDot = new Vector4D();
      quaternionCalculus.computeQDotByFiniteDifferenceCentral(bodyOrientation.getQuaternion(), bodyOrientationInFuture.getQuaternion(), 0.5 * dt, qDot);
      quaternionCalculus.computeAngularVelocityInWorldFrame(bodyOrientation.getQuaternion(), qDot, bodyAngularVelocity.getVector());

      bodyAngularVelocity.changeFrame(bodyFrame);
      return bodyAngularVelocity;
   }

   public static FrameVector3D computeLinearVelocityByFiniteDifference(double dt, ReferenceFrame bodyFrame, ReferenceFrame bodyFrameInFuture)
   {
      FramePoint3D bodyPosition = new FramePoint3D(bodyFrame);
      bodyPosition.changeFrame(worldFrame);
      FramePoint3D bodyPositionInFuture = new FramePoint3D(bodyFrameInFuture);
      bodyPositionInFuture.changeFrame(worldFrame);

      FrameVector3D bodyLinearVelocity = new FrameVector3D(worldFrame);
      bodyLinearVelocity.sub(bodyPositionInFuture, bodyPosition);
      bodyLinearVelocity.scale(1.0 / dt);
      bodyLinearVelocity.changeFrame(bodyFrame);
      return bodyLinearVelocity;
   }

   public static void main(String[] args)
   {
      Random random = new Random();
      int numberOfJoints = 5;
      List<RevoluteJoint> randomChainRobot = ScrewTestTools.createRandomChainRobot(numberOfJoints, random);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, randomChainRobot.get(0).getPredecessor());

      Twist dummyTwist = new Twist();

      while (true)
      {
         twistCalculator.compute();

         for (int i = 0; i < 100; i++)
            twistCalculator.getTwistOfBody(randomChainRobot.get(random.nextInt(numberOfJoints)).getSuccessor(), dummyTwist);
      }
   }
}
