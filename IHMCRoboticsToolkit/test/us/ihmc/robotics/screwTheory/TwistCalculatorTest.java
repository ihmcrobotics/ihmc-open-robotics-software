package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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

         FrameVector cumulatedLinearVelocity = new FrameVector(worldFrame);

         for (PrismaticJoint joint : prismaticJoints)
         {
            RigidBody body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(actualTwist, body);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

            FrameVector jointAxis = joint.getJointAxis();
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

         FrameVector cumulatedAngularVelocity = new FrameVector(worldFrame);

         for (RevoluteJoint joint : revoluteJoints)
         {
            RigidBody body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(actualTwist, body);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

            FrameVector jointAxis = joint.getJointAxis();
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
      // FIXME I think that is a bug, not a bad one but still
      //      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, prismaticJoints.get(random.nextInt(numberOfJoints)).getPredecessor());
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, prismaticJoints.get(0).getPredecessor());

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(prismaticJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomVelocities(prismaticJoints, random, -10.0, 10.0);
         twistCalculator.compute();

         for (PrismaticJoint joint : prismaticJoints)
         {
            RigidBody body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(actualTwist, body);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

            RigidBody currentBody = body;
            FrameVector cumulatedLinearVelocity = new FrameVector(worldFrame);

            while (currentBody.getParentJoint() != null)
            {
               PrismaticJoint parentJoint = (PrismaticJoint) currentBody.getParentJoint();
               FrameVector jointAxis = parentJoint.getJointAxis();
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
      // FIXME I think that is a bug, not a bad one but still
      //      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, prismaticJoints.get(random.nextInt(numberOfJoints)).getPredecessor());
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, revoluteJoints.get(0).getPredecessor());

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(revoluteJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomVelocities(revoluteJoints, random, -10.0, 10.0);
         twistCalculator.compute();

         for (RevoluteJoint joint : revoluteJoints)
         {
            RigidBody body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(actualTwist, body);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

            RigidBody currentBody = body;
            FrameVector cumulatedAngularVelocity = new FrameVector(worldFrame);

            while (currentBody.getParentJoint() != null)
            {
               RevoluteJoint parentJoint = (RevoluteJoint) currentBody.getParentJoint();
               FrameVector jointAxis = parentJoint.getJointAxis();
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
            twistCalculator.getTwistOfBody(actualTwist, body);

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
            twistCalculator.getTwistOfBody(actualTwist, body);

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
      List<InverseDynamicsJoint> jointsInFuture = Arrays.asList(ScrewTools.cloneJointPath(joints.toArray(new InverseDynamicsJoint[numberOfRevoluteJoints + 1])));
      SixDoFJoint floatingJointInFuture = (SixDoFJoint) jointsInFuture.get(0);
      List<RevoluteJoint> revoluteJointsInFuture = ScrewTools.filterJoints(jointsInFuture, RevoluteJoint.class);

      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, joints.get(0).getPredecessor());

      double dt = 1.0e-8;

      for (int i = 0; i < 100; i++)
      {
         floatingJoint.setRotation(RandomTools.generateRandomQuaternion(random));
         floatingJoint.setPosition(RandomTools.generateRandomPoint3d(random, -10.0, 10.0));
         Twist floatingJointTwist = Twist.generateRandomTwist(random, floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(), floatingJoint.getFrameAfterJoint());
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
            twistCalculator.getTwistOfBody(actualTwist, body);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            ReferenceFrame bodyFrameInFuture = jointsInFuture.get(jointIndex).getSuccessor().getBodyFixedFrame();
            Twist expectedTwist = computeExpectedTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);

            assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);
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
      List<InverseDynamicsJoint> jointsInFuture = Arrays.asList(ScrewTools.cloneJointPath(joints.toArray(new InverseDynamicsJoint[numberOfRevoluteJoints + 1])));
      SixDoFJoint floatingJointInFuture = (SixDoFJoint) jointsInFuture.get(0);
      List<RevoluteJoint> revoluteJointsInFuture = ScrewTools.filterJoints(jointsInFuture, RevoluteJoint.class);

      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, joints.get(0).getPredecessor());

      double dt = 1.0e-8;

      for (int i = 0; i < 50; i++)
      {
         floatingJoint.setRotation(RandomTools.generateRandomQuaternion(random));
         floatingJoint.setPosition(RandomTools.generateRandomPoint3d(random, -10.0, 10.0));
         Twist floatingJointTwist = Twist.generateRandomTwist(random, floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(), floatingJoint.getFrameAfterJoint());
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
            twistCalculator.getTwistOfBody(actualTwist, body);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            ReferenceFrame bodyFrameInFuture = jointsInFuture.get(jointIndex).getSuccessor().getBodyFixedFrame();
            Twist expectedTwist = computeExpectedTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);

            assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);

            // Assert relative twist
            for (int baseJointIndex = 0; baseJointIndex < numberOfRevoluteJoints + 1; baseJointIndex++)
            {
               RigidBody base = joints.get(baseJointIndex).getSuccessor();
               Twist actualRelativeTwist = new Twist();
               twistCalculator.getRelativeTwist(actualRelativeTwist, base, body);

               ReferenceFrame baseFrame = base.getBodyFixedFrame();
               ReferenceFrame baseFrameInFuture = jointsInFuture.get(baseJointIndex).getSuccessor().getBodyFixedFrame();
               Twist expectedRelativeTwist = computeExpectedRelativeTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture, baseFrame, baseFrameInFuture);

               assertTwistEquals(expectedRelativeTwist, actualRelativeTwist, 1.0e-5);
            }
         }
      }
   }

   public static void assertTwistEquals(Twist expectedTwist, Twist actualTwist, double epsilon) throws AssertionError
   {
      try
      {
         assertTrue(expectedTwist.epsilonEquals(actualTwist, epsilon));
      }
      catch (AssertionError e)
      {
         throw new AssertionError("expected:\n<" + expectedTwist + ">\n but was:\n<" + actualTwist + ">");
      }
   }

   private Twist computeExpectedTwistByFiniteDifference(double dt, ReferenceFrame bodyFrame, ReferenceFrame bodyFrameInFuture)
   {
      Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

      FrameVector bodyLinearVelocity = computeLinearVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);
      expectedTwist.setLinearPart(bodyLinearVelocity);

      FrameVector bodyAngularVelocity = computeAngularVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);
      expectedTwist.setAngularPart(bodyAngularVelocity);
      return expectedTwist;
   }

   private Twist computeExpectedRelativeTwistByFiniteDifference(double dt, ReferenceFrame bodyFrame, ReferenceFrame bodyFrameInFuture, ReferenceFrame baseFrame, ReferenceFrame baseFrameInFuture)
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

   private FrameVector computeAngularVelocityByFiniteDifference(double dt, ReferenceFrame bodyFrame, ReferenceFrame bodyFrameInFuture)
   {
      FrameOrientation bodyOrientation = new FrameOrientation(bodyFrame);
      bodyOrientation.changeFrame(worldFrame);
      FrameOrientation bodyOrientationInFuture = new FrameOrientation(bodyFrameInFuture);
      bodyOrientationInFuture.changeFrame(worldFrame);

      FrameVector bodyAngularVelocity = new FrameVector(worldFrame);
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      Vector4D qDot = new Vector4D();
      quaternionCalculus.computeQDotByFiniteDifferenceCentral(bodyOrientation.getQuaternion(), bodyOrientationInFuture.getQuaternion(), 0.5 * dt, qDot);
      quaternionCalculus.computeAngularVelocityInWorldFrame(bodyOrientation.getQuaternion(), qDot, bodyAngularVelocity.getVector());

      bodyAngularVelocity.changeFrame(bodyFrame);
      return bodyAngularVelocity;
   }

   private FrameVector computeLinearVelocityByFiniteDifference(double dt, ReferenceFrame bodyFrame, ReferenceFrame bodyFrameInFuture)
   {
      FramePoint bodyPosition = new FramePoint(bodyFrame);
      bodyPosition.changeFrame(worldFrame);
      FramePoint bodyPositionInFuture = new FramePoint(bodyFrameInFuture);
      bodyPositionInFuture.changeFrame(worldFrame);
      
      FrameVector bodyLinearVelocity = new FrameVector(worldFrame);
      bodyLinearVelocity.subAndScale(1.0 / dt, bodyPositionInFuture, bodyPosition);
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
            twistCalculator.getTwistOfBody(dummyTwist, randomChainRobot.get(random.nextInt(numberOfJoints)).getSuccessor());
      }
   }
}
