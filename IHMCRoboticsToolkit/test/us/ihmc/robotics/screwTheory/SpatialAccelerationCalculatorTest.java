package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.*;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;

public class SpatialAccelerationCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 1000;

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testWithChainComposedOfPrismaticJoints() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 20;
      List<PrismaticJoint> prismaticJoints = ScrewTestTools.createRandomChainRobotWithPrismaticJoints(numberOfJoints, random);
      RigidBody randomBody = prismaticJoints.get(random.nextInt(numberOfJoints)).getPredecessor();
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, randomBody);
      RigidBody rootBody = ScrewTools.getRootBody(randomBody);
      boolean doAccelerationTerms = true;

      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean doVelocityTerms = random.nextBoolean();

         FrameVector rootLinearAcceleration = new FrameVector(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.generateRandomVector3D(random, -10.0, 10.0));
         FrameVector rootAngularAcceleration = new FrameVector(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.generateRandomVector3D(random, -10.0, 10.0));
         SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.setLinearPart(rootLinearAcceleration);
         rootAcceleration.setAngularPart(rootAngularAcceleration);
         boolean useDesireds = random.nextBoolean();
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(randomBody, worldFrame, rootAcceleration,
                                                                                                         twistCalculator, doVelocityTerms, doAccelerationTerms,
                                                                                                         useDesireds);

         ScrewTestTools.setRandomPositions(prismaticJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomVelocities(prismaticJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomAccelerations(prismaticJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomDesiredAccelerations(prismaticJoints, random, -10.0, 10.0);

         twistCalculator.compute();
         spatialAccelerationCalculator.compute();

         FrameVector cumulatedLinearAcceleration = new FrameVector(rootLinearAcceleration);

         for (PrismaticJoint joint : prismaticJoints)
         {
            RigidBody body = joint.getSuccessor();
            SpatialAccelerationVector actualAcceleration = new SpatialAccelerationVector();
            spatialAccelerationCalculator.getAccelerationOfBody(actualAcceleration, body);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            SpatialAccelerationVector expectedAcceleration = new SpatialAccelerationVector(bodyFrame, worldFrame, bodyFrame);

            FrameVector jointAxis = joint.getJointAxis();
            cumulatedLinearAcceleration.changeFrame(jointAxis.getReferenceFrame());
            double qdd = useDesireds ? joint.getQddDesired() : joint.getQdd();
            cumulatedLinearAcceleration.scaleAdd(qdd, jointAxis, cumulatedLinearAcceleration);
            cumulatedLinearAcceleration.changeFrame(bodyFrame);
            expectedAcceleration.setLinearPart(cumulatedLinearAcceleration);

            // Need to compute the cross part due to the root angular acceleration
            FramePoint rootPosition = new FramePoint(rootBody.getBodyFixedFrame());
            rootPosition.changeFrame(bodyFrame);
            rootAngularAcceleration.changeFrame(bodyFrame);
            Vector3D crossPart = new Vector3D();
            crossPart.cross(rootPosition.getVectorCopy(), rootAngularAcceleration.getVector());
            expectedAcceleration.linearPart.add(crossPart);

            expectedAcceleration.setAngularPart(rootAngularAcceleration);

            assertSpatialAccelerationVectorEquals(expectedAcceleration, actualAcceleration, 1.0e-12);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testWithChainComposedOfRevoluteJointsAssertAngularAccelerationOnly() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 20;
      List<RevoluteJoint> revoluteJoints = ScrewTestTools.createRandomChainRobot(numberOfJoints, random);
      RigidBody randomBody = revoluteJoints.get(random.nextInt(numberOfJoints)).getPredecessor();
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, randomBody);
      RigidBody rootBody = ScrewTools.getRootBody(randomBody);
      boolean doAccelerationTerms = true;

      // No velocity
      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean doVelocityTerms = random.nextBoolean();

         FrameVector rootLinearAcceleration = new FrameVector(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.generateRandomVector3D(random, -10.0, 10.0));
         FrameVector rootAngularAcceleration = new FrameVector(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.generateRandomVector3D(random, -10.0, 10.0));
         SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.setLinearPart(rootLinearAcceleration);
         rootAcceleration.setAngularPart(rootAngularAcceleration);
         boolean useDesireds = random.nextBoolean();
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(randomBody, worldFrame, rootAcceleration,
                                                                                                         twistCalculator, doVelocityTerms, doAccelerationTerms,
                                                                                                         useDesireds);

         ScrewTestTools.setRandomPositions(revoluteJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomVelocities(revoluteJoints, random, 0.0, 0.0);
         ScrewTestTools.setRandomAccelerations(revoluteJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomDesiredAccelerations(revoluteJoints, random, -10.0, 10.0);

         twistCalculator.compute();
         spatialAccelerationCalculator.compute();

         FrameVector cumulatedAngularAcceleration = new FrameVector(rootAngularAcceleration);

         for (RevoluteJoint joint : revoluteJoints)
         {
            RigidBody body = joint.getSuccessor();
            SpatialAccelerationVector actualAcceleration = new SpatialAccelerationVector();
            spatialAccelerationCalculator.getAccelerationOfBody(actualAcceleration, body);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            SpatialAccelerationVector expectedAcceleration = new SpatialAccelerationVector(bodyFrame, worldFrame, bodyFrame);

            FrameVector jointAxis = joint.getJointAxis();
            cumulatedAngularAcceleration.changeFrame(jointAxis.getReferenceFrame());
            double qdd = useDesireds ? joint.getQddDesired() : joint.getQdd();
            cumulatedAngularAcceleration.scaleAdd(qdd, jointAxis, cumulatedAngularAcceleration);
            cumulatedAngularAcceleration.changeFrame(bodyFrame);
            expectedAcceleration.setAngularPart(cumulatedAngularAcceleration);

            expectedAcceleration.checkReferenceFramesMatch(actualAcceleration);

            EuclidCoreTestTools.assertTuple3DEquals(expectedAcceleration.getAngularPart(), actualAcceleration.getAngularPart(), 1.0e-12);
         }
      }

      // Non-zero velocities
      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean doVelocityTerms = random.nextBoolean();

         FrameVector rootLinearAcceleration = new FrameVector(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.generateRandomVector3D(random, -10.0, 10.0));
         FrameVector rootAngularAcceleration = new FrameVector(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.generateRandomVector3D(random, -10.0, 10.0));
         SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.setLinearPart(rootLinearAcceleration);
         rootAcceleration.setAngularPart(rootAngularAcceleration);
         boolean useDesireds = random.nextBoolean();
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(randomBody, worldFrame, rootAcceleration,
                                                                                                         twistCalculator, doVelocityTerms, doAccelerationTerms,
                                                                                                         useDesireds);

         ScrewTestTools.setRandomPositions(revoluteJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomVelocities(revoluteJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomAccelerations(revoluteJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomDesiredAccelerations(revoluteJoints, random, -10.0, 10.0);

         twistCalculator.compute();
         spatialAccelerationCalculator.compute();

         FrameVector cumulatedAngularAcceleration = new FrameVector(rootAngularAcceleration);

         for (int j = 0; j < revoluteJoints.size(); j++)
         {
            RevoluteJoint joint = revoluteJoints.get(j);
            RigidBody body = joint.getSuccessor();
            SpatialAccelerationVector actualAcceleration = new SpatialAccelerationVector();
            spatialAccelerationCalculator.getAccelerationOfBody(actualAcceleration, body);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            SpatialAccelerationVector expectedAcceleration = new SpatialAccelerationVector(bodyFrame, worldFrame, bodyFrame);

            FrameVector jointAxis = new FrameVector(joint.getJointAxis());
            cumulatedAngularAcceleration.changeFrame(jointAxis.getReferenceFrame());
            double qdd = useDesireds ? joint.getQddDesired() : joint.getQdd();
            cumulatedAngularAcceleration.scaleAdd(qdd, jointAxis, cumulatedAngularAcceleration);
            cumulatedAngularAcceleration.changeFrame(bodyFrame);

            if (doVelocityTerms)
            {
               // Need to account for the Coriolis acceleration
               Twist bodyTwist = new Twist();
               twistCalculator.getTwistOfBody(bodyTwist, body);
               bodyTwist.changeFrame(bodyFrame);
               FrameVector coriolis = new FrameVector(bodyFrame);
               jointAxis.changeFrame(bodyFrame);
               coriolis.cross(bodyTwist.getAngularPart(), jointAxis.getVector());
               coriolis.scale(joint.getQd());
               cumulatedAngularAcceleration.add(coriolis);
            }

            expectedAcceleration.setAngularPart(cumulatedAngularAcceleration);
            expectedAcceleration.checkReferenceFramesMatch(bodyFrame, worldFrame, bodyFrame);

            EuclidCoreTestTools.assertTuple3DEquals(expectedAcceleration.getAngularPart(), actualAcceleration.getAngularPart(), 1.0e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testWithChainRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(234234L);

      int numberOfJoints = 10;
      List<OneDoFJoint> joints = ScrewTestTools.createRandomChainRobotWithOneDoFJoints(numberOfJoints, random);
      List<OneDoFJoint> jointsInFuture = Arrays.asList(ScrewTools.cloneOneDoFJointPath(joints.toArray(new OneDoFJoint[numberOfJoints])));

      RigidBody randomBody = joints.get(random.nextInt(joints.size())).getPredecessor();
      RigidBody rootBody = ScrewTools.getRootBody(randomBody);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, randomBody);
      TwistCalculator twistCalculatorInFuture = new TwistCalculator(worldFrame, jointsInFuture.get(random.nextInt(joints.size())).getPredecessor());

      double dt = 1.0e-8;

      for (int i = 0; i < ITERATIONS; i++)
      {
         SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.setLinearPart(EuclidCoreRandomTools.generateRandomVector3D(random));
         rootAcceleration.setAngularPart(EuclidCoreRandomTools.generateRandomVector3D(random));
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(randomBody, worldFrame, rootAcceleration,
                                                                                                         twistCalculator, true, true, false);

         ScrewTestTools.setRandomPositions(joints, random, -1.0, 1.0);
         ScrewTestTools.setRandomVelocities(joints, random, -1.0, 1.0);
         ScrewTestTools.setRandomAccelerations(joints, random, -1.0, 1.0);

         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            double q = joints.get(jointIndex).getQ();
            double qd = joints.get(jointIndex).getQd();
            double qdd = joints.get(jointIndex).getQdd();

            double qdFuture = qd + dt * qdd;
            double qFuture = q + dt * qd + 0.5 * dt * dt * qdd;
            jointsInFuture.get(jointIndex).setQ(qFuture);
            jointsInFuture.get(jointIndex).setQd(qdFuture);
         }

         joints.get(0).updateFramesRecursively();
         jointsInFuture.get(0).updateFramesRecursively();

         twistCalculator.compute();
         twistCalculatorInFuture.compute();
         spatialAccelerationCalculator.compute();

         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            OneDoFJoint joint = joints.get(jointIndex);
            RigidBody body = joint.getSuccessor();
            RigidBody bodyInFuture = jointsInFuture.get(jointIndex).getSuccessor();

            SpatialAccelerationVector actualAcceleration = new SpatialAccelerationVector();
            spatialAccelerationCalculator.getAccelerationOfBody(actualAcceleration, body);

            SpatialAccelerationVector expectedAcceleration = computeExpectedAccelerationByFiniteDifference(dt, body, bodyInFuture, twistCalculator,
                                                                                                           twistCalculatorInFuture, rootAcceleration);

            assertSpatialAccelerationVectorEquals(expectedAcceleration, actualAcceleration, 1.0e-5);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
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

      RigidBody randomBody = joints.get(0).getPredecessor();
      RigidBody randomBodyInFuture = jointsInFuture.get(0).getPredecessor();
      RigidBody rootBody = ScrewTools.getRootBody(randomBody);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, randomBody);
      TwistCalculator twistCalculatorInFuture = new TwistCalculator(worldFrame, randomBodyInFuture);

      double dt = 1.0e-8;

      for (int i = 0; i < ITERATIONS; i++)
      {
         SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.setLinearPart(EuclidCoreRandomTools.generateRandomVector3D(random));
         rootAcceleration.setAngularPart(EuclidCoreRandomTools.generateRandomVector3D(random));
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(randomBody, worldFrame, rootAcceleration,
                                                                                                         twistCalculator, true, true, false);

         floatingJoint.setRotation(RandomGeometry.nextQuaternion(random));
         floatingJoint.setPosition(RandomGeometry.nextPoint3D(random, -10.0, 10.0));
         Twist floatingJointTwist = Twist.generateRandomTwist(random, floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(),
                                                              floatingJoint.getFrameAfterJoint());
         floatingJoint.setJointTwist(floatingJointTwist);
         ScrewTestTools.setRandomAcceleration(floatingJoint, random);

         floatingJointInFuture.setJointPositionVelocityAndAcceleration(floatingJoint);
         ScrewTestTools.doubleIntegrateAccelerations(floatingJointInFuture, dt);

         ScrewTestTools.setRandomPositions(revoluteJoints, random, -1.0, 1.0);
         ScrewTestTools.setRandomVelocities(revoluteJoints, random, -1.0, 1.0);
         ScrewTestTools.setRandomAccelerations(revoluteJoints, random, -1.0, 1.0);

         for (int jointIndex = 0; jointIndex < numberOfRevoluteJoints; jointIndex++)
         {
            double q = revoluteJoints.get(jointIndex).getQ();
            double qd = revoluteJoints.get(jointIndex).getQd();
            double qdd = revoluteJoints.get(jointIndex).getQdd();

            double qdFuture = qd + dt * qdd;
            double qFuture = q + dt * qd + 0.5 * dt * dt * qdd;
            revoluteJointsInFuture.get(jointIndex).setQ(qFuture);
            revoluteJointsInFuture.get(jointIndex).setQd(qdFuture);
         }

         floatingJoint.updateFramesRecursively();
         floatingJointInFuture.updateFramesRecursively();

         twistCalculator.compute();
         twistCalculatorInFuture.compute();
         spatialAccelerationCalculator.compute();

         for (int jointIndex = 0; jointIndex < numberOfRevoluteJoints + 1; jointIndex++)
         {
            InverseDynamicsJoint joint = joints.get(jointIndex);
            RigidBody body = joint.getSuccessor();
            RigidBody bodyInFuture = jointsInFuture.get(jointIndex).getSuccessor();
            SpatialAccelerationVector actualAcceleration = new SpatialAccelerationVector();
            spatialAccelerationCalculator.getAccelerationOfBody(actualAcceleration, body);

            SpatialAccelerationVector expectedAcceleration = computeExpectedAccelerationByFiniteDifference(dt, body, bodyInFuture, twistCalculator,
                                                                                                           twistCalculatorInFuture, rootAcceleration);

            assertSpatialAccelerationVectorEquals(expectedAcceleration, actualAcceleration, 1.0e-4);
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

      RigidBody randomBody = joints.get(random.nextInt(numberOfRevoluteJoints)).getPredecessor();
      RigidBody randomBodyInFuture = jointsInFuture.get(random.nextInt(numberOfRevoluteJoints)).getPredecessor();
      RigidBody rootBody = ScrewTools.getRootBody(randomBody);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, randomBody);
      TwistCalculator twistCalculatorInFuture = new TwistCalculator(worldFrame, randomBodyInFuture);

      double dt = 1.0e-8;

      for (int i = 0; i < 50; i++)
      {
         SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.setLinearPart(EuclidCoreRandomTools.generateRandomVector3D(random));
         rootAcceleration.setAngularPart(EuclidCoreRandomTools.generateRandomVector3D(random));
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(randomBody, worldFrame, rootAcceleration,
                                                                                                         twistCalculator, true, true, false);

         floatingJoint.setRotation(RandomGeometry.nextQuaternion(random));
         floatingJoint.setPosition(RandomGeometry.nextPoint3D(random, -10.0, 10.0));
         Twist floatingJointTwist = Twist.generateRandomTwist(random, floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(),
                                                              floatingJoint.getFrameAfterJoint());
         floatingJoint.setJointTwist(floatingJointTwist);
         ScrewTestTools.setRandomAcceleration(floatingJoint, random);

         floatingJointInFuture.setJointPositionVelocityAndAcceleration(floatingJoint);
         ScrewTestTools.doubleIntegrateAccelerations(floatingJointInFuture, dt);

         ScrewTestTools.setRandomPositions(revoluteJoints, random, -1.0, 1.0);
         ScrewTestTools.setRandomVelocities(revoluteJoints, random, -1.0, 1.0);
         ScrewTestTools.setRandomAccelerations(revoluteJoints, random, -1.0, 1.0);

         for (int jointIndex = 0; jointIndex < numberOfRevoluteJoints; jointIndex++)
         {
            double q = revoluteJoints.get(jointIndex).getQ();
            double qd = revoluteJoints.get(jointIndex).getQd();
            double qdd = revoluteJoints.get(jointIndex).getQdd();

            double qdFuture = qd + dt * qdd;
            double qFuture = q + dt * qd + 0.5 * dt * dt * qdd;
            revoluteJointsInFuture.get(jointIndex).setQ(qFuture);
            revoluteJointsInFuture.get(jointIndex).setQd(qdFuture);
         }

         floatingJoint.updateFramesRecursively();
         floatingJointInFuture.updateFramesRecursively();

         twistCalculator.compute();
         twistCalculatorInFuture.compute();
         spatialAccelerationCalculator.compute();

         for (int jointIndex = 0; jointIndex < numberOfRevoluteJoints + 1; jointIndex++)
         {
            InverseDynamicsJoint joint = joints.get(jointIndex);
            RigidBody body = joint.getSuccessor();
            RigidBody bodyInFuture = jointsInFuture.get(jointIndex).getSuccessor();
            SpatialAccelerationVector actualAcceleration = new SpatialAccelerationVector();
            spatialAccelerationCalculator.getAccelerationOfBody(actualAcceleration, body);

            SpatialAccelerationVector expectedAcceleration = computeExpectedAccelerationByFiniteDifference(dt, body, bodyInFuture, twistCalculator,
                                                                                                           twistCalculatorInFuture, rootAcceleration);

            assertSpatialAccelerationVectorEquals(expectedAcceleration, actualAcceleration, 1.0e-4);

            // Assert relative twist
            for (int baseJointIndex = 0; baseJointIndex < numberOfRevoluteJoints + 1; baseJointIndex++)
            {
               RigidBody base = joints.get(baseJointIndex).getSuccessor();
               RigidBody baseInFuture = jointsInFuture.get(baseJointIndex).getSuccessor();
               SpatialAccelerationVector actualRelativeAcceleration = new SpatialAccelerationVector();
               spatialAccelerationCalculator.getRelativeAcceleration(actualRelativeAcceleration, base, body);

               SpatialAccelerationVector expectedRelativeAcceleration = computeExpectedRelativeAccelerationByFiniteDifference(dt, body, bodyInFuture, base,
                                                                                                                              baseInFuture, twistCalculator,
                                                                                                                              twistCalculatorInFuture,
                                                                                                                              rootAcceleration);

               assertSpatialAccelerationVectorEquals(expectedRelativeAcceleration, actualRelativeAcceleration, 1.0e-4);
            }
         }
      }
   }

   private SpatialAccelerationVector computeExpectedRelativeAccelerationByFiniteDifference(double dt, RigidBody body, RigidBody bodyInFuture, RigidBody base,
                                                                                           RigidBody baseInFuture, TwistCalculator twistCalculator,
                                                                                           TwistCalculator twistCalculatorInFuture,
                                                                                           SpatialAccelerationVector rootAcceleration)
   {
      SpatialAccelerationVector expectedAcceleration = new SpatialAccelerationVector(body.getBodyFixedFrame(), base.getBodyFixedFrame(),
                                                                                     body.getBodyFixedFrame());

      Twist relativeTwist = new Twist();
      Twist relativeTwistInFuture = new Twist();

      twistCalculator.getRelativeTwist(relativeTwist, base, body);
      twistCalculatorInFuture.getRelativeTwist(relativeTwistInFuture, baseInFuture, bodyInFuture);

      Vector3D angularAcceleration = firstOrderFiniteDifference(dt, relativeTwist.getAngularPart(), relativeTwistInFuture.getAngularPart());
      Vector3D linearAcceleration = firstOrderFiniteDifference(dt, relativeTwist.getLinearPart(), relativeTwistInFuture.getLinearPart());
      expectedAcceleration.setAngularPart(angularAcceleration);
      expectedAcceleration.setLinearPart(linearAcceleration);

      return expectedAcceleration;
   }

   private static SpatialAccelerationVector computeExpectedAccelerationByFiniteDifference(double dt, RigidBody body, RigidBody bodyInFuture,
                                                                                          TwistCalculator twistCalculator,
                                                                                          TwistCalculator twistCalculatorInFuture,
                                                                                          SpatialAccelerationVector rootAcceleration)
   {
      SpatialAccelerationVector expectedAcceleration = new SpatialAccelerationVector(body.getBodyFixedFrame(), worldFrame, body.getBodyFixedFrame());

      Twist bodyTwist = new Twist();
      Twist bodyTwistInFuture = new Twist();

      twistCalculator.getTwistOfBody(bodyTwist, body);
      twistCalculatorInFuture.getTwistOfBody(bodyTwistInFuture, bodyInFuture);

      Vector3D angularAcceleration = firstOrderFiniteDifference(dt, bodyTwist.getAngularPart(), bodyTwistInFuture.getAngularPart());
      Vector3D linearAcceleration = firstOrderFiniteDifference(dt, bodyTwist.getLinearPart(), bodyTwistInFuture.getLinearPart());
      expectedAcceleration.setAngularPart(angularAcceleration);
      expectedAcceleration.setLinearPart(linearAcceleration);

      // Need to account for the root acceleration
      FrameVector rootAngularAcceleration = new FrameVector(rootAcceleration.getExpressedInFrame(), rootAcceleration.getAngularPart());
      FrameVector rootLinearAcceleration = new FrameVector(rootAcceleration.getExpressedInFrame(), rootAcceleration.getLinearPart());
      rootAngularAcceleration.changeFrame(expectedAcceleration.getExpressedInFrame());
      rootLinearAcceleration.changeFrame(expectedAcceleration.getExpressedInFrame());
      expectedAcceleration.angularPart.add(rootAngularAcceleration.getVector());

      FramePoint rootToBody = new FramePoint(rootAcceleration.getExpressedInFrame());
      rootToBody.changeFrame(expectedAcceleration.getExpressedInFrame());
      Vector3D crossPart = new Vector3D();
      crossPart.cross(rootToBody.getVectorCopy(), rootAngularAcceleration.getVector());
      expectedAcceleration.linearPart.add(crossPart);
      expectedAcceleration.linearPart.add(rootLinearAcceleration.getVector());

      return expectedAcceleration;
   }

   public static Vector3D firstOrderFiniteDifference(double dt, Vector3DReadOnly now, Vector3DReadOnly next)
   {
      Vector3D finiteDifference = new Vector3D();
      finiteDifference.sub(next, now);
      finiteDifference.scale(1.0 / dt);
      return finiteDifference;
   }

   public static void assertSpatialAccelerationVectorEquals(SpatialAccelerationVector expected, SpatialAccelerationVector actual, double epsilon)
         throws AssertionError
   {
      try
      {
         assertTrue(expected.epsilonEquals(actual, epsilon));
      }
      catch (AssertionError e)
      {
         Vector3D difference = new Vector3D();
         difference.sub(expected.getLinearPart(), actual.getLinearPart());
         double linearPartDifference = difference.length();
         difference.sub(expected.getAngularPart(), actual.getAngularPart());
         double angularPartDifference = difference.length();
         throw new AssertionError("expected:\n<" + expected + ">\n but was:\n<" + actual + ">\n difference: linear part: " + linearPartDifference
               + ", angular part: " + angularPartDifference);
      }
   }
}
