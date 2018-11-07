package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;

public class SpatialAccelerationCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 1000;

   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testWithChainComposedOfPrismaticJoints() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 20;
      List<PrismaticJoint> prismaticJoints = ScrewTestTools.createRandomChainRobotWithPrismaticJoints(numberOfJoints, random);
      RigidBodyBasics randomBody = prismaticJoints.get(random.nextInt(numberOfJoints)).getPredecessor();
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(randomBody);
      boolean doAccelerationTerms = true;

      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean doVelocityTerms = random.nextBoolean();

         FrameVector3D rootLinearAcceleration = new FrameVector3D(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         FrameVector3D rootAngularAcceleration = new FrameVector3D(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.getLinearPart().set(rootLinearAcceleration);
         rootAcceleration.getAngularPart().set(rootAngularAcceleration);
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(randomBody, rootAcceleration, doVelocityTerms,
                                                                                                         doAccelerationTerms);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -10.0, 10.0, prismaticJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -10.0, 10.0, prismaticJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, -10.0, 10.0, prismaticJoints);
         spatialAccelerationCalculator.getRootBody().updateFramesRecursively();

         spatialAccelerationCalculator.compute();

         FrameVector3D cumulatedLinearAcceleration = new FrameVector3D(rootLinearAcceleration);

         for (PrismaticJoint joint : prismaticJoints)
         {
            RigidBodyBasics body = joint.getSuccessor();
            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            spatialAccelerationCalculator.getAccelerationOfBody(body, actualAcceleration);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            SpatialAcceleration expectedAcceleration = new SpatialAcceleration(bodyFrame, worldFrame, bodyFrame);

            FrameVector3D jointAxis = new FrameVector3D(joint.getJointAxis());
            cumulatedLinearAcceleration.changeFrame(jointAxis.getReferenceFrame());
            cumulatedLinearAcceleration.scaleAdd(joint.getQdd(), jointAxis, cumulatedLinearAcceleration);
            cumulatedLinearAcceleration.changeFrame(bodyFrame);
            expectedAcceleration.getLinearPart().set(cumulatedLinearAcceleration);

            // Need to compute the cross part due to the root angular acceleration
            FramePoint3D rootPosition = new FramePoint3D(rootBody.getBodyFixedFrame());
            rootPosition.changeFrame(bodyFrame);
            rootAngularAcceleration.changeFrame(bodyFrame);
            Vector3D crossPart = new Vector3D();
            crossPart.cross(new Vector3D(rootPosition), rootAngularAcceleration);
            expectedAcceleration.getLinearPart().add(crossPart);

            expectedAcceleration.getAngularPart().set(rootAngularAcceleration);

            assertSpatialAccelerationVectorEquals(expectedAcceleration, actualAcceleration, 1.0e-12);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testWithChainComposedOfRevoluteJointsAssertAngularAccelerationOnly() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 20;
      List<RevoluteJoint> revoluteJoints = ScrewTestTools.createRandomChainRobot(numberOfJoints, random);
      RigidBodyBasics randomBody = revoluteJoints.get(random.nextInt(numberOfJoints)).getPredecessor();
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(randomBody);
      boolean doAccelerationTerms = true;

      // No velocity
      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean doVelocityTerms = random.nextBoolean();

         FrameVector3D rootLinearAcceleration = new FrameVector3D(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         FrameVector3D rootAngularAcceleration = new FrameVector3D(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.getLinearPart().set(rootLinearAcceleration);
         rootAcceleration.getAngularPart().set(rootAngularAcceleration);
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(randomBody, rootAcceleration, doVelocityTerms,
                                                                                                         doAccelerationTerms);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -10.0, 10.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, 0.0, 0.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, -10.0, 10.0, revoluteJoints);

         spatialAccelerationCalculator.compute();

         FrameVector3D cumulatedAngularAcceleration = new FrameVector3D(rootAngularAcceleration);

         for (RevoluteJoint joint : revoluteJoints)
         {
            RigidBodyBasics body = joint.getSuccessor();
            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            spatialAccelerationCalculator.getAccelerationOfBody(body, actualAcceleration);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            SpatialAcceleration expectedAcceleration = new SpatialAcceleration(bodyFrame, worldFrame, bodyFrame);

            FrameVector3D jointAxis = new FrameVector3D(joint.getJointAxis());
            cumulatedAngularAcceleration.changeFrame(jointAxis.getReferenceFrame());
            cumulatedAngularAcceleration.scaleAdd(joint.getQdd(), jointAxis, cumulatedAngularAcceleration);
            cumulatedAngularAcceleration.changeFrame(bodyFrame);
            expectedAcceleration.getAngularPart().set(cumulatedAngularAcceleration);

            expectedAcceleration.checkReferenceFrameMatch(actualAcceleration);

            EuclidCoreTestTools.assertTuple3DEquals(expectedAcceleration.getAngularPart(), actualAcceleration.getAngularPart(), 1.0e-12);
         }
      }

      // Non-zero velocities
      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean doVelocityTerms = random.nextBoolean();

         FrameVector3D rootLinearAcceleration = new FrameVector3D(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         FrameVector3D rootAngularAcceleration = new FrameVector3D(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.getLinearPart().set(rootLinearAcceleration);
         rootAcceleration.getAngularPart().set(rootAngularAcceleration);
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(randomBody, rootAcceleration, doVelocityTerms,
                                                                                                         doAccelerationTerms);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -10.0, 10.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -10.0, 10.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, -10.0, 10.0, revoluteJoints);
         spatialAccelerationCalculator.getRootBody().updateFramesRecursively();

         spatialAccelerationCalculator.compute();

         FrameVector3D cumulatedAngularAcceleration = new FrameVector3D(rootAngularAcceleration);

         for (int j = 0; j < revoluteJoints.size(); j++)
         {
            RevoluteJoint joint = revoluteJoints.get(j);
            RigidBodyBasics body = joint.getSuccessor();
            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            spatialAccelerationCalculator.getAccelerationOfBody(body, actualAcceleration);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            SpatialAcceleration expectedAcceleration = new SpatialAcceleration(bodyFrame, worldFrame, bodyFrame);

            FrameVector3D jointAxis = new FrameVector3D(joint.getJointAxis());
            cumulatedAngularAcceleration.changeFrame(jointAxis.getReferenceFrame());
            cumulatedAngularAcceleration.scaleAdd(joint.getQdd(), jointAxis, cumulatedAngularAcceleration);
            cumulatedAngularAcceleration.changeFrame(bodyFrame);

            if (doVelocityTerms)
            {
               // Need to account for the Coriolis acceleration
               Twist bodyTwist = new Twist();
               body.getBodyFixedFrame().getTwistOfFrame(bodyTwist);
               bodyTwist.changeFrame(bodyFrame);
               FrameVector3D coriolis = new FrameVector3D(bodyFrame);
               jointAxis.changeFrame(bodyFrame);
               coriolis.cross(bodyTwist.getAngularPart(), jointAxis);
               coriolis.scale(joint.getQd());
               cumulatedAngularAcceleration.add(coriolis);
            }

            expectedAcceleration.getAngularPart().set(cumulatedAngularAcceleration);
            expectedAcceleration.checkReferenceFrameMatch(bodyFrame, worldFrame, bodyFrame);

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

      RigidBodyBasics randomBody = joints.get(random.nextInt(joints.size())).getPredecessor();
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(randomBody);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, randomBody);
      TwistCalculator twistCalculatorInFuture = new TwistCalculator(worldFrame, jointsInFuture.get(random.nextInt(joints.size())).getPredecessor());

      double dt = 1.0e-8;

      for (int i = 0; i < ITERATIONS; i++)
      {
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.getLinearPart().set(EuclidCoreRandomTools.nextVector3D(random));
         rootAcceleration.getAngularPart().set(EuclidCoreRandomTools.nextVector3D(random));
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(randomBody, rootAcceleration, true, true);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -1.0, 1.0, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -1.0, 1.0, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, -1.0, 1.0, joints);

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
            RigidBodyBasics body = joint.getSuccessor();
            RigidBodyBasics bodyInFuture = jointsInFuture.get(jointIndex).getSuccessor();

            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            spatialAccelerationCalculator.getAccelerationOfBody(body, actualAcceleration);

            SpatialAcceleration expectedAcceleration = computeExpectedAccelerationByFiniteDifference(dt, body, bodyInFuture, rootAcceleration);

            assertSpatialAccelerationVectorEquals(expectedAcceleration, actualAcceleration, 1.0e-5);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 4.2)
   @Test(timeout = 30000)
   public void testWithFloatingJointRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(435345L);

      int numberOfRevoluteJoints = 100;
      RandomFloatingChain floatingChain = new RandomFloatingChain(random, numberOfRevoluteJoints);
      SixDoFJoint floatingJoint = floatingChain.getRootJoint();
      List<RevoluteJoint> revoluteJoints = floatingChain.getRevoluteJoints();
      List<JointBasics> joints = floatingChain.getInverseDynamicsJoints();
      List<JointBasics> jointsInFuture = Arrays.asList(ScrewTools.cloneJointPath(joints.toArray(new JointBasics[numberOfRevoluteJoints
            + 1])));
      SixDoFJoint floatingJointInFuture = (SixDoFJoint) jointsInFuture.get(0);
      List<RevoluteJoint> revoluteJointsInFuture = ScrewTools.filterJoints(jointsInFuture, RevoluteJoint.class);

      RigidBodyBasics randomBody = joints.get(0).getPredecessor();
      RigidBodyBasics randomBodyInFuture = jointsInFuture.get(0).getPredecessor();
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(randomBody);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, randomBody);
      TwistCalculator twistCalculatorInFuture = new TwistCalculator(worldFrame, randomBodyInFuture);

      double dt = 1.0e-8;

      for (int i = 0; i < ITERATIONS; i++)
      {
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.getLinearPart().set(EuclidCoreRandomTools.nextVector3D(random));
         rootAcceleration.getAngularPart().set(EuclidCoreRandomTools.nextVector3D(random));
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(randomBody, rootAcceleration, true, true);

         floatingJoint.setJointOrientation(RandomGeometry.nextQuaternion(random));
         floatingJoint.setJointPosition(RandomGeometry.nextPoint3D(random, -10.0, 10.0));
         Twist floatingJointTwist = MecanoRandomTools.nextTwist(random, floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(),
                                                                floatingJoint.getFrameAfterJoint());
         floatingJoint.setJointTwist(floatingJointTwist);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, floatingJoint);

         floatingJointInFuture.setJointConfiguration(floatingJoint);
         floatingJointInFuture.setJointTwist(floatingJoint);
         floatingJointInFuture.setJointAcceleration(floatingJoint);
         ScrewTestTools.doubleIntegrateFromAcceleration(floatingJointInFuture, dt);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -1.0, 1.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -1.0, 1.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, -1.0, 1.0, revoluteJoints);

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
            JointBasics joint = joints.get(jointIndex);
            RigidBodyBasics body = joint.getSuccessor();
            RigidBodyBasics bodyInFuture = jointsInFuture.get(jointIndex).getSuccessor();
            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            spatialAccelerationCalculator.getAccelerationOfBody(body, actualAcceleration);

            SpatialAcceleration expectedAcceleration = computeExpectedAccelerationByFiniteDifference(dt, body, bodyInFuture, rootAcceleration);

            assertSpatialAccelerationVectorEquals(expectedAcceleration, actualAcceleration, 1.0e-4);

            Point3D bodyFixedPoint = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
            FramePoint3D frameBodyFixedPoint = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
            FrameVector3D actualLinearAcceleration = new FrameVector3D();
            spatialAccelerationCalculator.getLinearAccelerationOfBodyFixedPoint(body, frameBodyFixedPoint, actualLinearAcceleration);
            FrameVector3D expectedLinearAcceleration = computeExpectedLinearAccelerationByFiniteDifference(dt, body, bodyInFuture, twistCalculator,
                                                                                                           twistCalculatorInFuture, bodyFixedPoint,
                                                                                                           rootAcceleration);

            expectedLinearAcceleration.checkReferenceFrameMatch(actualLinearAcceleration);
            EuclidCoreTestTools.assertTuple3DEquals(expectedLinearAcceleration, actualLinearAcceleration, 1.0e-4);

         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 8.6)
   @Test(timeout = 43000)
   public void testRelativeAccelerationWithFloatingJointRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(435345L);

      int numberOfRevoluteJoints = 100;
      RandomFloatingChain floatingChain = new RandomFloatingChain(random, numberOfRevoluteJoints);
      SixDoFJoint floatingJoint = floatingChain.getRootJoint();
      List<RevoluteJoint> revoluteJoints = floatingChain.getRevoluteJoints();
      List<JointBasics> joints = floatingChain.getInverseDynamicsJoints();
      List<JointBasics> jointsInFuture = Arrays.asList(ScrewTools.cloneJointPath(joints.toArray(new JointBasics[numberOfRevoluteJoints
            + 1])));
      SixDoFJoint floatingJointInFuture = (SixDoFJoint) jointsInFuture.get(0);
      List<RevoluteJoint> revoluteJointsInFuture = ScrewTools.filterJoints(jointsInFuture, RevoluteJoint.class);

      RigidBodyBasics randomBody = joints.get(random.nextInt(numberOfRevoluteJoints)).getPredecessor();
      RigidBodyBasics randomBodyInFuture = jointsInFuture.get(random.nextInt(numberOfRevoluteJoints)).getPredecessor();
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(randomBody);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, randomBody);
      TwistCalculator twistCalculatorInFuture = new TwistCalculator(worldFrame, randomBodyInFuture);

      double dt = 1.0e-8;

      for (int i = 0; i < 50; i++)
      {
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.getLinearPart().set(EuclidCoreRandomTools.nextVector3D(random));
         rootAcceleration.getAngularPart().set(EuclidCoreRandomTools.nextVector3D(random));
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(randomBody, rootAcceleration, true, true);

         floatingJoint.setJointOrientation(RandomGeometry.nextQuaternion(random));
         floatingJoint.setJointPosition(RandomGeometry.nextPoint3D(random, -10.0, 10.0));
         Twist floatingJointTwist = MecanoRandomTools.nextTwist(random, floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(),
                                                                floatingJoint.getFrameAfterJoint());
         floatingJoint.setJointTwist(floatingJointTwist);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, floatingJoint);

         floatingJointInFuture.setJointConfiguration(floatingJoint);
         floatingJointInFuture.setJointTwist(floatingJoint);
         floatingJointInFuture.setJointAcceleration(floatingJoint);
         ScrewTestTools.doubleIntegrateFromAcceleration(floatingJointInFuture, dt);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -1.0, 1.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -1.0, 1.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, -1.0, 1.0, revoluteJoints);

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
            JointBasics joint = joints.get(jointIndex);
            RigidBodyBasics body = joint.getSuccessor();
            RigidBodyBasics bodyInFuture = jointsInFuture.get(jointIndex).getSuccessor();
            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            spatialAccelerationCalculator.getAccelerationOfBody(body, actualAcceleration);

            SpatialAcceleration expectedAcceleration = computeExpectedAccelerationByFiniteDifference(dt, body, bodyInFuture, rootAcceleration);

            assertSpatialAccelerationVectorEquals(expectedAcceleration, actualAcceleration, 1.0e-4);

            // Assert relative twist
            for (int baseJointIndex = 0; baseJointIndex < numberOfRevoluteJoints + 1; baseJointIndex++)
            {
               RigidBodyBasics base = joints.get(baseJointIndex).getSuccessor();
               RigidBodyBasics baseInFuture = jointsInFuture.get(baseJointIndex).getSuccessor();
               SpatialAcceleration actualRelativeAcceleration = new SpatialAcceleration();
               spatialAccelerationCalculator.getRelativeAcceleration(base, body, actualRelativeAcceleration);

               SpatialAcceleration expectedRelativeAcceleration = computeExpectedRelativeAccelerationByFiniteDifference(dt, body, bodyInFuture, base,
                                                                                                                        baseInFuture, twistCalculator,
                                                                                                                        twistCalculatorInFuture,
                                                                                                                        rootAcceleration);

               assertSpatialAccelerationVectorEquals(expectedRelativeAcceleration, actualRelativeAcceleration, 1.0e-4);

               Point3D bodyFixedPoint = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
               FramePoint3D frameBodyFixedPoint = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
               FrameVector3D actualLinearAcceleration = new FrameVector3D();
               spatialAccelerationCalculator.getLinearAccelerationOfBodyFixedPoint(base, body, frameBodyFixedPoint, actualLinearAcceleration);
               FrameVector3D expectedLinearAcceleration = computeExpectedLinearAccelerationByFiniteDifference(dt, body, bodyInFuture, base, baseInFuture,
                                                                                                              twistCalculator, twistCalculatorInFuture,
                                                                                                              bodyFixedPoint);

               expectedLinearAcceleration.checkReferenceFrameMatch(actualLinearAcceleration);
               EuclidCoreTestTools.assertTuple3DEquals("iteration: " + i + ", joint index: " + baseJointIndex, expectedLinearAcceleration,
                                                       actualLinearAcceleration, 7.0e-5);
            }
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 7.3)
   @Test(timeout = 37000)
   public void testWithDoVelocityTermsSetToFalse() throws Exception
   {
      Random random = new Random(435345L);

      int numberOfRevoluteJoints = 100;
      RandomFloatingChain floatingChain = new RandomFloatingChain(random, numberOfRevoluteJoints);
      SixDoFJoint floatingJoint = floatingChain.getRootJoint();
      List<RevoluteJoint> revoluteJoints = floatingChain.getRevoluteJoints();
      List<JointBasics> joints = floatingChain.getInverseDynamicsJoints();
      List<JointBasics> jointsNoVelocity = Arrays.asList(ScrewTools.cloneJointPath(joints.toArray(new JointBasics[numberOfRevoluteJoints
            + 1]), ""));
      SixDoFJoint floatingJointNoVelocity = (SixDoFJoint) jointsNoVelocity.get(0);
      List<RevoluteJoint> revoluteJointsNoVelocity = ScrewTools.filterJoints(jointsNoVelocity, RevoluteJoint.class);

      RigidBodyBasics randomBody = joints.get(random.nextInt(numberOfRevoluteJoints)).getPredecessor();
      RigidBodyBasics randomBodyNoVelocity = jointsNoVelocity.get(random.nextInt(numberOfRevoluteJoints)).getPredecessor();
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(randomBody);
      RigidBodyBasics rootBodyNoVelocity = MultiBodySystemTools.getRootBody(randomBodyNoVelocity);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, randomBody);
      TwistCalculator twistCalculatorNoVelocity = new TwistCalculator(worldFrame, randomBodyNoVelocity);

      for (int i = 0; i < 50; i++)
      {
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.getLinearPart().set(EuclidCoreRandomTools.nextVector3D(random));
         rootAcceleration.getAngularPart().set(EuclidCoreRandomTools.nextVector3D(random));
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(randomBody, rootAcceleration, false, true);

         SpatialAcceleration rootAccelerationNoVelocity = new SpatialAcceleration(rootBodyNoVelocity.getBodyFixedFrame(), worldFrame,
                                                                                  rootBodyNoVelocity.getBodyFixedFrame());
         rootAccelerationNoVelocity.getLinearPart().set( (Vector3DReadOnly) rootAcceleration.getLinearPart());
         rootAccelerationNoVelocity.getAngularPart().set((Vector3DReadOnly) rootAcceleration.getAngularPart());
         SpatialAccelerationCalculator spatialAccelerationCalculatorNoVelocity = new SpatialAccelerationCalculator(randomBodyNoVelocity,
                                                                                                                   rootAccelerationNoVelocity, true, true);

         Quaternion floatingJointRotation = RandomGeometry.nextQuaternion(random);
         Point3D floatingJointPosition = RandomGeometry.nextPoint3D(random, -10.0, 10.0);

         floatingJoint.setJointOrientation(floatingJointRotation);
         floatingJoint.setJointPosition(floatingJointPosition);
         floatingJointNoVelocity.setJointOrientation(floatingJointRotation);
         floatingJointNoVelocity.setJointPosition(floatingJointPosition);

         SpatialAcceleration floatJointAcceleration = new SpatialAcceleration(floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(),
                                                                              floatingJoint.getFrameAfterJoint());
         floatJointAcceleration.getLinearPart().set(EuclidCoreRandomTools.nextVector3D(random));
         floatJointAcceleration.getAngularPart().set(EuclidCoreRandomTools.nextVector3D(random));
         floatingJoint.setJointAcceleration(floatJointAcceleration);
         SpatialAcceleration floatJointAccelerationNoVelocity = new SpatialAcceleration(floatingJointNoVelocity.getFrameAfterJoint(),
                                                                                        floatingJointNoVelocity.getFrameBeforeJoint(),
                                                                                        floatingJointNoVelocity.getFrameAfterJoint());
         floatJointAccelerationNoVelocity.getLinearPart().set((Vector3DReadOnly) floatJointAcceleration.getLinearPart());
         floatJointAccelerationNoVelocity.getAngularPart().set((Vector3DReadOnly) floatJointAcceleration.getAngularPart());
         floatingJointNoVelocity.setJointAcceleration(floatJointAccelerationNoVelocity);

         Twist floatingJointTwist = MecanoRandomTools.nextTwist(random, floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(),
                                                                floatingJoint.getFrameAfterJoint());
         floatingJoint.setJointTwist(floatingJointTwist);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -1.0, 1.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -10.0, 10.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, -1.0, 1.0, revoluteJoints);

         for (int jointIndex = 0; jointIndex < revoluteJointsNoVelocity.size(); jointIndex++)
         {
            RevoluteJoint joint = revoluteJoints.get(jointIndex);
            RevoluteJoint jointNoVelocity = revoluteJointsNoVelocity.get(jointIndex);
            jointNoVelocity.setQ(joint.getQ());
            jointNoVelocity.setQd(0.0);
            jointNoVelocity.setQdd(joint.getQdd());
         }

         floatingJoint.updateFramesRecursively();
         floatingJointNoVelocity.updateFramesRecursively();

         twistCalculator.compute();
         spatialAccelerationCalculator.compute();
         twistCalculatorNoVelocity.compute();
         spatialAccelerationCalculatorNoVelocity.compute();

         for (int jointIndex = 0; jointIndex < numberOfRevoluteJoints + 1; jointIndex++)
         {
            JointBasics joint = joints.get(jointIndex);
            RigidBodyBasics body = joint.getSuccessor();
            RigidBodyBasics bodyNoVelocity = jointsNoVelocity.get(jointIndex).getSuccessor();
            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            spatialAccelerationCalculator.getAccelerationOfBody(body, actualAcceleration);

            SpatialAcceleration expectedAcceleration = new SpatialAcceleration();
            spatialAccelerationCalculatorNoVelocity.getAccelerationOfBody(bodyNoVelocity, expectedAcceleration);

            assertSpatialAccelerationVectorEquals(expectedAcceleration, actualAcceleration, 1.0e-12, true);

            Point3D bodyFixedPoint = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
            FramePoint3D frameBodyFixedPoint = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
            FrameVector3D actualLinearAcceleration = new FrameVector3D();
            spatialAccelerationCalculator.getLinearAccelerationOfBodyFixedPoint(body, frameBodyFixedPoint, actualLinearAcceleration);
            FrameVector3D expectedLinearAcceleration = new FrameVector3D();
            spatialAccelerationCalculatorNoVelocity.getLinearAccelerationOfBodyFixedPoint(bodyNoVelocity, frameBodyFixedPoint, expectedLinearAcceleration);

            assertEquals(expectedLinearAcceleration.getReferenceFrame().getName(), actualLinearAcceleration.getReferenceFrame().getName());
            EuclidCoreTestTools.assertTuple3DEquals(expectedLinearAcceleration, actualLinearAcceleration, 1.0e-12);

            // Assert relative twist
            for (int baseJointIndex = 0; baseJointIndex < numberOfRevoluteJoints + 1; baseJointIndex++)
            {
               RigidBodyBasics base = joints.get(baseJointIndex).getSuccessor();
               RigidBodyBasics baseNoVelocity = jointsNoVelocity.get(baseJointIndex).getSuccessor();
               SpatialAcceleration actualRelativeAcceleration = new SpatialAcceleration();
               spatialAccelerationCalculator.getRelativeAcceleration(base, body, actualRelativeAcceleration);

               SpatialAcceleration expectedRelativeAcceleration = new SpatialAcceleration();
               spatialAccelerationCalculatorNoVelocity.getRelativeAcceleration(baseNoVelocity, bodyNoVelocity, expectedRelativeAcceleration);

               String messagePrefix = "iteration: " + i + ", joint index: " + jointIndex + ", base joint index: " + baseJointIndex;
               assertSpatialAccelerationVectorEquals(messagePrefix, expectedRelativeAcceleration, actualRelativeAcceleration, 1.0e-12, true);

               bodyFixedPoint = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
               frameBodyFixedPoint = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
               spatialAccelerationCalculator.getLinearAccelerationOfBodyFixedPoint(base, body, frameBodyFixedPoint, actualLinearAcceleration);
               spatialAccelerationCalculatorNoVelocity.getLinearAccelerationOfBodyFixedPoint(baseNoVelocity, bodyNoVelocity, frameBodyFixedPoint,
                                                                                             expectedLinearAcceleration);

               assertEquals(expectedLinearAcceleration.getReferenceFrame().getName(), actualLinearAcceleration.getReferenceFrame().getName());
               EuclidCoreTestTools.assertTuple3DEquals(messagePrefix, expectedLinearAcceleration, actualLinearAcceleration, 1.0e-12);
            }
         }
      }
   }

   public static FrameVector3D computeExpectedLinearAccelerationByFiniteDifference(double dt, RigidBodyBasics body, RigidBodyBasics bodyInFuture,
                                                                                   TwistCalculator twistCalculator, TwistCalculator twistCalculatorInFuture,
                                                                                   Point3D bodyFixedPoint, SpatialAcceleration rootAcceleration)
   {
      FrameVector3D pointLinearVelocity = new FrameVector3D();
      FrameVector3D pointLinearVelocityInFuture = new FrameVector3D();

      FramePoint3D point = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
      FramePoint3D pointInFuture = new FramePoint3D(bodyInFuture.getBodyFixedFrame(), bodyFixedPoint);

      twistCalculator.getLinearVelocityOfBodyFixedPoint(body, point, pointLinearVelocity);
      twistCalculatorInFuture.getLinearVelocityOfBodyFixedPoint(bodyInFuture, pointInFuture, pointLinearVelocityInFuture);

      FrameVector3D pointLinearAcceleration = new FrameVector3D(worldFrame);
      pointLinearAcceleration.sub(pointLinearVelocityInFuture, pointLinearVelocity);
      pointLinearAcceleration.scale(1.0 / dt);

      // Need to account for the root acceleration
      rootAcceleration.getBodyFrame().checkReferenceFrameMatch(rootAcceleration.getReferenceFrame());
      FrameVector3D rootAngularAcceleration = new FrameVector3D();
      FrameVector3D rootLinearAcceleration = new FrameVector3D();
      rootAngularAcceleration.setIncludingFrame(rootAcceleration.getAngularPart());
      rootLinearAcceleration.setIncludingFrame(rootAcceleration.getLinearPart());

      FramePoint3D bodyFixedPointToRoot = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
      bodyFixedPointToRoot.changeFrame(rootAcceleration.getBodyFrame());
      FrameVector3D crossPart = new FrameVector3D(rootAcceleration.getBodyFrame());
      crossPart.cross(bodyFixedPointToRoot, rootAngularAcceleration);
      crossPart.changeFrame(worldFrame);
      rootLinearAcceleration.changeFrame(worldFrame);
      pointLinearAcceleration.sub(crossPart);
      pointLinearAcceleration.add(rootLinearAcceleration);

      return pointLinearAcceleration;

   }

   public static FrameVector3D computeExpectedLinearAccelerationByFiniteDifference(double dt, RigidBodyBasics body, RigidBodyBasics bodyInFuture, RigidBodyBasics base,
                                                                                   RigidBodyBasics baseInFuture, TwistCalculator twistCalculator,
                                                                                   TwistCalculator twistCalculatorInFuture, Point3D bodyFixedPoint)
   {
      FrameVector3D pointLinearVelocity = new FrameVector3D();
      FrameVector3D pointLinearVelocityInFuture = new FrameVector3D();

      FramePoint3D point = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
      FramePoint3D pointInFuture = new FramePoint3D(bodyInFuture.getBodyFixedFrame(), bodyFixedPoint);

      twistCalculator.getLinearVelocityOfBodyFixedPoint(base, body, point, pointLinearVelocity);
      twistCalculatorInFuture.getLinearVelocityOfBodyFixedPoint(baseInFuture, bodyInFuture, pointInFuture, pointLinearVelocityInFuture);

      FrameVector3D pointLinearAcceleration = new FrameVector3D(base.getBodyFixedFrame());
      pointLinearAcceleration.sub((Vector3DReadOnly) pointLinearVelocityInFuture, pointLinearVelocity);
      pointLinearAcceleration.scale(1.0 / dt);

      return pointLinearAcceleration;

   }

   private SpatialAcceleration computeExpectedRelativeAccelerationByFiniteDifference(double dt, RigidBodyBasics body, RigidBodyBasics bodyInFuture, RigidBodyBasics base,
                                                                                     RigidBodyBasics baseInFuture, TwistCalculator twistCalculator,
                                                                                     TwistCalculator twistCalculatorInFuture,
                                                                                     SpatialAcceleration rootAcceleration)
   {
      SpatialAcceleration expectedAcceleration = new SpatialAcceleration(body.getBodyFixedFrame(), base.getBodyFixedFrame(), body.getBodyFixedFrame());

      Twist relativeTwist = new Twist();
      Twist relativeTwistInFuture = new Twist();

      twistCalculator.getRelativeTwist(base, body, relativeTwist);
      twistCalculatorInFuture.getRelativeTwist(baseInFuture, bodyInFuture, relativeTwistInFuture);

      Vector3D angularAcceleration = firstOrderFiniteDifference(dt, relativeTwist.getAngularPart(), relativeTwistInFuture.getAngularPart());
      Vector3D linearAcceleration = firstOrderFiniteDifference(dt, relativeTwist.getLinearPart(), relativeTwistInFuture.getLinearPart());
      expectedAcceleration.getAngularPart().set(angularAcceleration);
      expectedAcceleration.getLinearPart().set(linearAcceleration);

      return expectedAcceleration;
   }

   private static SpatialAcceleration computeExpectedAccelerationByFiniteDifference(double dt, RigidBodyBasics body, RigidBodyBasics bodyInFuture,
                                                                                    SpatialAcceleration rootAcceleration)
   {
      SpatialAcceleration expectedAcceleration = new SpatialAcceleration(body.getBodyFixedFrame(), worldFrame, body.getBodyFixedFrame());

      Twist bodyTwist = new Twist();
      Twist bodyTwistInFuture = new Twist();

      body.getBodyFixedFrame().getTwistOfFrame(bodyTwist);
      bodyInFuture.getBodyFixedFrame().getTwistOfFrame(bodyTwistInFuture);

      Vector3D angularAcceleration = firstOrderFiniteDifference(dt, bodyTwist.getAngularPart(), bodyTwistInFuture.getAngularPart());
      Vector3D linearAcceleration = firstOrderFiniteDifference(dt, bodyTwist.getLinearPart(), bodyTwistInFuture.getLinearPart());
      expectedAcceleration.getAngularPart().set(angularAcceleration);
      expectedAcceleration.getLinearPart().set(linearAcceleration);

      // Need to account for the root acceleration
      FrameVector3D rootAngularAcceleration = new FrameVector3D(rootAcceleration.getReferenceFrame(), rootAcceleration.getAngularPart());
      FrameVector3D rootLinearAcceleration = new FrameVector3D(rootAcceleration.getReferenceFrame(), rootAcceleration.getLinearPart());
      rootAngularAcceleration.changeFrame(expectedAcceleration.getReferenceFrame());
      rootLinearAcceleration.changeFrame(expectedAcceleration.getReferenceFrame());
      expectedAcceleration.getAngularPart().add(rootAngularAcceleration);

      FramePoint3D rootToBody = new FramePoint3D(rootAcceleration.getReferenceFrame());
      rootToBody.changeFrame(expectedAcceleration.getReferenceFrame());
      Vector3D crossPart = new Vector3D();
      crossPart.cross(rootToBody, rootAngularAcceleration);
      expectedAcceleration.getLinearPart().add(crossPart);
      expectedAcceleration.getLinearPart().add(rootLinearAcceleration);

      return expectedAcceleration;
   }

   public static Vector3D firstOrderFiniteDifference(double dt, Vector3DReadOnly now, Vector3DReadOnly next)
   {
      Vector3D finiteDifference = new Vector3D();
      finiteDifference.sub(next, now);
      finiteDifference.scale(1.0 / dt);
      return finiteDifference;
   }

   public static void assertSpatialAccelerationVectorEquals(SpatialAcceleration expected, SpatialAcceleration actual, double epsilon) throws AssertionError
   {
      assertSpatialAccelerationVectorEquals(null, expected, actual, epsilon, false);
   }

   public static void assertSpatialAccelerationVectorEquals(SpatialAcceleration expected, SpatialAcceleration actual, double epsilon, boolean checkFrameByName)
   {
      assertSpatialAccelerationVectorEquals(null, expected, actual, epsilon, checkFrameByName);
   }

   public static void assertSpatialAccelerationVectorEquals(String messagePrefix, SpatialAcceleration expected, SpatialAcceleration actual, double epsilon,
                                                            boolean checkFrameByName)
         throws AssertionError
   {
      try
      {
         if (checkFrameByName)
         {
            assertEquals(expected.getBodyFrame().getName(), actual.getBodyFrame().getName());
            assertEquals(expected.getBaseFrame().getName(), actual.getBaseFrame().getName());
            assertEquals(expected.getReferenceFrame().getName(), actual.getReferenceFrame().getName());
            EuclidCoreTestTools.assertTuple3DEquals(expected.getAngularPart(), actual.getAngularPart(), epsilon);
            EuclidCoreTestTools.assertTuple3DEquals(expected.getLinearPart(), actual.getLinearPart(), epsilon);
         }
         else
         {
            assertTrue(expected.epsilonEquals(actual, epsilon));
         }
      }
      catch (AssertionError e)
      {
         Vector3D difference = new Vector3D();
         difference.sub(expected.getLinearPart(), actual.getLinearPart());
         double linearPartDifference = difference.length();
         difference.sub(expected.getAngularPart(), actual.getAngularPart());
         double angularPartDifference = difference.length();
         messagePrefix = messagePrefix != null ? messagePrefix + " " : "";
         throw new AssertionError(messagePrefix + "expected:\n<" + expected + ">\n but was:\n<" + actual + ">\n difference: linear part: "
               + linearPartDifference + ", angular part: " + angularPartDifference);
      }
   }
}
