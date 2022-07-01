package us.ihmc.robotics.screwTheory;

import static us.ihmc.robotics.Assert.*;

import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.random.RandomGeometry;

public class TwistCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testWithChainComposedOfPrismaticJoints() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 20;
      List<PrismaticJoint> prismaticJoints = MultiBodySystemRandomTools.nextPrismaticJointChain(random, numberOfJoints);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, prismaticJoints.get(random.nextInt(numberOfJoints)).getPredecessor());

      for (int i = 0; i < 100; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -10.0, 10.0, prismaticJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -10.0, 10.0, prismaticJoints);
         twistCalculator.compute();

         FrameVector3D cumulatedLinearVelocity = new FrameVector3D(worldFrame);

         for (PrismaticJoint joint : prismaticJoints)
         {
            RigidBodyBasics body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

            FrameVector3D jointAxis = new FrameVector3D(joint.getJointAxis());
            cumulatedLinearVelocity.changeFrame(jointAxis.getReferenceFrame());
            cumulatedLinearVelocity.scaleAdd(joint.getQd(), jointAxis, cumulatedLinearVelocity);
            cumulatedLinearVelocity.changeFrame(bodyFrame);
            expectedTwist.getLinearPart().set(cumulatedLinearVelocity);

            MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, 1.0e-12);
         }
      }
   }

   @Test
   public void testWithChainComposedOfRevoluteJointsAssertAngularVelocityOnly() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 20;
      List<RevoluteJoint> revoluteJoints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, revoluteJoints.get(random.nextInt(numberOfJoints)).getPredecessor());

      for (int i = 0; i < 100; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -10.0, 10.0, revoluteJoints);
         twistCalculator.compute();

         FrameVector3D cumulatedAngularVelocity = new FrameVector3D(worldFrame);

         for (RevoluteJoint joint : revoluteJoints)
         {
            RigidBodyBasics body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

            FrameVector3D jointAxis = new FrameVector3D(joint.getJointAxis());
            cumulatedAngularVelocity.changeFrame(jointAxis.getReferenceFrame());
            cumulatedAngularVelocity.scaleAdd(joint.getQd(), jointAxis, cumulatedAngularVelocity);
            cumulatedAngularVelocity.changeFrame(bodyFrame);
            expectedTwist.getAngularPart().set(cumulatedAngularVelocity);

            expectedTwist.checkReferenceFrameMatch(actualTwist);

            assertTrue(expectedTwist.getAngularPart().epsilonEquals(actualTwist.getAngularPart(), 1.0e-12));
         }
      }
   }

   @Test
   public void testWithTreeComposedOfPrismaticJoints() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 100;
      List<PrismaticJoint> prismaticJoints = MultiBodySystemRandomTools.nextPrismaticJointTree(random, numberOfJoints);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, prismaticJoints.get(random.nextInt(numberOfJoints)).getPredecessor());

      for (int i = 0; i < 100; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -10.0, 10.0, prismaticJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -10.0, 10.0, prismaticJoints);
         twistCalculator.compute();

         for (PrismaticJoint joint : prismaticJoints)
         {
            RigidBodyBasics body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

            RigidBodyBasics currentBody = body;
            FrameVector3D cumulatedLinearVelocity = new FrameVector3D(worldFrame);

            while (currentBody.getParentJoint() != null)
            {
               PrismaticJoint parentJoint = (PrismaticJoint) currentBody.getParentJoint();
               FrameVector3D jointAxis = new FrameVector3D(parentJoint.getJointAxis());
               cumulatedLinearVelocity.changeFrame(jointAxis.getReferenceFrame());
               cumulatedLinearVelocity.scaleAdd(parentJoint.getQd(), jointAxis, cumulatedLinearVelocity);
               currentBody = parentJoint.getPredecessor();
            }

            cumulatedLinearVelocity.changeFrame(bodyFrame);
            expectedTwist.getLinearPart().set(cumulatedLinearVelocity);

            MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, 1.0e-12);
         }
      }
   }

   @Test
   public void testWithTreeComposedOfRevoluteJointsAssertAngularVelocity() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 100;
      List<RevoluteJoint> revoluteJoints = MultiBodySystemRandomTools.nextRevoluteJointTree(random, numberOfJoints);
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, revoluteJoints.get(random.nextInt(numberOfJoints)).getPredecessor());

      for (int i = 0; i < 100; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -10.0, 10.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -10.0, 10.0, revoluteJoints);
         twistCalculator.compute();

         for (RevoluteJoint joint : revoluteJoints)
         {
            RigidBodyBasics body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

            RigidBodyBasics currentBody = body;
            FrameVector3D cumulatedAngularVelocity = new FrameVector3D(worldFrame);

            while (currentBody.getParentJoint() != null)
            {
               RevoluteJoint parentJoint = (RevoluteJoint) currentBody.getParentJoint();
               FrameVector3D jointAxis = new FrameVector3D(parentJoint.getJointAxis());
               cumulatedAngularVelocity.changeFrame(jointAxis.getReferenceFrame());
               cumulatedAngularVelocity.scaleAdd(parentJoint.getQd(), jointAxis, cumulatedAngularVelocity);
               currentBody = parentJoint.getPredecessor();
            }

            cumulatedAngularVelocity.changeFrame(bodyFrame);
            expectedTwist.getAngularPart().set(cumulatedAngularVelocity);

            expectedTwist.checkReferenceFrameMatch(actualTwist);

            assertTrue(expectedTwist.getAngularPart().epsilonEquals(actualTwist.getAngularPart(), 1.0e-12));
         }
      }
   }

   @Test
   public void testWithChainRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(234234L);

      int numberOfJoints = 10;
      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
      List<OneDoFJointBasics> jointsInFuture = Arrays.asList(MultiBodySystemFactories.cloneOneDoFJointKinematicChain(joints.toArray(new OneDoFJointBasics[numberOfJoints])));

      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, joints.get(0).getPredecessor());

      double dt = 1.0e-8;

      for (int i = 0; i < 100; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -1.0, 1.0, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -1.0, 1.0, joints);

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
            OneDoFJointBasics joint = joints.get(jointIndex);
            RigidBodyBasics body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            ReferenceFrame bodyFrameInFuture = jointsInFuture.get(jointIndex).getSuccessor().getBodyFixedFrame();
            Twist expectedTwist = computeExpectedTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);

            MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);
         }
      }
   }

   @Test
   public void testWithTreeRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(234234L);

      int numberOfJoints = 100;
      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointTree(random, numberOfJoints);
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      RigidBodyBasics rootBodyInFuture = MultiBodySystemFactories.cloneMultiBodySystem(rootBody, worldFrame, "Test");
      List<OneDoFJointBasics> jointsInFuture = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBodyInFuture).collect(Collectors.toList());

      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, rootBody);

      double dt = 1.0e-8;

      for (int i = 0; i < 100; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -1.0, 1.0, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -1.0, 1.0, joints);

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
            OneDoFJointBasics joint = joints.get(jointIndex);
            RigidBodyBasics body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            ReferenceFrame bodyFrameInFuture = jointsInFuture.get(jointIndex).getSuccessor().getBodyFixedFrame();
            Twist expectedTwist = computeExpectedTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);

            MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);
         }
      }
   }

   @Test
   public void testWithFloatingJointRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(435345L);

      int numberOfRevoluteJoints = 100;
      RandomFloatingRevoluteJointChain floatingChain = new RandomFloatingRevoluteJointChain(random, numberOfRevoluteJoints);
      SixDoFJoint floatingJoint = floatingChain.getRootJoint();
      List<RevoluteJoint> revoluteJoints = floatingChain.getRevoluteJoints();
      List<Joint> joints = floatingChain.getJoints();
      List<JointBasics> jointsInFuture = Arrays.asList(MultiBodySystemFactories.cloneKinematicChain(joints.toArray(new JointBasics[numberOfRevoluteJoints
      + 1])));
      SixDoFJoint floatingJointInFuture = (SixDoFJoint) jointsInFuture.get(0);
      List<RevoluteJoint> revoluteJointsInFuture = MultiBodySystemTools.filterJoints(jointsInFuture, RevoluteJoint.class);

      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, joints.get(0).getPredecessor());

      double dt = 1.0e-8;
      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dt);

      for (int i = 0; i < 100; i++)
      {
         floatingJoint.setJointOrientation(RandomGeometry.nextQuaternion(random));
         floatingJoint.setJointPosition(RandomGeometry.nextPoint3D(random, -10.0, 10.0));
         Twist floatingJointTwist = MecanoRandomTools.nextTwist(random, floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(),
                                                                floatingJoint.getFrameAfterJoint());
         floatingJoint.setJointTwist(floatingJointTwist);

         floatingJointInFuture.setJointConfiguration(floatingJoint);
         floatingJointInFuture.setJointTwist(floatingJoint);
         floatingJointInFuture.setJointAcceleration(floatingJoint);
         integrator.integrateFromVelocity(floatingJointInFuture);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -1.0, 1.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -1.0, 1.0, revoluteJoints);

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
            JointBasics joint = joints.get(jointIndex);
            RigidBodyBasics body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            ReferenceFrame bodyFrameInFuture = jointsInFuture.get(jointIndex).getSuccessor().getBodyFixedFrame();
            Twist expectedTwist = computeExpectedTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);

            MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);

            Point3D bodyFixedPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
            FramePoint3D frameBodyFixedPoint = new FramePoint3D(bodyFrame, bodyFixedPoint);
            FrameVector3D actualLinearVelocity = new FrameVector3D();
            twistCalculator.getLinearVelocityOfBodyFixedPoint(body, frameBodyFixedPoint, actualLinearVelocity);
            FrameVector3D expectedLinearVelocity = computeExpectedLinearVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture, bodyFixedPoint);

            expectedLinearVelocity.checkReferenceFrameMatch(actualLinearVelocity);
            EuclidCoreTestTools.assertEquals(expectedLinearVelocity, actualLinearVelocity, 2.0e-5);

            FrameVector3D expectedAngularVelocity = computeAngularVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);
            FrameVector3D actualAngularVelocity = new FrameVector3D();
            twistCalculator.getAngularVelocityOfBody(body, actualAngularVelocity);

            expectedAngularVelocity.checkReferenceFrameMatch(actualAngularVelocity);
            EuclidCoreTestTools.assertEquals(expectedAngularVelocity, actualAngularVelocity, 1.0e-5);
         }
      }
   }

   @Test
   public void testRelativeTwistWithFloatingJointRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(435345L);

      int numberOfRevoluteJoints = 100;
      RandomFloatingRevoluteJointChain floatingChain = new RandomFloatingRevoluteJointChain(random, numberOfRevoluteJoints);
      SixDoFJoint floatingJoint = floatingChain.getRootJoint();
      List<RevoluteJoint> revoluteJoints = floatingChain.getRevoluteJoints();
      List<Joint> joints = floatingChain.getJoints();
      List<JointBasics> jointsInFuture = Arrays.asList(MultiBodySystemFactories.cloneKinematicChain(joints.toArray(new JointBasics[numberOfRevoluteJoints
      + 1])));
      SixDoFJoint floatingJointInFuture = (SixDoFJoint) jointsInFuture.get(0);
      List<RevoluteJoint> revoluteJointsInFuture = MultiBodySystemTools.filterJoints(jointsInFuture, RevoluteJoint.class);

      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, joints.get(random.nextInt(numberOfRevoluteJoints)).getPredecessor());

      double dt = 1.0e-8;
      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dt);

      for (int i = 0; i < 50; i++)
      {
         floatingJoint.setJointOrientation(RandomGeometry.nextQuaternion(random));
         floatingJoint.setJointPosition(RandomGeometry.nextPoint3D(random, -10.0, 10.0));
         Twist floatingJointTwist = MecanoRandomTools.nextTwist(random, floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(),
                                                                floatingJoint.getFrameAfterJoint());
         floatingJoint.setJointTwist(floatingJointTwist);

         floatingJointInFuture.setJointConfiguration(floatingJoint);
         floatingJointInFuture.setJointTwist(floatingJoint);
         floatingJointInFuture.setJointAcceleration(floatingJoint);
         integrator.integrateFromVelocity(floatingJointInFuture);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -1.0, 1.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -1.0, 1.0, revoluteJoints);

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
            JointBasics joint = joints.get(jointIndex);
            RigidBodyBasics body = joint.getSuccessor();
            Twist actualTwist = new Twist();
            twistCalculator.getTwistOfBody(body, actualTwist);

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            ReferenceFrame bodyFrameInFuture = jointsInFuture.get(jointIndex).getSuccessor().getBodyFixedFrame();
            Twist expectedTwist = computeExpectedTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);

            MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);

            // Assert relative twist
            for (int baseJointIndex = 0; baseJointIndex < numberOfRevoluteJoints + 1; baseJointIndex++)
            {
               RigidBodyBasics base = joints.get(baseJointIndex).getSuccessor();
               Twist actualRelativeTwist = new Twist();
               twistCalculator.getRelativeTwist(base, body, actualRelativeTwist);

               ReferenceFrame baseFrame = base.getBodyFixedFrame();
               ReferenceFrame baseFrameInFuture = jointsInFuture.get(baseJointIndex).getSuccessor().getBodyFixedFrame();
               Twist expectedRelativeTwist = computeExpectedRelativeTwistByFiniteDifference(dt, bodyFrame, bodyFrameInFuture, baseFrame, baseFrameInFuture);

               MecanoTestTools.assertTwistEquals(expectedRelativeTwist, actualRelativeTwist, 1.0e-5);

               Point3D bodyFixedPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               FramePoint3D frameBodyFixedPoint = new FramePoint3D(bodyFrame, bodyFixedPoint);
               FrameVector3D actualLinearVelocity = new FrameVector3D();
               twistCalculator.getLinearVelocityOfBodyFixedPoint(base, body, frameBodyFixedPoint, actualLinearVelocity);
               FrameVector3D expectedLinearVelocity = computeExpectedLinearVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture, baseFrame,
                                                                                                      baseFrameInFuture, bodyFixedPoint);

               expectedLinearVelocity.checkReferenceFrameMatch(actualLinearVelocity);
               EuclidCoreTestTools.assertEquals(expectedLinearVelocity, actualLinearVelocity, 2.0e-5);

               FrameVector3D expectedAngularVelocity = new FrameVector3D();
               expectedAngularVelocity.setIncludingFrame(expectedRelativeTwist.getAngularPart());
               FrameVector3D actualAngularVelocity = new FrameVector3D();
               twistCalculator.getRelativeAngularVelocity(base, body, actualAngularVelocity);

               expectedAngularVelocity.checkReferenceFrameMatch(actualAngularVelocity);
               EuclidCoreTestTools.assertEquals(expectedAngularVelocity, actualAngularVelocity, 1.0e-5);
            }
         }
      }
   }

   public static FrameVector3D computeExpectedLinearVelocityByFiniteDifference(double dt, ReferenceFrame bodyFrame, ReferenceFrame bodyFrameInFuture,
                                                                               Point3D bodyFixedPoint)
   {
      return computeExpectedLinearVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture, worldFrame, worldFrame, bodyFixedPoint);
   }

   public static FrameVector3D computeExpectedLinearVelocityByFiniteDifference(double dt, ReferenceFrame bodyFrame, ReferenceFrame bodyFrameInFuture,
                                                                               ReferenceFrame baseFrame, ReferenceFrame baseFrameInFuture,
                                                                               Point3D bodyFixedPoint)
   {
      FramePoint3D point = new FramePoint3D(bodyFrame, bodyFixedPoint);
      FramePoint3D pointInFuture = new FramePoint3D(bodyFrameInFuture, bodyFixedPoint);
      point.changeFrame(baseFrame);
      pointInFuture.changeFrame(baseFrameInFuture);

      FrameVector3D pointLinearVelocity = new FrameVector3D(baseFrame);
      pointLinearVelocity.sub((Point3DReadOnly) pointInFuture, point);
      pointLinearVelocity.scale(1.0 / dt);
      return pointLinearVelocity;
   }

   public static Twist computeExpectedTwistByFiniteDifference(double dt, ReferenceFrame bodyFrame, ReferenceFrame bodyFrameInFuture)
   {
      Twist expectedTwist = new Twist(bodyFrame, worldFrame, bodyFrame);

      FrameVector3D bodyLinearVelocity = computeLinearVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);
      expectedTwist.getLinearPart().set(bodyLinearVelocity);

      FrameVector3D bodyAngularVelocity = computeAngularVelocityByFiniteDifference(dt, bodyFrame, bodyFrameInFuture);
      expectedTwist.getAngularPart().set(bodyAngularVelocity);
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
      relativeTwist.setIncludingFrame(bodyTwist);
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
      quaternionCalculus.computeQDotByFiniteDifferenceCentral(bodyOrientation, bodyOrientationInFuture, 0.5 * dt, qDot);
      quaternionCalculus.computeAngularVelocityInWorldFrame(bodyOrientation, qDot, bodyAngularVelocity);

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
      List<RevoluteJoint> randomChainRobot = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
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
