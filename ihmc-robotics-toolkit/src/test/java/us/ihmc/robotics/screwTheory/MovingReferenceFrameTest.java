package us.ihmc.robotics.screwTheory;

import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class MovingReferenceFrameTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test(timeout = 30000)
   public void testAgainstTwistCalculatorWithPrismaticChainRobot() throws Exception
   {
      Random random = new Random(435423L);

      int numberOfJoints = 100;
      List<PrismaticJoint> joints = ScrewTestTools.createRandomChainRobotWithPrismaticJoints(numberOfJoints, random);

      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, joints.get(0).getPredecessor());
      Twist actualTwist = new Twist();
      Twist expectedTwist = new Twist();

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(joints, random, -1.0, 1.0);
         ScrewTestTools.setRandomVelocities(joints, random, -1.0, 1.0);
         joints.get(0).getPredecessor().updateFramesRecursively();

         twistCalculator.compute();

         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            OneDoFJoint joint = joints.get(jointIndex);
            RigidBody body = joint.getSuccessor();
            MovingReferenceFrame bodyFrame = body.getBodyFixedFrame();

            twistCalculator.getTwistOfBody(body, expectedTwist);
            bodyFrame.getTwistOfFrame(actualTwist);
            TwistCalculatorTest.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);
         }
      }
   }

   @Test(timeout = 30000)
   public void testAgainstTwistCalculatorWithChainRobot() throws Exception
   {
      Random random = new Random(435423L);

      int numberOfJoints = 100;
      List<OneDoFJoint> joints = ScrewTestTools.createRandomChainRobotWithOneDoFJoints(numberOfJoints, random);

      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, joints.get(0).getPredecessor());
      Twist actualTwist = new Twist();
      Twist expectedTwist = new Twist();

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(joints, random, -1.0, 1.0);
         ScrewTestTools.setRandomVelocities(joints, random, -1.0, 1.0);
         joints.get(0).getPredecessor().updateFramesRecursively();

         twistCalculator.compute();

         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            OneDoFJoint joint = joints.get(jointIndex);
            RigidBody body = joint.getSuccessor();
            MovingReferenceFrame bodyFrame = body.getBodyFixedFrame();

            twistCalculator.getTwistOfBody(body, expectedTwist);
            bodyFrame.getTwistOfFrame(actualTwist);
            TwistCalculatorTest.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);
         }
      }
   }

   @Test(timeout = 30000)
   public void testAgainstTwistCalculatorWithTreeRobot() throws Exception
   {
      Random random = new Random(435423L);

      int numberOfJoints = 100;
      List<OneDoFJoint> joints = ScrewTestTools.createRandomTreeRobotWithOneDoFJoints(numberOfJoints, random);

      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, joints.get(0).getPredecessor());
      Twist actualTwist = new Twist();
      Twist expectedTwist = new Twist();

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(joints, random, -1.0, 1.0);
         ScrewTestTools.setRandomVelocities(joints, random, -1.0, 1.0);
         joints.get(0).getPredecessor().updateFramesRecursively();

         twistCalculator.compute();

         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            OneDoFJoint joint = joints.get(jointIndex);
            RigidBody body = joint.getSuccessor();
            MovingReferenceFrame bodyFrame = body.getBodyFixedFrame();

            twistCalculator.getTwistOfBody(body, expectedTwist);
            bodyFrame.getTwistOfFrame(actualTwist);
            TwistCalculatorTest.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);

            twistCalculator.getTwistOfBody(body, expectedTwist);
            expectedTwist.changeBodyFrameNoRelativeTwist(joint.getFrameAfterJoint());
            expectedTwist.changeFrame(joint.getFrameAfterJoint());
            joint.getFrameAfterJoint().getTwistOfFrame(actualTwist);
            TwistCalculatorTest.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);

            for (InverseDynamicsJoint child : body.getChildrenJoints())
            {
               twistCalculator.getTwistOfBody(body, expectedTwist);
               expectedTwist.changeBodyFrameNoRelativeTwist(child.getFrameBeforeJoint());
               expectedTwist.changeFrame(child.getFrameBeforeJoint());
               child.getFrameBeforeJoint().getTwistOfFrame(actualTwist);
               TwistCalculatorTest.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);
            }
         }
      }
   }
}
