package us.ihmc.robotics.screwTheory;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.Set;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;

public class MovingMidFrameZUpFrameTest
{
   @Test(timeout = 30000)
   public void testAgainstFiniteDifferenceWithChainRobot()
   {
      Random random = new Random(3452345L);
      int numberOfJoints = 20;
      double updateDT = 1.0e-8;

      List<OneDoFJoint> joints = ScrewTestTools.createRandomChainRobotWithOneDoFJoints(numberOfJoints, random);

      Map<MovingMidFrameZUpFrame, NumericalMovingReferenceFrame> jointFramesToFDFrames = new HashMap<>();
      for (int i = 0; i < 5; i++)
      {
         int firstJointIndex = random.nextInt(numberOfJoints);
         int secondJointIndex = random.nextInt(numberOfJoints - 1) + firstJointIndex;
         secondJointIndex %= numberOfJoints;

         MovingReferenceFrame frameOne = joints.get(firstJointIndex).getFrameAfterJoint();
         MovingReferenceFrame frameTwo = joints.get(secondJointIndex).getFrameAfterJoint();

         MovingMidFrameZUpFrame movingMidFrameZUpFrame = new MovingMidFrameZUpFrame("midFrame" + i, frameOne, frameTwo);

         NumericalMovingReferenceFrame frameFD = new NumericalMovingReferenceFrame(movingMidFrameZUpFrame, updateDT);
         jointFramesToFDFrames.put(movingMidFrameZUpFrame, frameFD);
      }

      Twist actualTwist = new Twist();
      Twist expectedTwist = new Twist();

      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomVelocities(joints, random);
      joints.get(0).getPredecessor().updateFramesRecursively();

      jointFramesToFDFrames.keySet().forEach(MovingReferenceFrame::update);
      jointFramesToFDFrames.values().forEach(MovingReferenceFrame::update);

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.integrateVelocities(joints, updateDT);
         joints.get(0).getPredecessor().updateFramesRecursively();
         jointFramesToFDFrames.keySet().forEach(MovingReferenceFrame::update);
         jointFramesToFDFrames.values().forEach(MovingReferenceFrame::update);

         Set<Entry<MovingMidFrameZUpFrame, NumericalMovingReferenceFrame>> entrySet = jointFramesToFDFrames.entrySet();
         for (Entry<MovingMidFrameZUpFrame, NumericalMovingReferenceFrame> entry : entrySet)
         {
            entry.getKey().getTwistOfFrame(expectedTwist);
            entry.getValue().getTwistOfFrame(actualTwist);
            expectedTwist.changeBodyFrameNoRelativeTwist(entry.getValue());
            expectedTwist.changeFrame(entry.getValue());

            TwistCalculatorTest.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);
         }
      }
   }

   @Test(timeout = 30000)
   public void testConsistencyWithMidFrameZUpFrameWithChainRobot()
   {
      Random random = new Random(3452345L);
      int numberOfJoints = 20;

      List<OneDoFJoint> joints = ScrewTestTools.createRandomChainRobotWithOneDoFJoints(numberOfJoints, random);

      Map<MidFrameZUpFrame, MovingMidFrameZUpFrame> zUpFramesToMovingZUpFrames = new HashMap<>();
      for (int i = 0; i < 5; i++)
      {
         int firstJointIndex = random.nextInt(numberOfJoints);
         int secondJointIndex = random.nextInt(numberOfJoints - 1) + firstJointIndex;
         secondJointIndex %= numberOfJoints;

         MovingReferenceFrame frameOne = joints.get(firstJointIndex).getFrameAfterJoint();
         MovingReferenceFrame frameTwo = joints.get(secondJointIndex).getFrameAfterJoint();

         MovingMidFrameZUpFrame movingMidFrameZUpFrame = new MovingMidFrameZUpFrame("midFrame" + i, frameOne, frameTwo);
         MidFrameZUpFrame midFrameZUpFrame = new MidFrameZUpFrame("midFrame" + i, ReferenceFrame.getWorldFrame(), frameOne, frameTwo);

         zUpFramesToMovingZUpFrames.put(midFrameZUpFrame, movingMidFrameZUpFrame);
      }

      RigidBodyTransform actualTransform = new RigidBodyTransform();
      RigidBodyTransform expectedTransform = new RigidBodyTransform();

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(joints, random);
         joints.get(0).getPredecessor().updateFramesRecursively();
         zUpFramesToMovingZUpFrames.keySet().forEach(ReferenceFrame::update);
         zUpFramesToMovingZUpFrames.values().forEach(ReferenceFrame::update);

         Set<Entry<MidFrameZUpFrame, MovingMidFrameZUpFrame>> entrySet = zUpFramesToMovingZUpFrames.entrySet();
         for (Entry<MidFrameZUpFrame, MovingMidFrameZUpFrame> entry : entrySet)
         {
            expectedTransform.set(entry.getKey().getTransformToRoot());
            actualTransform.set(entry.getValue().getTransformToRoot());

            EuclidCoreTestTools.assertRigidBodyTransformEquals(expectedTransform, actualTransform, 1.0e-12);
         }
      }
   }
}
