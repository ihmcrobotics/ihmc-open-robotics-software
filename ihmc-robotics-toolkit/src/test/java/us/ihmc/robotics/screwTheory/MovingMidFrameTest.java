package us.ihmc.robotics.screwTheory;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.Set;

import org.junit.Test;

public class MovingMidFrameTest
{
   @Test(timeout = 30000)
   public void testAgainstFiniteDifferenceWithChainRobot()
   {
      Random random = new Random(3452345L);
      int numberOfJoints = 20;
      double updateDT = 1.0e-8;

      List<OneDoFJoint> joints = ScrewTestTools.createRandomChainRobotWithOneDoFJoints(numberOfJoints, random);

      Map<MovingMidFrame, NumericalMovingReferenceFrame> jointFramesToFDFrames = new HashMap<>();
      for (int i = 0; i < 5; i++)
      {
         int firstJointIndex = random.nextInt(numberOfJoints);
         int secondJointIndex = random.nextInt(numberOfJoints - 1) + firstJointIndex;
         secondJointIndex %= numberOfJoints;

         MovingReferenceFrame frameOne = joints.get(firstJointIndex).getFrameAfterJoint();
         MovingReferenceFrame frameTwo = joints.get(secondJointIndex).getFrameAfterJoint();

         MovingMidFrame movingMidFrameZUpFrame = new MovingMidFrame("midFrame" + i, frameOne, frameTwo);

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

         Set<Entry<MovingMidFrame, NumericalMovingReferenceFrame>> entrySet = jointFramesToFDFrames.entrySet();
         for (Entry<MovingMidFrame, NumericalMovingReferenceFrame> entry : entrySet)
         {
            entry.getKey().getTwistOfFrame(expectedTwist);
            entry.getValue().getTwistOfFrame(actualTwist);
            expectedTwist.changeBodyFrameNoRelativeTwist(entry.getValue());
            expectedTwist.changeFrame(entry.getValue());

            TwistCalculatorTest.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);
         }
      }
   }
}
