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
import us.ihmc.robotics.referenceFrames.ZUpFrame;

public class MovingZUpFrameTest
{
   @Test(timeout = 30000)
   public void testAgainstFiniteDifferenceWithChainRobot()
   {
      Random random = new Random(3452345L);
      int numberOfJoints = 20;
      double updateDT = 1.0e-8;

      List<OneDoFJoint> joints = ScrewTestTools.createRandomChainRobotWithOneDoFJoints(numberOfJoints, random);

      Map<MovingZUpFrame, NumericalMovingReferenceFrame> jointFramesToFDFrames = new HashMap<>();
      for (OneDoFJoint joint : joints)
      {
         MovingReferenceFrame frameAfterJoint = joint.getFrameAfterJoint();

         MovingZUpFrame zupFrameAfterJoint = new MovingZUpFrame(frameAfterJoint, frameAfterJoint.getName() + "ZUp");

         NumericalMovingReferenceFrame frameAfterJointFD = new NumericalMovingReferenceFrame(zupFrameAfterJoint, updateDT);
         jointFramesToFDFrames.put(zupFrameAfterJoint, frameAfterJointFD);
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

         Set<Entry<MovingZUpFrame, NumericalMovingReferenceFrame>> entrySet = jointFramesToFDFrames.entrySet();
         for (Entry<MovingZUpFrame, NumericalMovingReferenceFrame> entry : entrySet)
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
   public void testConsistencyWithZUpFrameWithChainRobot()
   {
      Random random = new Random(3452345L);
      int numberOfJoints = 20;

      List<OneDoFJoint> joints = ScrewTestTools.createRandomChainRobotWithOneDoFJoints(numberOfJoints, random);

      Map<ZUpFrame, MovingZUpFrame> zUpFramesToMovingZUpFrames = new HashMap<>();
      for (OneDoFJoint joint : joints)
      {
         MovingReferenceFrame frameAfterJoint = joint.getFrameAfterJoint();

         ZUpFrame zUpFrameAfterJoint = new ZUpFrame(frameAfterJoint.getRootFrame(), frameAfterJoint, frameAfterJoint.getName() + "ZUp");
         MovingZUpFrame movingZUpFrameAfterJoint = new MovingZUpFrame(frameAfterJoint, frameAfterJoint.getName() + "ZUp");

         zUpFramesToMovingZUpFrames.put(zUpFrameAfterJoint, movingZUpFrameAfterJoint);
      }

      RigidBodyTransform actualTransform = new RigidBodyTransform();
      RigidBodyTransform expectedTransform = new RigidBodyTransform();

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.setRandomPositions(joints, random);
         joints.get(0).getPredecessor().updateFramesRecursively();
         zUpFramesToMovingZUpFrames.keySet().forEach(ReferenceFrame::update);
         zUpFramesToMovingZUpFrames.values().forEach(ReferenceFrame::update);

         Set<Entry<ZUpFrame, MovingZUpFrame>> entrySet = zUpFramesToMovingZUpFrames.entrySet();
         for (Entry<ZUpFrame, MovingZUpFrame> entry : entrySet)
         {
            expectedTransform.set(entry.getKey().getTransformToRoot());
            actualTransform.set(entry.getValue().getTransformToRoot());

            EuclidCoreTestTools.assertRigidBodyTransformEquals(expectedTransform, actualTransform, 1.0e-12);
         }
      }
   }
}
