package us.ihmc.robotics.screwTheory;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.Set;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;

public class MovingMidFootZUpGroundFrameTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testAgainstFiniteDifferenceWithChainRobot()
   {
      Random random = new Random(3452345L);
      int numberOfJoints = 20;
      double updateDT = 1.0e-8;
      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(updateDT);

      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);

      Map<MovingMidFootZUpGroundFrame, NumericalMovingReferenceFrame> jointFramesToFDFrames = new HashMap<>();
      for (int i = 0; i < 5; i++)
      {
         int firstJointIndex = random.nextInt(numberOfJoints);
         int secondJointIndex = random.nextInt(numberOfJoints - 1) + firstJointIndex;
         secondJointIndex %= numberOfJoints;

         MovingReferenceFrame nonZUpFrameOne = joints.get(firstJointIndex).getFrameAfterJoint();
         MovingReferenceFrame nonZUpFrameTwo = joints.get(secondJointIndex).getFrameAfterJoint();
         MovingZUpFrame frameOne = new MovingZUpFrame(nonZUpFrameOne, nonZUpFrameOne.getName() + "ZUp");
         MovingZUpFrame frameTwo = new MovingZUpFrame(nonZUpFrameTwo, nonZUpFrameTwo.getName() + "ZUp");

         MovingMidFootZUpGroundFrame movingMidFrameZUpFrame = new MovingMidFootZUpGroundFrame("midFrame" + i, frameOne, frameTwo);

         NumericalMovingReferenceFrame frameFD = new NumericalMovingReferenceFrame(movingMidFrameZUpFrame, updateDT);
         jointFramesToFDFrames.put(movingMidFrameZUpFrame, frameFD);
      }

      Twist actualTwist = new Twist();
      Twist expectedTwist = new Twist();

      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      joints.get(0).getPredecessor().updateFramesRecursively();

      jointFramesToFDFrames.keySet().forEach(MovingReferenceFrame::update);
      jointFramesToFDFrames.values().forEach(MovingReferenceFrame::update);

      for (int i = 0; i < 100; i++)
      {
         integrator.integrateFromVelocity(joints);
         joints.get(0).getPredecessor().updateFramesRecursively();
         jointFramesToFDFrames.keySet().forEach(MovingReferenceFrame::update);
         jointFramesToFDFrames.values().forEach(MovingReferenceFrame::update);

         Set<Entry<MovingMidFootZUpGroundFrame, NumericalMovingReferenceFrame>> entrySet = jointFramesToFDFrames.entrySet();
         for (Entry<MovingMidFootZUpGroundFrame, NumericalMovingReferenceFrame> entry : entrySet)
         {
            entry.getKey().getTwistOfFrame(expectedTwist);
            entry.getValue().getTwistOfFrame(actualTwist);
            expectedTwist.setBodyFrame(entry.getValue());
            expectedTwist.changeFrame(entry.getValue());

            MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);
         }
      }
   }

   @Test
   public void testConsistencyWithMidFrameZUpFrameWithChainRobot()
   {
      Random random = new Random(3452345L);
      int numberOfJoints = 20;

      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);

      Map<MidFootZUpGroundFrame, MovingMidFootZUpGroundFrame> zUpFramesToMovingZUpFrames = new HashMap<>();
      for (int i = 0; i < 5; i++)
      {
         int firstJointIndex = random.nextInt(numberOfJoints);
         int secondJointIndex = random.nextInt(numberOfJoints - 1) + firstJointIndex;
         secondJointIndex %= numberOfJoints;

         MovingReferenceFrame nonZUpFrameOne = joints.get(firstJointIndex).getFrameAfterJoint();
         MovingReferenceFrame nonZUpFrameTwo = joints.get(secondJointIndex).getFrameAfterJoint();
         MovingZUpFrame frameOne = new MovingZUpFrame(nonZUpFrameOne, nonZUpFrameOne.getName() + "ZUp");
         MovingZUpFrame frameTwo = new MovingZUpFrame(nonZUpFrameTwo, nonZUpFrameTwo.getName() + "ZUp");

         MovingMidFootZUpGroundFrame movingMidFrameZUpFrame = new MovingMidFootZUpGroundFrame("midFrame" + i, frameOne, frameTwo);
         MidFootZUpGroundFrame midFrameZUpFrame = new MidFootZUpGroundFrame("midFrame" + i, frameOne, frameTwo);

         zUpFramesToMovingZUpFrames.put(midFrameZUpFrame, movingMidFrameZUpFrame);
      }

      RigidBodyTransform actualTransform = new RigidBodyTransform();
      RigidBodyTransform expectedTransform = new RigidBodyTransform();

      for (int i = 0; i < 100; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
         joints.get(0).getPredecessor().updateFramesRecursively();
         zUpFramesToMovingZUpFrames.keySet().forEach(ReferenceFrame::update);
         zUpFramesToMovingZUpFrames.values().forEach(ReferenceFrame::update);

         Set<Entry<MidFootZUpGroundFrame, MovingMidFootZUpGroundFrame>> entrySet = zUpFramesToMovingZUpFrames.entrySet();
         for (Entry<MidFootZUpGroundFrame, MovingMidFootZUpGroundFrame> entry : entrySet)
         {
            expectedTransform.set(entry.getKey().getTransformToRoot());
            actualTransform.set(entry.getValue().getTransformToRoot());

            EuclidCoreTestTools.assertEquals(expectedTransform, actualTransform, 1.0e-12);
         }
      }
   }
}
