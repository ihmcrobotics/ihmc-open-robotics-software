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
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;

public class NumericalMovingReferenceFrameTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testWithChainRobot()
   {
      Random random = new Random(3452345L);
      int numberOfJoints = 20;
      double updateDT = 1.0e-8;
      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(updateDT);

      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);

      Map<MovingReferenceFrame, NumericalMovingReferenceFrame> jointFramesToFDFrames = new HashMap<>();
      for (OneDoFJointBasics joint : joints)
      {
         MovingReferenceFrame frameBeforeJoint = joint.getFrameBeforeJoint();
         NumericalMovingReferenceFrame frameBeforeJointFD = new NumericalMovingReferenceFrame(frameBeforeJoint, updateDT);
         jointFramesToFDFrames.put(frameBeforeJoint, frameBeforeJointFD);

         MovingReferenceFrame frameAfterJoint = joint.getFrameAfterJoint();
         NumericalMovingReferenceFrame frameAfterJointFD = new NumericalMovingReferenceFrame(frameAfterJoint, updateDT);
         jointFramesToFDFrames.put(frameAfterJoint, frameAfterJointFD);
      }

      Twist actualTwist = new Twist();
      Twist expectedTwist = new Twist();

      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      joints.get(0).getPredecessor().updateFramesRecursively();

      jointFramesToFDFrames.values().forEach(NumericalMovingReferenceFrame::update);

      for (int i = 0; i < 100; i++)
      {
         integrator.integrateFromVelocity(joints);
         joints.get(0).getPredecessor().updateFramesRecursively();

         for (NumericalMovingReferenceFrame fdFrame : jointFramesToFDFrames.values())
            fdFrame.update();

         Set<Entry<MovingReferenceFrame, NumericalMovingReferenceFrame>> entrySet = jointFramesToFDFrames.entrySet();
         for (Entry<MovingReferenceFrame, NumericalMovingReferenceFrame> entry : entrySet)
         {
            entry.getKey().getTwistOfFrame(expectedTwist);
            entry.getValue().getTwistOfFrame(actualTwist);
            expectedTwist.setBodyFrame(entry.getValue());
            expectedTwist.changeFrame(entry.getValue());

            MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, 1.0e-5);
         }
      }
   }
}
