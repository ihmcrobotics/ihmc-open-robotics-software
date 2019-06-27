package us.ihmc.robotModels;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public class FullRobotModelUtils
{
   public static OneDoFJointBasics[] getAllJointsExcludingHands(FullHumanoidRobotModel model)
   {
      ArrayList<OneDoFJointBasics> joints = new ArrayList<>();
      getAllJointsExcludingHands(joints, model);
      return joints.toArray(new OneDoFJointBasics[joints.size()]);
   }

   public static void getAllJointsExcludingHands(ArrayList<OneDoFJointBasics> jointsToPack, FullHumanoidRobotModel model)
   {
      model.getOneDoFJoints(jointsToPack);
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = model.getHand(robotSide);
         if (hand != null)
         {
            OneDoFJointBasics[] fingerJoints = MultiBodySystemTools.filterJoints(MultiBodySystemTools.collectSubtreeJoints(hand), OneDoFJointBasics.class);
            for (OneDoFJointBasics fingerJoint : fingerJoints)
            {
               jointsToPack.remove(fingerJoint);
            }
         }
      }
   }

   public static void checkJointNameHash(int expectedJointNameHash, int actualJointNameHash)
   {
      if (expectedJointNameHash != actualJointNameHash)
      {
         throw new RuntimeException(String.format("Joint names do not match. expected: %s actual: %s", expectedJointNameHash, actualJointNameHash));
      }
   }
}
