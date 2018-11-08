package us.ihmc.robotModels;

import java.util.ArrayList;

import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class FullRobotModelUtils
{
   public static OneDoFJoint[] getAllJointsExcludingHands(FullHumanoidRobotModel model)
   {
      ArrayList<OneDoFJoint> joints = new ArrayList<>();
      getAllJointsExcludingHands(joints, model);
      return joints.toArray(new OneDoFJoint[joints.size()]);
   }
   public static void getAllJointsExcludingHands(ArrayList<OneDoFJoint> jointsToPack, FullHumanoidRobotModel model)
   {
      model.getOneDoFJoints(jointsToPack);
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = model.getHand(robotSide);
         if (hand != null)
         {
            OneDoFJoint[] fingerJoints = MultiBodySystemTools.filterJoints(ScrewTools.computeSubtreeJoints(hand), OneDoFJoint.class);
            for (OneDoFJoint fingerJoint : fingerJoints)
            {
               jointsToPack.remove(fingerJoint);
            }
         }
      }
   }
}
