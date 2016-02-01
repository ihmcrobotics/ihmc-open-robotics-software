package us.ihmc.SdfLoader.models;

import java.util.ArrayList;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
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
         RigidBody hand = model.getHand(robotSide);
         if (hand != null)
         {
            OneDoFJoint[] fingerJoints = ScrewTools.filterJoints(ScrewTools.computeSubtreeJoints(hand), OneDoFJoint.class);
            for (OneDoFJoint fingerJoint : fingerJoints)
            {
               jointsToPack.remove(fingerJoint);
            }
         }
      }
   }
}
