package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class HighLevelHumanoidControllerFactoryHelper
{
   public static InverseDynamicsJoint[] computeJointsToOptimizeFor(FullHumanoidRobotModel fullRobotModel, InverseDynamicsJoint... jointsToRemove)
   {
      List<InverseDynamicsJoint> joints = new ArrayList<InverseDynamicsJoint>();
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      joints.addAll(Arrays.asList(allJoints));

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = fullRobotModel.getHand(robotSide);
         if (hand != null)
         {
            List<InverseDynamicsJoint> fingerJoints = Arrays.asList(ScrewTools.computeSubtreeJoints(hand));
            joints.removeAll(fingerJoints);
         }
      }

      if (jointsToRemove != null)
      {
         for (InverseDynamicsJoint joint : jointsToRemove)
         {
            joints.remove(joint);
         }
      }

      return joints.toArray(new InverseDynamicsJoint[joints.size()]);
   }
}
