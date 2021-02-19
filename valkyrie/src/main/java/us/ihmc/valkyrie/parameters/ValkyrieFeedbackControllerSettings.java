package us.ihmc.valkyrie.parameters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.robotics.dataStructures.parameters.GroupParameter;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class ValkyrieFeedbackControllerSettings implements FeedbackControllerSettings
{
   private final ValkyrieJointMap jointMap;
   private final RobotTarget target;

   public ValkyrieFeedbackControllerSettings(ValkyrieJointMap jointMap, RobotTarget target)
   {
      this.jointMap = jointMap;
      this.target = target;
   }

   @Override
   public boolean enableIntegralTerm()
   {
      return false; // Saves about 130 YoVariables.
   }

   @Override
   public List<GroupParameter<Double>> getErrorVelocityFilterBreakFrequencies()
   {
      if (target == RobotTarget.SCS)
         return null;

      GroupParameter<Double> pelvisGroup = new GroupParameter<>(jointMap.getPelvisName(), 25.0);
      GroupParameter<Double> chestGroup = new GroupParameter<>(jointMap.getChestName(), 16.0);
      GroupParameter<Double> footGroup = new GroupParameter<>("foot", 16.0, new ArrayList<>());
      GroupParameter<Double> armJointsGroup = new GroupParameter<>("armJoints", 25.0, new ArrayList<>());

      for (RobotSide robotSide : RobotSide.values)
      {
         footGroup.getMemberNames().add(jointMap.getFootName(robotSide));

         for (ArmJointName armJointName : jointMap.getArmJointNames())
            armJointsGroup.getMemberNames().add(jointMap.getArmJointName(robotSide, armJointName));
      }

      return Arrays.asList(pelvisGroup, chestGroup, footGroup, armJointsGroup);
   }
}
