package us.ihmc.commonWalkingControlModules.configurations;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class NoArmsArmControllerParameters extends ArmControllerParameters
{
   @Override
   public YoPIDGains createJointspaceControlGains(YoVariableRegistry registry)
   {
      return null;
   }

   @Override
   public YoSE3PIDGainsInterface createTaskspaceControlGains(YoVariableRegistry registry)
   {
      return null;
   }

   @Override
   public YoSE3PIDGainsInterface createTaskspaceControlGainsForLoadBearing(YoVariableRegistry registry)
   {
      return null;
   }

   @Override
   public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide)
   {
      return new LinkedHashMap<OneDoFJoint, Double>();
   }
}
