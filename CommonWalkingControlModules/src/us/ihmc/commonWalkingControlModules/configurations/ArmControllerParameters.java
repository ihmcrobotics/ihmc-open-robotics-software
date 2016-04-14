package us.ihmc.commonWalkingControlModules.configurations;

import java.util.Map;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public abstract class ArmControllerParameters
{
   public abstract YoPIDGains createJointspaceControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGainsInterface createTaskspaceControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGainsInterface createTaskspaceControlGainsForLoadBearing(YoVariableRegistry registry);

   /**
    * Override this method to specify arm joints that should be position controlled.
    * @param robotSide
    * @return
    */
   public String[] getPositionControlledJointNames(RobotSide robotSide)
   {
      return null;
   }

   public abstract Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide);
}
