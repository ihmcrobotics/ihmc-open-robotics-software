package us.ihmc.commonWalkingControlModules.configurations;

import java.util.Map;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.controllers.YoPIDGains;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;


public interface ArmControllerParameters
{
   public abstract YoPIDGains createJointspaceControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGains createTaskspaceControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGains createTaskspaceControlGainsForLoadBearing(YoVariableRegistry registry);

   public abstract boolean useInverseKinematicsTaskspaceControl();

   public abstract boolean doLowLevelPositionControl();

   public abstract Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide);
   
   public abstract double getWristHandCenterOffset();
}
