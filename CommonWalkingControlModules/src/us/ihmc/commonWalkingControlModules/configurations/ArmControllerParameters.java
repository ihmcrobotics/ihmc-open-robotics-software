package us.ihmc.commonWalkingControlModules.configurations;

import java.util.Map;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.YoVariableRegistry;

import com.yobotics.simulationconstructionset.util.controller.YoPIDGains;
import com.yobotics.simulationconstructionset.util.controller.YoSE3PIDGains;

public interface ArmControllerParameters
{
   public abstract YoPIDGains createJointspaceControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGains createTaskspaceControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGains createTaskspaceControlGainsForLoadBearing(YoVariableRegistry registry);

   public abstract boolean useInverseKinematicsTaskspaceControl();

   public abstract boolean doLowLevelPositionControl();

   public abstract Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide);
}
