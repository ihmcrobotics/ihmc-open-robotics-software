package us.ihmc.commonWalkingControlModules.configurations;

import java.util.Map;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.controller.YoPIDGains;
import com.yobotics.simulationconstructionset.util.controller.YoSE3PIDGains;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public interface ArmControllerParameters
{
   public abstract YoPIDGains createJointspaceControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGains createTaskspaceControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGains createTaskspaceControlGainsForLoadBearing(YoVariableRegistry registry);

   public abstract boolean useInverseKinematicsTaskspaceControl();

   public abstract boolean doLowLevelPositionControl();

   public abstract Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide);
}
