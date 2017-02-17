package us.ihmc.commonWalkingControlModules.configurations;

import java.util.Map;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.partNames.ArmJointName;
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

   /**
    * Parameter used for position controlled joints, does not affect force controlled joints.
    * If empty or a parameter is missing for a joint, a default will be used.
    * Alpha position should be between 0 and 1 inclusive, it refers to how fast the integrated desired position is leaking towards the actual joint position.
    * 1 == pure integration and 0 == desired position equals to the actual position.
    * @param registry
    * @return
    */
   public Map<ArmJointName, DoubleYoVariable> getOrCreateAccelerationIntegrationAlphaPosition(YoVariableRegistry registry)
   {
      return null;
   }

   /**
    * Parameter used for position controlled joints, does not affect force controlled joints.
    * If empty or a parameter is missing for a joint, a default will be used.
    * Alpha velocity should be between 0 and 1 inclusive, it refers to how fast the integrated desired velocity is leaking towards zero velocity (it acts as a damping).
    * 1 == pure integration and 0 == desired velocity equals to zero.
    * @param registry
    * @return
    */
   public Map<ArmJointName, DoubleYoVariable> getOrCreateAccelerationIntegrationAlphaVelocity(YoVariableRegistry registry)
   {
      return null;
   }

   /**
    * Parameter used for position controlled joints, does not affect force controlled joints.
    * If empty or a parameter is missing for a joint, a default will be used.
    * Max position error should be positive. It prevents the integration to wind up.
    * @param registry
    * @return
    */
   public Map<ArmJointName, DoubleYoVariable> getOrCreateAccelerationIntegrationMaxPositionError(YoVariableRegistry registry)
   {
      return null;
   }

   /**
    * Parameter used for position controlled joints, does not affect force controlled joints.
    * If empty or a parameter is missing for a joint, a default will be used.
    * Limits the maximum desired integrated velocity, it has to be positive. Very useful for tuning other parameters/gains.
    * @param registry
    * @return
    */
   public Map<ArmJointName, DoubleYoVariable> getOrCreateAccelerationIntegrationMaxVelocity(YoVariableRegistry registry)
   {
      return null;
   }

   public abstract Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide);
}
