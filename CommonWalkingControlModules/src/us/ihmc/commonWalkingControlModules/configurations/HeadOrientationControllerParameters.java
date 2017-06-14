package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface HeadOrientationControllerParameters
{
   public abstract String[] getDefaultHeadOrientationControlJointNames();

   public abstract YoOrientationPIDGainsInterface createHeadOrientationControlGains(YoVariableRegistry registry);

   public abstract YoPIDGains createHeadJointspaceControlGains(YoVariableRegistry registry);

   public abstract double[] getInitialHeadYawPitchRoll();

   public abstract boolean isNeckPositionControlled();

   public abstract double getNeckPitchUpperLimit();

   public abstract double getNeckPitchLowerLimit();

   public abstract double getHeadYawLimit();

   public abstract double getHeadRollLimit();

   public abstract double getTrajectoryTimeHeadOrientation();
}
