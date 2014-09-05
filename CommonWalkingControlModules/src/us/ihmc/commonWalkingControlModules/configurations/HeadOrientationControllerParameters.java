package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.util.controller.YoOrientationPIDGains;

public interface HeadOrientationControllerParameters
{
   public abstract String[] getDefaultHeadOrientationControlJointNames();

   public abstract YoOrientationPIDGains createHeadOrientationControlGains(YoVariableRegistry registry);

   public abstract double[] getInitialHeadYawPitchRoll();

   public abstract String getJointNameForExtendedPitchRange();

   public abstract double getUpperNeckPitchLimit();

   public abstract double getLowerNeckPitchLimit();

   public abstract double getHeadYawLimit();

   public abstract double getHeadRollLimit();
}
