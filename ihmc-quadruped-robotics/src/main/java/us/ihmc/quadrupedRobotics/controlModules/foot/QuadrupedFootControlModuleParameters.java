package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class QuadrupedFootControlModuleParameters
{
   // final registry
   private final YoVariableRegistry finalRegistry = new YoVariableRegistry("QuadrupedFootControlModule");

   private static final int defaultTouchdownTriggerWindow = 1;

   // final parameters
   private final ParameterizedPID3DGains solePositionGainsVMC;
   private final ParameterizedPID3DGains solePositionGainsIK;

   private final DoubleParameter touchdownPressureLimitParameter = new DoubleParameter("touchdownPressureLimit", finalRegistry, 50);
   private final IntegerParameter touchdownTriggerWindowParameter = new IntegerParameter("touchdownTriggerWindow", finalRegistry,
                                                                                         defaultTouchdownTriggerWindow);
   private final DoubleParameter minimumStepAdjustmentTimeParameter = new DoubleParameter("minimumStepAdjustmentTime", finalRegistry, 0.1);
   private final DoubleParameter stepGoalOffsetZParameter = new DoubleParameter("stepGoalOffsetZ", finalRegistry, 0.0);

   private final YoBoolean isPositionControlled;

   public QuadrupedFootControlModuleParameters(YoBoolean isPositionControlled)
   {
      this.isPositionControlled = isPositionControlled;

      DefaultPID3DGains solePositionVMCGains = new DefaultPID3DGains();
      solePositionVMCGains.setProportionalGains(10000.0, 10000.0, 5000.0);
      solePositionVMCGains.setDerivativeGains(200.0, 200.0, 200.0);
      solePositionVMCGains.setIntegralGains(0.0, 0.0, 0.0, 0.0);
      solePositionGainsVMC = new ParameterizedPID3DGains("_solePositionVMC", GainCoupling.NONE, false, solePositionVMCGains, finalRegistry);

      DefaultPID3DGains solePositionIKGains = new DefaultPID3DGains();
      solePositionIKGains.setProportionalGains(100.0, 100.0, 50.0);
      solePositionIKGains.setDerivativeGains(10.0, 10.0, 50.0);
      solePositionIKGains.setIntegralGains(0.0, 0.0, 0.0, 0.0);
      solePositionGainsIK = new ParameterizedPID3DGains("_solePositionIK", GainCoupling.NONE, false, solePositionIKGains, finalRegistry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return finalRegistry;
   }

   public PID3DGainsReadOnly getSolePositionGains()
   {
      if (isPositionControlled.getBooleanValue())
         return solePositionGainsIK;
      else
         return solePositionGainsVMC;
   }

   public double getTouchdownPressureLimitParameter()
   {
      return touchdownPressureLimitParameter.getValue();
   }

   public int getTouchdownTriggerWindowParameter()
   {
      return touchdownTriggerWindowParameter.getValue();
   }

   public double getMinimumStepAdjustmentTimeParameter()
   {
      return minimumStepAdjustmentTimeParameter.getValue();
   }

   public double getStepGoalOffsetZParameter()
   {
      return stepGoalOffsetZParameter.getValue();
   }

   public static int getDefaultTouchdownTriggerWindow()
   {
      return defaultTouchdownTriggerWindow;
   }
}
