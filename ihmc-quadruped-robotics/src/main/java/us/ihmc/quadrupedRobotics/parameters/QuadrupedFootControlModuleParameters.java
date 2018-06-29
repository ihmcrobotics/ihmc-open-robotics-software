package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuadrupedFootControlModuleParameters
{
   // final registry
   private final YoVariableRegistry finalRegistry = new YoVariableRegistry("QuadrupedFootControlModule");

   private static final int defaultTouchdownTriggerWindow = 1;

   // final parameters
   private final ParameterizedPID3DGains solePositionGainsVMC;
   private final ParameterizedPID3DGains solePositionGainsIK;

   private final ParameterVector3D solePositionWeightsVMC = new ParameterVector3D("solePositionWeightVMC_", new Vector3D(), finalRegistry);
   private final ParameterVector3D solePositionWeightsIK = new ParameterVector3D("solePositionWeightIK_", new Vector3D(50.0, 50.0, 10.0), finalRegistry);

   private final DoubleParameter touchdownPressureLimitParameter = new DoubleParameter("touchdownPressureLimit", finalRegistry, 50);
   private final IntegerParameter touchdownTriggerWindowParameter = new IntegerParameter("touchdownTriggerWindow", finalRegistry,
                                                                                         defaultTouchdownTriggerWindow);
   private final DoubleParameter minimumStepAdjustmentTimeParameter = new DoubleParameter("minimumStepAdjustmentTime", finalRegistry, 0.1);
   private final DoubleParameter stepGoalOffsetZParameter = new DoubleParameter("stepGoalOffsetZ", finalRegistry, 0.0);

   private final YoEnum<WholeBodyControllerCoreMode> controllerCoreMode;

   public QuadrupedFootControlModuleParameters(YoEnum<WholeBodyControllerCoreMode> controllerCoreMode)
   {
      this.controllerCoreMode = controllerCoreMode;

      DefaultPID3DGains solePositionVMCGains = new DefaultPID3DGains();
      solePositionVMCGains.setProportionalGains(10000.0, 10000.0, 5000.0);
      solePositionVMCGains.setDerivativeGains(200.0, 200.0, 200.0);
      solePositionVMCGains.setIntegralGains(0.0, 0.0, 0.0, 0.0);
      solePositionGainsVMC = new ParameterizedPID3DGains("_solePositionVMC", GainCoupling.NONE, false, solePositionVMCGains, finalRegistry);

      DefaultPID3DGains solePositionIKGains = new DefaultPID3DGains();
      solePositionIKGains.setProportionalGains(50.0, 50.0, 10.0);
      solePositionIKGains.setDerivativeGains(10.0, 10.0, 1.0);
      solePositionIKGains.setIntegralGains(0.0, 0.0, 0.0, 0.0);
      solePositionGainsIK = new ParameterizedPID3DGains("_solePositionIK", GainCoupling.NONE, false, solePositionIKGains, finalRegistry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return finalRegistry;
   }

   public PID3DGainsReadOnly getSolePositionGains()
   {
      switch (controllerCoreMode.getEnumValue())
      {
      case VIRTUAL_MODEL:
         return solePositionGainsVMC;
      case INVERSE_KINEMATICS:
         return solePositionGainsIK;
      default:
         throw new RuntimeException("The controller core mode " + controllerCoreMode.getEnumValue() + " does not have foot control gains.");
      }
   }

   public ParameterVector3D getSolePositionWeights()
   {
      switch (controllerCoreMode.getEnumValue())
      {
         case VIRTUAL_MODEL:
            return solePositionWeightsVMC;
         case INVERSE_KINEMATICS:
            return solePositionWeightsIK;
         default:
            throw new RuntimeException("The controller core mode " + controllerCoreMode.getEnumValue() + " does not have foot control weights.");
      }
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
