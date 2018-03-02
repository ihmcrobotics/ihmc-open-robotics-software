package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.Axis;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedFootControlModuleParameters
{
   // final registry
   private final YoVariableRegistry finalRegistry = new YoVariableRegistry("QuadrupedFootControlModule");

   private static final int defaultTouchdownTriggerWindow = 1;

   // final parameters
   private final DoubleParameter[] solePositionProportionalGainsParameter = new DoubleParameter[3];
   private final DoubleParameter[] solePositionDerivativeGainsParameter = new DoubleParameter[3];
   private final DoubleParameter[] solePositionIntegralGainsParameter = new DoubleParameter[3];
   private final DoubleParameter solePositionMaxIntegralErrorParameter = new DoubleParameter("solePositionMaxIntegralError", finalRegistry, 0);
   private final DoubleParameter touchdownPressureLimitParameter = new DoubleParameter("touchdownPressureLimit", finalRegistry, 50);
   private final IntegerParameter touchdownTriggerWindowParameter = new IntegerParameter("touchdownTriggerWindow", finalRegistry, defaultTouchdownTriggerWindow);
   private final DoubleParameter minimumStepAdjustmentTimeParameter = new DoubleParameter("minimumStepAdjustmentTime", finalRegistry, 0.1);
   private final DoubleParameter stepGoalOffsetZParameter = new DoubleParameter("stepGoalOffsetZ", finalRegistry, 0.0);

   private final double[] solePositionProportionalGains = new double[3];
   private final double[] solePositionDerivativeGains = new double[3];
   private final double[] solePositionIntegralGains = new double[3];

   public QuadrupedFootControlModuleParameters()
   {
      for (int i = 0; i < 3; i++)
      {
         double solePositionProportionalGain = (i == 2) ? 5000.0 : 10000.0;
         solePositionProportionalGainsParameter[i] = new DoubleParameter("solePositionProportionalGain" + Axis.values[i], finalRegistry, solePositionProportionalGain);
         solePositionDerivativeGainsParameter[i] = new DoubleParameter("solePositionDerivativeGain" + Axis.values[i], finalRegistry, 200.0);
         solePositionIntegralGainsParameter[i] = new DoubleParameter("solePositionIntegralGain" + Axis.values[i], finalRegistry, 0.0);
      }
   }
   
   
   public YoVariableRegistry getYoVariableRegistry()
   {
      return finalRegistry;
   }



   public double[] getSolePositionProportionalGainsParameter()
   {
      updateGains(solePositionProportionalGainsParameter, solePositionProportionalGains);
      return solePositionProportionalGains;
   }


   public double[] getSolePositionDerivativeGainsParameter()
   {
      updateGains(solePositionDerivativeGainsParameter, solePositionDerivativeGains);
      return solePositionDerivativeGains;
   }


   public double[] getSolePositionIntegralGainsParameter()
   {
      updateGains(solePositionIntegralGainsParameter, solePositionIntegralGains);
      return solePositionIntegralGains;
   }


   public double getSolePositionMaxIntegralErrorParameter()
   {
      return solePositionMaxIntegralErrorParameter.getValue();
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
   
   private static void updateGains(DoubleParameter[] parameters, double[] values)
   {
      for (int i = 0; i < parameters.length; i++)
      {
         values[i] = parameters[i].getValue();
      }
   }

   public static int getDefaultTouchdownTriggerWindow()
   {
      return defaultTouchdownTriggerWindow;
   }
}
