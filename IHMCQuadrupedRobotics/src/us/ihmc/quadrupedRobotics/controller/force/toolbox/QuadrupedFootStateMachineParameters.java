package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.IntegerParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class QuadrupedFootStateMachineParameters
{
   // final registry
   private final YoVariableRegistry finalRegistry = new YoVariableRegistry("QuadrupedFootStateMachine");

   // final parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(QuadrupedFootStateMachine.class, finalRegistry);
   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory
         .createDoubleArray("solePositionProportionalGains", 10000, 10000, 5000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 200, 200, 200);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);
   private final DoubleParameter touchdownPressureLimitParameter = parameterFactory.createDouble("touchdownPressureLimit", 50);
   private final IntegerParameter touchdownTriggerWindowParameter = parameterFactory.createInteger("touchdownTriggerWindow", 1);
   private final DoubleParameter minimumStepAdjustmentTimeParameter = parameterFactory.createDouble("minimumStepAdjustmentTime", 0.1);
   private final DoubleParameter stepGoalOffsetZParameter = parameterFactory.createDouble("stepGoalOffsetZ", 0.0);

   
   public QuadrupedFootStateMachineParameters()
   {
      
   }
   
   
   public YoVariableRegistry getYoVariableRegistry()
   {
      return finalRegistry;
   }



   public double[] getSolePositionProportionalGainsParameter()
   {
      return solePositionProportionalGainsParameter.get();
   }


   public double[] getSolePositionDerivativeGainsParameter()
   {
      return solePositionDerivativeGainsParameter.get();
   }


   public double[] getSolePositionIntegralGainsParameter()
   {
      return solePositionIntegralGainsParameter.get();
   }


   public double getSolePositionMaxIntegralErrorParameter()
   {
      return solePositionMaxIntegralErrorParameter.get();
   }


   public double getTouchdownPressureLimitParameter()
   {
      return touchdownPressureLimitParameter.get();
   }


   public int getTouchdownTriggerWindowParameter()
   {
      return touchdownTriggerWindowParameter.get();
   }


   public double getMinimumStepAdjustmentTimeParameter()
   {
      return minimumStepAdjustmentTimeParameter.get();
   }


   public double getStepGoalOffsetZParameter()
   {
      return stepGoalOffsetZParameter.get();
   }
   
   
   
}
