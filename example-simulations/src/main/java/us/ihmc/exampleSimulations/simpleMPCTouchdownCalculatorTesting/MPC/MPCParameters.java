package us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.MPC;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MPCParameters
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final static double NUMBER_OF_STEPS_IN_HORIZON = 2;

   // MPC Specific
   private final YoDouble numberOfStepsInHorizon = new YoDouble("numberOfStepsInHorizon", registry);
   private final YoDouble timeDurationOfHorizon = new YoDouble("timeDurationOfHorizon", registry);
   private final YoDouble numberOfTicksInHorizon = new YoDouble("numberOfTicksInHorizon", registry);
   private final YoDouble numberOfTicksPerStep = new YoDouble("numberOfTicksPerStep", registry);

   private final YoDouble timeDurationOfStep;
   private final YoDouble controllerDt;

   public MPCParameters(YoDouble swingDuration, YoDouble controllerDt, YoRegistry parentRegistry)
   {
      this.timeDurationOfStep = swingDuration;
      this.controllerDt = controllerDt;

      numberOfStepsInHorizon.set(NUMBER_OF_STEPS_IN_HORIZON);

      timeDurationOfStep.addListener(value -> updateMPCParametersFromNewSwingDuration());

      parentRegistry.addChild(registry);
   }

   private void updateMPCParametersFromNewSwingDuration()
   {
      numberOfTicksPerStep.set(timeDurationOfStep.getDoubleValue() / controllerDt.getDoubleValue());
      numberOfTicksInHorizon.set(numberOfTicksPerStep.getDoubleValue() * numberOfStepsInHorizon.getDoubleValue());
      timeDurationOfHorizon.set(timeDurationOfStep.getDoubleValue() * numberOfStepsInHorizon.getDoubleValue());
   }

   public double getNumberOfStepsInHorizon()
   {
      return numberOfStepsInHorizon.getDoubleValue();
   }

   public double getTimeDurationOfHorizon()
   {
      return timeDurationOfHorizon.getDoubleValue();
   }

   public double getNumberOfTicksInHorizon()
   {
      return numberOfTicksInHorizon.getDoubleValue();
   }

   public double getNumberOfTicksPerStep()
   {
      return numberOfTicksPerStep.getDoubleValue();
   }

   public double getTimeDurationOfStep()
   {
      return timeDurationOfStep.getDoubleValue();
   }
}
