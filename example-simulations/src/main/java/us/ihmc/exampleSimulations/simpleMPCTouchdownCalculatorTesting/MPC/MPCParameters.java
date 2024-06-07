package us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.MPC;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MPCParameters
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final static double NUMBER_OF_STEPS_IN_HORIZON = 2;
   private final static double GRAVITY = 9.81;
   private final static double DESIRED_STANCE_WIDTH = 0.3;

   // MPC Specific
   private final YoDouble numberOfStepsInHorizon = new YoDouble("numberOfStepsInHorizon", registry);
   private final YoDouble timeDurationOfHorizon = new YoDouble("timeDurationOfHorizon", registry);
   private final YoDouble numberOfTicksInHorizon = new YoDouble("numberOfTicksInHorizon", registry);
   private final YoDouble numberOfTicksPerStep = new YoDouble("numberOfTicksPerStep", registry);

   // Other
   private final YoDouble desiredWalkingHeight = new YoDouble("desiredWalkingHeight", registry);
   private final YoDouble desiredStanceWidth = new YoDouble("desiredStanceWidth", registry);
   private final YoDouble omegaX = new YoDouble("omegaX", registry);
   private final YoDouble omegaY = new YoDouble("omegaY", registry);

   private final YoDouble swingDuration;
   private final YoDouble controllerDt;

   public MPCParameters(YoDouble swingDuration, double desiredWalkingHeight, YoDouble controllerDt, YoRegistry parentRegistry)
   {
      this.swingDuration = swingDuration;
      this.controllerDt = controllerDt;

      numberOfStepsInHorizon.set(NUMBER_OF_STEPS_IN_HORIZON);
      desiredStanceWidth.set(DESIRED_STANCE_WIDTH);
      this.desiredWalkingHeight.set(desiredWalkingHeight);
      omegaX.set(Math.sqrt(getGravity() / this.desiredWalkingHeight.getDoubleValue()));
      omegaY.set(Math.sqrt(getGravity() / this.desiredWalkingHeight.getDoubleValue()));

      //this.swingDuration.addListener(value -> updateMPCParametersFromNewSwingDuration());

      parentRegistry.addChild(registry);
   }

//   private void updateMPCParametersFromNewSwingDuration()
//   {
//      numberOfTicksPerStep.set(swingDuration.getDoubleValue() / controllerDt.getDoubleValue());
//      numberOfTicksInHorizon.set(numberOfTicksPerStep.getDoubleValue() * numberOfStepsInHorizon.getDoubleValue());
//      timeDurationOfHorizon.set(swingDuration.getDoubleValue() * numberOfStepsInHorizon.getDoubleValue());
//   }

   public double getDesiredWalkingHeight()
   {
      return desiredWalkingHeight.getDoubleValue();
   }

   public double getStanceWidth()
   {
      return desiredWalkingHeight.getDoubleValue();
   }

   public YoDouble getOmegaX()
   {
      return omegaX;
   }

   public YoDouble getOmegaY()
   {
      return omegaY;
   }

   public double getGravity()
   {
      return GRAVITY;
   }

   public YoDouble getSwingDuration()
   {
      return swingDuration;
   }

   public double getControllerDt()
   {
      return controllerDt.getDoubleValue();
   }
}
