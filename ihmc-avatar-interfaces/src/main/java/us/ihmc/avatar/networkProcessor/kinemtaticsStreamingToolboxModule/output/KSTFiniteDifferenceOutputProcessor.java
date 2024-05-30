package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class KSTFiniteDifferenceOutputProcessor implements KSTOutputProcessor
{
   private final BooleanProvider computeSignal;

   private final YoDouble previousTime;
   private final YoKinematicsToolboxOutputStatus previousRobotState;

   private final YoKinematicsToolboxOutputStatus outputRobotState;

   public KSTFiniteDifferenceOutputProcessor(KSTTools tools, BooleanProvider computeSignal, YoRegistry registry)
   {
      this.computeSignal = computeSignal;
      outputRobotState = new YoKinematicsToolboxOutputStatus("finiteDifference", tools.getDesiredFullRobotModel(), registry);

      previousTime = new YoDouble("previousTime", registry);
      previousRobotState = new YoKinematicsToolboxOutputStatus("previous", tools.getDesiredFullRobotModel(), registry);
   }

   @Override
   public void initialize()
   {
      previousTime.set(Double.NaN);
      previousRobotState.setToNaN();
      outputRobotState.setToNaN();
   }

   @Override
   public void update(double time, boolean wasStreaming, boolean isStreaming, KSTOutputDataReadOnly latestOutput)
   {
      if (computeSignal.getValue())
      {
         double dt = time - previousTime.getDoubleValue();
         outputRobotState.setConfiguration(latestOutput);
         outputRobotState.setVelocitiesByFiniteDifference(previousRobotState, latestOutput, dt);
         previousTime.set(time);
         previousRobotState.set(latestOutput);
      }
      else
      {
         outputRobotState.setConfiguration(latestOutput);
      }
   }

   @Override
   public KSTOutputDataReadOnly getProcessedOutput()
   {
      return outputRobotState;
   }
}
