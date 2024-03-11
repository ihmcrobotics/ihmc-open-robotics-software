package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.YoKinematicsToolboxOutputStatus;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class KSTLowPassFilteredOutputProcessor implements KSTOutputProcessor
{
   private final YoKinematicsToolboxOutputStatus filteredRobotState;
   private final DoubleProvider breakFrequency;
   private final double updateDT;

   private boolean resetFilter = false;

   public KSTLowPassFilteredOutputProcessor(KSTTools tools, DoubleProvider breakFrequency, YoRegistry registry)
   {
      this.breakFrequency = breakFrequency;
      updateDT = tools.getToolboxControllerPeriod();

      FullHumanoidRobotModel desiredFullRobotModel = tools.getDesiredFullRobotModel();
      filteredRobotState = new YoKinematicsToolboxOutputStatus("lpf", desiredFullRobotModel, registry);
   }

   @Override
   public void initialize()
   {
      resetFilter = true;
      filteredRobotState.setToNaN();
   }

   @Override
   public void update(double time, boolean wasStreaming, boolean isStreaming, KSTOutputDataReadOnly latestOutput)
   {
      if (resetFilter)
      {
         filteredRobotState.set(latestOutput);
         resetFilter = false;
      }
      else
      {
         double alphaFilter = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency.getValue(), updateDT);
         filteredRobotState.interpolate(latestOutput, filteredRobotState, alphaFilter);
      }
   }

   @Override
   public KSTOutputDataReadOnly getProcessedOutput()
   {
      return filteredRobotState;
   }
}
