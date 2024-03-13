package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class KSTDownscaleVelocityOutputProcessor implements KSTOutputProcessor
{
   private final KSTTools tools;
   private final DoubleProvider downscaleFactor;

   private final YoKinematicsToolboxOutputStatus outputRobotState;

   public KSTDownscaleVelocityOutputProcessor(KSTTools tools, DoubleProvider downscaleFactor, YoRegistry registry)
   {
      this.tools = tools;
      this.downscaleFactor = downscaleFactor;

      FullHumanoidRobotModel desiredFullRobotModel = tools.getDesiredFullRobotModel();
      outputRobotState = new YoKinematicsToolboxOutputStatus("downscaled", desiredFullRobotModel, registry);
   }

   @Override
   public void initialize()
   {
      outputRobotState.setToNaN();
   }

   @Override
   public void update(double time, boolean wasStreaming, boolean isStreaming, KSTOutputDataReadOnly latestOutput)
   {
      if (isStreaming)
      {
         outputRobotState.set(latestOutput);
         outputRobotState.scaleVelocities(downscaleFactor.getValue());
      }
   }

   @Override
   public KSTOutputDataReadOnly getProcessedOutput()
   {
      return outputRobotState;
   }
}
