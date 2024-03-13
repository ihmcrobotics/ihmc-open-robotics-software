package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class KSTBlendingOutputProcessor implements KSTOutputProcessor
{
   private final YoKinematicsToolboxOutputStatus initialRobotState;
   private final YoKinematicsToolboxOutputStatus blendedRobotState;

   private final YoDouble streamingStartTime;
   private final DoubleProvider streamingBlendingDuration;

   private final KSTTools tools;
   private final KinematicsStreamingToolboxParameters parameters;

   public KSTBlendingOutputProcessor(KSTTools tools, DoubleProvider streamingBlendingDuration, YoRegistry registry)
   {
      parameters = tools.getParameters();
      this.tools = tools;

      streamingStartTime = new YoDouble("streamingStartTime", registry);
      this.streamingBlendingDuration = streamingBlendingDuration;

      FullHumanoidRobotModel desiredFullRobotModel = tools.getDesiredFullRobotModel();
      initialRobotState = new YoKinematicsToolboxOutputStatus("initial", desiredFullRobotModel, registry);
      blendedRobotState = new YoKinematicsToolboxOutputStatus("blended", desiredFullRobotModel, registry);
      blendedRobotState.createAccelerationState();
   }

   @Override
   public void initialize()
   {
      streamingStartTime.set(Double.NaN);
      initialRobotState.setToNaN();
      blendedRobotState.setToNaN();
   }

   @Override
   public void update(double time, boolean wasStreaming, boolean isStreaming, KSTOutputDataReadOnly latestOutput)
   {
      if (isStreaming)
      {
         if (!wasStreaming)
         {
            tools.getCurrentState(initialRobotState);
            streamingStartTime.set(time);
         }

         double timeInBlending = time - streamingStartTime.getValue();

         if (timeInBlending < streamingBlendingDuration.getValue())
         {
            double alpha = MathTools.clamp(timeInBlending / streamingBlendingDuration.getValue(), 0.0, 1.0);
            blendedRobotState.interpolate(initialRobotState, latestOutput, alpha);
         }
         else
         {
            blendedRobotState.set(latestOutput);
         }
      }
   }

   @Override
   public KSTOutputDataReadOnly getProcessedOutput()
   {
      return blendedRobotState;
   }

   public DoubleProvider getBlendingDuration()
   {
      return streamingBlendingDuration;
   }
}
