package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class KSTCompiledOutputProcessor implements KSTOutputProcessor
{
   private final YoKinematicsToolboxOutputStatus ikRobotState;
   private final YoKinematicsToolboxOutputStatus outputRobotState;
   private final KSTOutputProcessors outputProcessors = new KSTOutputProcessors();

   private final YoDouble outputLPFBreakFrequency;
   private final YoDouble outputJointVelocityScale;

   public KSTCompiledOutputProcessor(KSTTools tools, DoubleProvider streamingBlendingDuration, BooleanProvider isPublishing, YoRegistry registry)
   {
      outputLPFBreakFrequency = new YoDouble("outputLPFBreakFrequency", registry);
      outputJointVelocityScale = new YoDouble("outputJointVelocityScale", registry);

      FullHumanoidRobotModel desiredFullRobotModel = tools.getDesiredFullRobotModel();
      ikRobotState = new YoKinematicsToolboxOutputStatus("IK", desiredFullRobotModel, registry);
      outputRobotState = new YoKinematicsToolboxOutputStatus("output", desiredFullRobotModel, registry);
      outputRobotState.createAccelerationState();

      KinematicsStreamingToolboxParameters parameters = tools.getParameters();
      outputJointVelocityScale.set(parameters.getOutputJointVelocityScale());
      outputLPFBreakFrequency.set(parameters.getOutputLPFBreakFrequency());

      //      outputProcessors.add(new KSTFiniteDifferenceOutputProcessor(tools, isPublishing, registry));
      outputProcessors.add(new KSTDownscaleVelocityOutputProcessor(tools, outputJointVelocityScale, registry));
      outputProcessors.add(new KSTFBOutputProcessor(tools, registry));
      outputProcessors.add(new KSTLowPassFilteredOutputProcessor(tools, outputLPFBreakFrequency, registry));
      outputProcessors.add(new KSTBlendingOutputProcessor(tools, streamingBlendingDuration, registry));
   }

   @Override
   public void initialize()
   {
      ikRobotState.setToNaN();
      outputRobotState.setToNaN();
      outputProcessors.initialize();
   }

   @Override
   public void update(double time, boolean wasStreaming, boolean isStreaming, KSTOutputDataReadOnly latestOutput)
   {
      ikRobotState.set(latestOutput);
      outputProcessors.update(time, wasStreaming, isStreaming, latestOutput);
      outputRobotState.set(outputProcessors.getProcessedOutput());
   }

   @Override
   public KSTOutputDataReadOnly getProcessedOutput()
   {
      return outputRobotState;
   }
}
