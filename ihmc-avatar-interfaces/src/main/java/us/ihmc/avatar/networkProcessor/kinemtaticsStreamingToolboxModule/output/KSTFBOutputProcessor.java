package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.YoKinematicsToolboxOutputStatus;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.yoVariables.registry.YoRegistry;

public class KSTFBOutputProcessor implements KSTOutputProcessor
{
   private final YoPDGains gains;
   private final YoKinematicsToolboxOutputStatus outputRobotState;

   public KSTFBOutputProcessor(KSTTools tools, YoRegistry registry)
   {
      gains = new YoPDGains("Output", registry);
      gains.createDerivativeGainUpdater(false);
      gains.setKp(100.0);
      gains.setZeta(1.0);

      outputRobotState = new YoKinematicsToolboxOutputStatus("feedback", tools.getDesiredFullRobotModel(), registry);
   }

   @Override
   public void initialize()
   {
      outputRobotState.setToNaN();
   }

   @Override
   public void update(double time, boolean wasStreaming, boolean isStreaming, KSTOutputDataReadOnly latestOutput)
   {

      outputRobotState.setConfiguration(latestOutput);
   }

   @Override
   public KSTOutputDataReadOnly getProcessedOutput()
   {
      return outputRobotState;
   }
}
