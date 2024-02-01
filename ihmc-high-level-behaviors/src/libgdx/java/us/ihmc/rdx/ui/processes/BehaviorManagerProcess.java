package us.ihmc.rdx.ui.processes;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.log.LogTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

import java.util.function.Supplier;

public class BehaviorManagerProcess extends RestartableProcess
{
   private final Supplier<DRCRobotModel> robotModelSupplier;
   private IHMCHumanoidBehaviorManager behaviorManager;

   public BehaviorManagerProcess(Supplier<DRCRobotModel> robotModelSupplier)
   {
      this.robotModelSupplier = robotModelSupplier;
   }

   @Override
   protected void startInternal()
   {
      LogTools.info("Starting humanoid behavior manager");
      DRCRobotModel robotModel = robotModelSupplier.get();
      ExceptionTools.handle(() ->
      {
         HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
         LogModelProvider logModelProvider = robotModel.getLogModelProvider();
         behaviorManager = new IHMCHumanoidBehaviorManager(robotModel.getSimpleRobotName(),
                                                           robotModel.getFootstepPlannerParameters(),
                                                           robotModel,
                                                           robotModel,
                                                           logModelProvider,
                                                           false,
                                                           sensorInformation);
      }, DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   @Override
   protected void stopInternal()
   {
      behaviorManager.closeAndDispose();
      behaviorManager = null;
   }

   @Override
   public String getName()
   {
      return "Behavior manager";
   }
}
