package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.dyanmicsBasedFootstepGenerator;

import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.HumanoidSteppingPlugin;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.HumanoidSteppingPluginFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.StepGeneratorCommandInputManager;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.function.Consumer;

public class DynamicsBasedFootstepPluginFactory implements HumanoidSteppingPluginFactory
{
   private final OptionalFactoryField<StepGeneratorCommandInputManager> csgCommandInputManagerField = new OptionalFactoryField<>("csgCommandInputManagerField");

   public StepGeneratorCommandInputManager setStepGeneratorCommandInputManager()
   {
      StepGeneratorCommandInputManager csgCommandInputManager = new StepGeneratorCommandInputManager();
      setStepGeneratorCommandInputManager(csgCommandInputManager);
      return csgCommandInputManager;
   }

   public void setStepGeneratorCommandInputManager(StepGeneratorCommandInputManager commandInputManager)
   {
      this.csgCommandInputManagerField.set(commandInputManager);
   }

   @Override
   public StepGeneratorCommandInputManager getStepGeneratorCommandInputManager()
   {
      if (csgCommandInputManagerField.hasValue())
         return csgCommandInputManagerField.get();
      else
         return setStepGeneratorCommandInputManager();
   }

   @Override
   public void setFootStepAdjustment(FootstepAdjustment footstepAdjustment)
   {

   }

   @Override
   public void setFootStepPlanAdjustment(FootstepPlanAdjustment footstepAdjustment)
   {

   }

   @Override
   public void addFootstepValidityIndicator(FootstepValidityIndicator footstepValidityIndicator)
   {

   }

   @Override
   public void addPlanarRegionsListCommandConsumer(Consumer<PlanarRegionsListCommand> planarRegionsListCommandConsumer)
   {

   }

   @Override
   public void addUpdatable(Updatable updatable)
   {

   }

   @Override
   public DynamicsBasedFootstepPlugin buildPlugin(FullHumanoidRobotModel robotModel,
                                                  CommonHumanoidReferenceFrames referenceFrames,
                                                  double updateDT,
                                                  WalkingControllerParameters walkingControllerParameters,
                                                  StatusMessageOutputManager walkingStatusMessageOutputManager,
                                                  CommandInputManager walkingCommandInputManager,
                                                  YoGraphicsListRegistry yoGraphicsListRegistry,
                                                  SideDependentList<? extends ContactableBody> contactableFeet,
                                                  DoubleProvider timeProvider)
   {
      DynamicsBasedFootstepPlugin dynamicsBasedFootstepPlugin = new DynamicsBasedFootstepPlugin(robotModel, referenceFrames, updateDT, yoGraphicsListRegistry);

      dynamicsBasedFootstepPlugin.setStopWalkingMessenger(new StopWalkingMessenger()
      {
         private final PauseWalkingMessage message = HumanoidMessageTools.createPauseWalkingMessage(true);

         @Override
         public void submitStopWalkingRequest()
         {
            message.setClearRemainingFootstepQueue(true);
            walkingCommandInputManager.submitMessage(message);
         }
      });

      dynamicsBasedFootstepPlugin.setStartWalkingMessenger(new StartWalkingMessenger()
      {
         private final PauseWalkingMessage message = HumanoidMessageTools.createPauseWalkingMessage(false);

         @Override
         public void submitStartWalkingRequest()
         {
            walkingCommandInputManager.submitMessage(message);
         }
      });

      dynamicsBasedFootstepPlugin.setFootstepStatusListener(walkingStatusMessageOutputManager);

      dynamicsBasedFootstepPlugin.setFootstepMessenger(walkingCommandInputManager::submitMessage);

      return dynamicsBasedFootstepPlugin;
   }
}
