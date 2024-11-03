package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class VelocityBasedSteppingPluginFactory implements HumanoidSteppingPluginFactory
{
   private final RequiredFactoryField<YoRegistry> registryField = new RequiredFactoryField<>("registry");
   private final OptionalFactoryField<StepGeneratorCommandInputManager> csgCommandInputManagerField = new OptionalFactoryField<>("csgCommandInputManagerField");
   private final OptionalFactoryField<VelocityBasedSteppingParameters> inputParametersField = new OptionalFactoryField<>("inputParametersField");
   private final List<Updatable> updatables = new ArrayList<>();

   public void setRegistry()
   {
      setRegistry(ComponentBasedFootstepDataMessageGenerator.class.getSimpleName());
   }

   public void setRegistry(String name)
   {
      registryField.set(new YoRegistry(name));
   }

   public void setInputParameters(VelocityBasedSteppingParameters parameters)
   {
      inputParametersField.set(parameters);
   }

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
   public void setFootStepAdjustment(FootstepAdjustment footstepAdjustment)
   {
   }

   @Override
   public void setFootStepPlanAdjustment(FootstepPlanAdjustment footStepAdjustment)
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
      updatables.add(updatable);
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
   public VelocityBasedSteppingPlugin buildPlugin(CommonHumanoidReferenceFrames referenceFrames,
                                                  double updateDT,
                                                  WalkingControllerParameters walkingControllerParameters,
                                                  StatusMessageOutputManager walkingStatusMessageOutputManager,
                                                  CommandInputManager walkingCommandInputManager,
                                                  YoGraphicsListRegistry yoGraphicsListRegistry,
                                                  SideDependentList<? extends ContactableBody> contactableFeet,
                                                  DoubleProvider timeProvider)
   {
      if (!registryField.hasBeenSet())
         setRegistry();

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      VelocityBasedSteppingPlugin fastWalkingJoystickPlugin = new VelocityBasedSteppingPlugin(updatables);

      fastWalkingJoystickPlugin.setDirectionalControlMessenger(new DirectionalControlMessenger()
      {
         private final DirectionalControlInputMessage message = new DirectionalControlInputMessage();
         private final FastWalkingGaitParametersMessage gaitParameters = new FastWalkingGaitParametersMessage();

         @Override
         public void submitDirectionalControlRequest(double desiredXVelocity, double desiredYVelocity, double desiredTurningSpeed)
         {
            message.setForward(desiredXVelocity);
            message.setRight(-desiredYVelocity);
            message.setClockwise(-desiredTurningSpeed);

            walkingCommandInputManager.submitMessage(message);
         }

         @Override
         public void submitGaitParameters(double swingHeight, double swingDuration, double doubleSupportFraction)
         {
            gaitParameters.setSwingHeight(swingHeight);
            gaitParameters.setSwingDuration(swingDuration);
            gaitParameters.setDoubleSupportFraction(doubleSupportFraction);

            walkingCommandInputManager.submitMessage(gaitParameters);
         }
      });
      if (csgCommandInputManagerField.hasValue())
      {
         StepGeneratorCommandInputManager commandInputManager = csgCommandInputManagerField.get();
         fastWalkingJoystickPlugin.setDesiredVelocityProvider(commandInputManager.createDesiredVelocityProvider());
         fastWalkingJoystickPlugin.setDesiredTurningVelocityProvider(commandInputManager.createDesiredTurningVelocityProvider());
         fastWalkingJoystickPlugin.setWalkInputProvider(commandInputManager.createWalkInputProvider());
         walkingStatusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class, commandInputManager::setHighLevelStateChangeStatusMessage);
         walkingStatusMessageOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, commandInputManager::setWalkingStatus);
         commandInputManager.setFootstepStatusListener(walkingStatusMessageOutputManager);

         updatables.add(commandInputManager);
      }

      fastWalkingJoystickPlugin.setStopWalkingMessenger(new StopWalkingMessenger()
      {
         private final PauseWalkingMessage message = HumanoidMessageTools.createPauseWalkingMessage(true);

         @Override
         public void submitStopWalkingRequest()
         {
            message.setClearRemainingFootstepQueue(true);
            walkingCommandInputManager.submitMessage(message);
         }
      });

      fastWalkingJoystickPlugin.setStartWalkingMessenger(new StartWalkingMessenger()
      {
         private final PauseWalkingMessage message = HumanoidMessageTools.createPauseWalkingMessage(false);

         @Override
         public void submitStartWalkingRequest()
         {
            walkingCommandInputManager.submitMessage(message);
         }
      });
      if (inputParametersField.hasValue())
         fastWalkingJoystickPlugin.setInputParameters(inputParametersField.get());

      FactoryTools.disposeFactory(this);

      return fastWalkingJoystickPlugin;
   }
}
