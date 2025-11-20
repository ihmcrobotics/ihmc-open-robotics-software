package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.quicksterFootstepProvider.QuicksterFootstepProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeightMapCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ComponentBasedFootstepDataMessageGeneratorFactory implements HumanoidSteppingPluginFactory
{
   private final OptionalFactoryField<YoRegistry> registryField = new OptionalFactoryField<>("registry");
   private final OptionalFactoryField<Boolean> useHeadingAndVelocityScriptField = new OptionalFactoryField<>("useHeadingAndVelocityScript", false);
   private final OptionalFactoryField<HeadingAndVelocityEvaluationScriptParameters> headingAndVelocityEvaluationScriptParametersField = new OptionalFactoryField<>("headingAndVelocityEvaluationScriptParameters");
   private final OptionalFactoryField<StepGeneratorCommandInputManager> csgCommandInputManagerField = new OptionalFactoryField<>("csgCommandInputManagerField");
   private final OptionalFactoryField<StatusMessageOutputManager> csgStatusMessageOutputManagerField = new OptionalFactoryField<>("csgStatusMessageOutputManagerField");
   private final OptionalFactoryField<Boolean> createSupportFootBasedFootstepAdjustment = new OptionalFactoryField<>("csgCreateSupportFootBasedFootstepAdjustment");
   /** This is used only when the support foot based footstep adjustment is created. */
   private final OptionalFactoryField<Boolean> adjustPitchAndRoll = new OptionalFactoryField<>("csgSupportFootBasedFootstepAdjustmentAdjustPitchAndRoll");
   private final OptionalFactoryField<FootstepAdjustment> primaryFootstepAdjusterField = new OptionalFactoryField<>("csgPrimaryFootstepAdjusterField");
   private final List<FootstepAdjustment> secondaryFootstepAdjusters = new ArrayList<>();
   private final List<FootstepValidityIndicator> footstepValidityIndicators = new ArrayList<>();
   private final List<Consumer<HeightMapCommand>> heightMapCommandConsumers = new ArrayList<>();

   private final List<Updatable> updatables = new ArrayList<>();

   public ComponentBasedFootstepDataMessageGeneratorFactory()
   {
      createSupportFootBasedFootstepAdjustment.setDefaultValue(true);
      adjustPitchAndRoll.setDefaultValue(false);
   }

   public void setRegistry()
   {
      setRegistry(ComponentBasedFootstepDataMessageGenerator.class.getSimpleName());
   }

   public void setRegistry(String name)
   {
      registryField.set(new YoRegistry(name));
   }

   @Override
   public void setFootStepAdjustment(FootstepAdjustment footStepAdjustment)
   {
      primaryFootstepAdjusterField.set(footStepAdjustment);
   }

   @Override
   public void addFootstepValidityIndicator(FootstepValidityIndicator footstepValidityIndicator)
   {
      footstepValidityIndicators.add(footstepValidityIndicator);
   }

   @Override
   public void addHeightMapCommandConsumer(Consumer<HeightMapCommand> heightMapCommandConsumer)
   {
      heightMapCommandConsumers.add(heightMapCommandConsumer);
   }

   @Override
   public void addUpdatable(Updatable updatable)
   {
      this.updatables.add(updatable);
   }

   public void addSecondaryFootStepAdjustment(FootstepAdjustment footStepAdjustment)
   {
      secondaryFootstepAdjusters.add(footStepAdjustment);
   }

   public void setUseHeadingAndVelocityScript(boolean useHeadingAndVelocityScript)
   {
      useHeadingAndVelocityScriptField.set(useHeadingAndVelocityScript);
   }

   public void setHeadingAndVelocityEvaluationScriptParameters(HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters)
   {
      this.headingAndVelocityEvaluationScriptParametersField.set(headingAndVelocityEvaluationScriptParameters);
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

   public void setStepGeneratorStatusMessageOutputManager(StatusMessageOutputManager statusMessageOutputManager)
   {
      this.csgStatusMessageOutputManagerField.set(statusMessageOutputManager);
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
   public StatusMessageOutputManager getStepGeneratorStatusMessageOutputManager()
   {
      if (!csgStatusMessageOutputManagerField.hasValue())
         csgStatusMessageOutputManagerField.set(new StatusMessageOutputManager(StepGeneratorAPIDefinition.getStepGeneratorSupportedStatusMessages()));

      return csgStatusMessageOutputManagerField.get();
   }

   @Override
   public ComponentBasedFootstepDataMessageGenerator buildPlugin(FullHumanoidRobotModel robotModel,
                                                                 CommonHumanoidReferenceFrames referenceFrames,
                                                                 double updateDT,
                                                                 WalkingControllerParameters walkingControllerParameters,
                                                                 StatusMessageOutputManager walkingStatusMessageOutputManager,
                                                                 CommandInputManager walkingCommandInputManager,
                                                                 SideDependentList<? extends ContactableBody> contactableFeet,
                                                                 DoubleProvider timeProvider)
   {
      if (!registryField.hasValue())
         setRegistry();

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator(getStepGeneratorStatusMessageOutputManager(), registryField.get());
      continuousStepGenerator.setQuicksterFootstepProvider(new QuicksterFootstepProvider(robotModel,
                                                                                         referenceFrames,
                                                                                         updateDT,
                                                                                         registryField.get(),
                                                                                         null,
                                                                                         timeProvider));

      if (createSupportFootBasedFootstepAdjustment.hasValue() && createSupportFootBasedFootstepAdjustment.get())
         continuousStepGenerator.setSupportFootBasedFootstepAdjustment(adjustPitchAndRoll.hasValue() && adjustPitchAndRoll.get());
      if (primaryFootstepAdjusterField.hasValue() && primaryFootstepAdjusterField.get() != null)
         continuousStepGenerator.setFootstepAdjustment(primaryFootstepAdjusterField.get());
      for (FootstepAdjustment footstepAdjustment : secondaryFootstepAdjusters)
         continuousStepGenerator.addFootstepAdjustment(footstepAdjustment);
      for (FootstepValidityIndicator footstepValidityIndicator : footstepValidityIndicators)
         continuousStepGenerator.addFootstepValidityIndicator(footstepValidityIndicator);
      continuousStepGenerator.setFootstepStatusListener(walkingStatusMessageOutputManager);
      continuousStepGenerator.setFrameBasedFootPoseProvider(referenceFrames.getSoleZUpFrames());
      continuousStepGenerator.configureWith(walkingControllerParameters);
      continuousStepGenerator.setStopWalkingMessenger(new StopWalkingMessenger()
      {
         private final PauseWalkingMessage message = HumanoidMessageTools.createPauseWalkingMessage(true);

         @Override
         public void submitStopWalkingRequest()
         {
            message.setClearRemainingFootstepQueue(true);
            walkingCommandInputManager.submitMessage(message);
         }
      });
      continuousStepGenerator.setStartWalkingMessenger(new StartWalkingMessenger()
      {
         private final PauseWalkingMessage message = HumanoidMessageTools.createPauseWalkingMessage(false);

         @Override
         public void submitStartWalkingRequest()
         {
            walkingCommandInputManager.submitMessage(message);
         }
      });

      continuousStepGenerator.setFootstepMessenger(walkingCommandInputManager::submitMessage);

      if (contactableFeet != null)
         continuousStepGenerator.setupVisualization(contactableFeet);

      if (useHeadingAndVelocityScriptField.get())
      {
         HeadingAndVelocityEvaluationScriptParameters parameters = headingAndVelocityEvaluationScriptParametersField.hasValue() ? headingAndVelocityEvaluationScriptParametersField.get()
                                                                                                                                : null;
         HeadingAndVelocityEvaluationScript script = new HeadingAndVelocityEvaluationScript(updateDT, timeProvider, parameters, registryField.get());
         continuousStepGenerator.setDesiredTurningVelocityProvider(script.getDesiredTurningVelocityProvider());
         continuousStepGenerator.setDesiredVelocityProvider(script.getDesiredVelocityProvider());
         updatables.add(script);
      }
      else if (csgCommandInputManagerField.hasValue())
      {
         StepGeneratorCommandInputManager commandInputManager = csgCommandInputManagerField.get();
         for (Consumer<HeightMapCommand> heightMapCommandConsumer : heightMapCommandConsumers)
            commandInputManager.addHeightMapCommandConsumer(heightMapCommandConsumer);

         continuousStepGenerator.setDesiredVelocityProvider(commandInputManager.createDesiredVelocityProvider());
         continuousStepGenerator.setDesiredTurningVelocityProvider(commandInputManager.createDesiredTurningVelocityProvider());
         continuousStepGenerator.setWalkInputProvider(commandInputManager.createWalkInputProvider());
         walkingStatusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class,
                                                                       commandInputManager::setHighLevelStateChangeStatusMessage);
         walkingStatusMessageOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, commandInputManager::setWalkingStatus);
         walkingStatusMessageOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, commandInputManager::consumeFootstepStatus);
         commandInputManager.setFootstepStatusListener(walkingStatusMessageOutputManager);

         updatables.add(commandInputManager);

         //this is probably not the way the class was intended to be modified.
         commandInputManager.setCSG(continuousStepGenerator);
         continuousStepGenerator.setYoComponentProviders();
      }
      else
      {
         continuousStepGenerator.setYoComponentProviders();
      }

      ComponentBasedFootstepDataMessageGenerator plugin = new ComponentBasedFootstepDataMessageGenerator(continuousStepGenerator,
                                                                                                         updatables,
                                                                                                         registryField.get());
      FactoryTools.disposeFactory(this);
      return plugin;
   }
}
