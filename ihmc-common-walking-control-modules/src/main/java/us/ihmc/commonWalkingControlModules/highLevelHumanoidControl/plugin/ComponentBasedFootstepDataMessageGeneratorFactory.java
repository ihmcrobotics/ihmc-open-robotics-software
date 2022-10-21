package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
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
   private final OptionalFactoryField<Boolean> createSupportFootBasedFootstepAdjustment = new OptionalFactoryField<>("csgCreateSupportFootBasedFootstepAdjustment");
   /** This is used only when the support foot based footstep adjustment is created. */
   private final OptionalFactoryField<Boolean> adjustPitchAndRoll = new OptionalFactoryField<>("csgSupportFootBasedFootstepAdjustmentAdjustPitchAndRoll");
   private final OptionalFactoryField<FootstepAdjustment> primaryFootstepAdjusterField = new OptionalFactoryField<>("csgPrimaryFootstepAdjusterField");
   private final OptionalFactoryField<List<FootstepAdjustment>> secondaryFootstepAdjusterField = new OptionalFactoryField<>("csgSecondaryFootstepAdjusterFields");

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

   public void setFootStepAdjustment(FootstepAdjustment footStepAdjustment)
   {
      primaryFootstepAdjusterField.set(footStepAdjustment);
   }

   public void addSecondaryFootStepAdjustment(FootstepAdjustment footStepAdjustment)
   {
      if (!secondaryFootstepAdjusterField.hasValue())
         secondaryFootstepAdjusterField.set(new ArrayList<>());
      secondaryFootstepAdjusterField.get().add(footStepAdjustment);
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

   @Override
   public StepGeneratorCommandInputManager getStepGeneratorCommandInputManager()
   {
      if (csgCommandInputManagerField.hasValue())
         return csgCommandInputManagerField.get();
      else
         return setStepGeneratorCommandInputManager();
   }

   @Override
   public ComponentBasedFootstepDataMessageGenerator buildPlugin(CommonHumanoidReferenceFrames referenceFrames,
                                                                 double updateDT,
                                                                 WalkingControllerParameters walkingControllerParameters,
                                                                 StatusMessageOutputManager walkingStatusMessageOutputManager,
                                                                 CommandInputManager walkingCommandInputManager,
                                                                 YoGraphicsListRegistry yoGraphicsListRegistry,
                                                                 SideDependentList<? extends ContactableBody> contactableFeet,
                                                                 DoubleProvider timeProvider)
   {
      if (!registryField.hasValue())
         setRegistry();

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator(registryField.get());

      if (createSupportFootBasedFootstepAdjustment.hasValue() && createSupportFootBasedFootstepAdjustment.get())
         continuousStepGenerator.setSupportFootBasedFootstepAdjustment(adjustPitchAndRoll.hasValue() && adjustPitchAndRoll.get());
      if (secondaryFootstepAdjusterField.hasValue())
      {
         for (FootstepAdjustment footstepAdjustment : secondaryFootstepAdjusterField.get())
            continuousStepGenerator.addFootstepAdjustment(footstepAdjustment);
      }
      continuousStepGenerator.setFootstepStatusListener(walkingStatusMessageOutputManager);
      continuousStepGenerator.setFrameBasedFootPoseProvider(referenceFrames.getSoleZUpFrames());
      continuousStepGenerator.configureWith(walkingControllerParameters);
      continuousStepGenerator.setStopWalkingMessenger(new StopWalkingMessenger()
      {
         private final PauseWalkingMessage message = HumanoidMessageTools.createPauseWalkingMessage(true);

         @Override
         public void submitStopWalkingRequest()
         {
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

      List<Updatable> updatables = new ArrayList<>();

      if (yoGraphicsListRegistry != null && contactableFeet != null)
         continuousStepGenerator.setupVisualization(contactableFeet, yoGraphicsListRegistry);
      if (primaryFootstepAdjusterField.hasValue() && primaryFootstepAdjusterField.get() != null)
         continuousStepGenerator.setFootstepAdjustment(primaryFootstepAdjusterField.get());

      if (useHeadingAndVelocityScriptField.get())
      {
         HeadingAndVelocityEvaluationScript script = new HeadingAndVelocityEvaluationScript(updateDT,
                                                                                            timeProvider,
                                                                                            headingAndVelocityEvaluationScriptParametersField.get(),
                                                                                            registryField.get());
         continuousStepGenerator.setDesiredTurningVelocityProvider(script.getDesiredTurningVelocityProvider());
         continuousStepGenerator.setDesiredVelocityProvider(script.getDesiredVelocityProvider());
         updatables.add(script);
      }
      else if (csgCommandInputManagerField.hasValue())
      {
         StepGeneratorCommandInputManager commandInputManager = csgCommandInputManagerField.get();
         continuousStepGenerator.setDesiredVelocityProvider(commandInputManager.createDesiredVelocityProvider());
         continuousStepGenerator.setDesiredTurningVelocityProvider(commandInputManager.createDesiredTurningVelocityProvider());
         continuousStepGenerator.setWalkInputProvider(commandInputManager.createWalkInputProvider());
         walkingStatusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class,
                                                                       commandInputManager::setHighLevelStateChangeStatusMessage);
         updatables.add(commandInputManager);

         //this is probably not the way the class was intended to be modified.
         commandInputManager.setCSG(continuousStepGenerator);
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
