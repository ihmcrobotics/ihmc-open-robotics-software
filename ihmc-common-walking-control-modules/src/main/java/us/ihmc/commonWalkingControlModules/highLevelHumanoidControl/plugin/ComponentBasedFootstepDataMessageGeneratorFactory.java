package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.DirectionalControlInputMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ComponentBasedFootstepDataMessageGeneratorFactory implements HighLevelHumanoidControllerPluginFactory
{
   private final OptionalFactoryField<YoRegistry> registryField = new OptionalFactoryField<>("registry");
   private final OptionalFactoryField<Boolean> useHeadingAndVelocityScriptField = new OptionalFactoryField<>("useHeadingAndVelocityScript", false);
   private final OptionalFactoryField<HeadingAndVelocityEvaluationScriptParameters> headingAndVelocityEvaluationScriptParametersField = new OptionalFactoryField<>("headingAndVelocityEvaluationScriptParameters");
   private final OptionalFactoryField<CSGCommandInputManager> csgCommandInputManagerField = new OptionalFactoryField<>("csgCommandInputManagerField");
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

   public CSGCommandInputManager setCSGCommandInputManager()
   {
      CSGCommandInputManager csgCommandInputManager = new CSGCommandInputManager();
      setCSGCommandInputManager(csgCommandInputManager);
      return csgCommandInputManager;
   }

   public void setCSGCommandInputManager(CSGCommandInputManager commandInputManager)
   {
      this.csgCommandInputManagerField.set(commandInputManager);
   }

   public CSGCommandInputManager getCSGCommandInputManager()
   {
      if (csgCommandInputManagerField.hasValue())
         return csgCommandInputManagerField.get();
      else
         return setCSGCommandInputManager();
   }

   @Override
   public ComponentBasedFootstepDataMessageGenerator buildPlugin(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      HighLevelHumanoidControllerToolbox controllerToolbox = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox();

      return buildPlugin(controllerToolbox.getReferenceFrames(),
                         controllerToolbox.getControlDT(),
                         controllerFactoryHelper.getWalkingControllerParameters(),
                         controllerFactoryHelper.getStatusMessageOutputManager(),
                         controllerFactoryHelper.getCommandInputManager(),
                         controllerToolbox.getYoGraphicsListRegistry(),
                         controllerToolbox.getContactableFeet(),
                         controllerToolbox.getYoTime() );
   }

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
         private final FootstepDataListMessage emptyFootstepMessage = new FootstepDataListMessage();

         @Override
         public void submitStopWalkingRequest()
         {
            walkingCommandInputManager.submitMessage(message);
            walkingCommandInputManager.submitMessage(emptyFootstepMessage);
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
      continuousStepGenerator.setDirectionalControlMessenger(new DirectionalControlMessenger()
      {
         private final DirectionalControlInputMessage message = new DirectionalControlInputMessage();

         @Override
         public void submitDirectionalControlRequest(double desiredXVelocity, double desiredYVelocity, double desiredTurningSpeed)
         {
            message.setForward(desiredXVelocity);
            message.setRight(-desiredYVelocity);
            message.setClockwise(desiredTurningSpeed);

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
         CSGCommandInputManager commandInputManager = csgCommandInputManagerField.get();
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

   public static class CSGCommandInputManager implements Updatable
   {
      private final CommandInputManager commandInputManager = new CommandInputManager(supportedCommands());

      private boolean isOpen = false;
      private boolean walk = false;
      private boolean isUnitVelocities = false;
      private final Vector2D desiredVelocity = new Vector2D();
      private double turningVelocity = 0.0;
      private HighLevelControllerName currentController;
      private ContinuousStepGenerator continuousStepGenerator;

      public CSGCommandInputManager()
      {
      }

      public void setCSG(ContinuousStepGenerator continuousStepGenerator)
      {
         this.continuousStepGenerator = continuousStepGenerator;

      }

      public CommandInputManager getCommandInputManager()
      {
         return commandInputManager;
      }

      public List<Class<? extends Command<?, ?>>> supportedCommands()
      {
         List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
         commands.add(ContinuousStepGeneratorParametersCommand.class);
         commands.add(ContinuousStepGeneratorInputCommand.class);
         commands.add(PlanarRegionsListCommand.class);
         return commands;
      }

      private void setHighLevelStateChangeStatusMessage(HighLevelStateChangeStatusMessage message)
      {
         currentController = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
      }

      @Override
      public void update(double time)
      {
         isOpen = currentController == HighLevelControllerName.WALKING || currentController == HighLevelControllerName.CUSTOM1;

         if (commandInputManager.isNewCommandAvailable(ContinuousStepGeneratorInputCommand.class))
         {
            ContinuousStepGeneratorInputCommand command = commandInputManager.pollNewestCommand(ContinuousStepGeneratorInputCommand.class);
            desiredVelocity.setX(command.getForwardVelocity());
            desiredVelocity.setY(command.getLateralVelocity());
            turningVelocity = command.getTurnVelocity();
            isUnitVelocities = command.isUnitVelocities();
            walk = command.isWalk();
         }

         if (commandInputManager.isNewCommandAvailable(ContinuousStepGeneratorParametersCommand.class))
         {
            ContinuousStepGeneratorParametersCommand command = commandInputManager.pollNewestCommand(ContinuousStepGeneratorParametersCommand.class);
            ContinuousStepGeneratorParameters parameters = command.getParameters();

            if (continuousStepGenerator != null)
            {
               continuousStepGenerator.setFootstepTiming(parameters.getSwingDuration(), parameters.getTransferDuration());
               continuousStepGenerator.setSwingHeight(parameters.getSwingHeight());
               continuousStepGenerator.setFootstepsAreAdjustable(parameters.getStepsAreAdjustable());
               continuousStepGenerator.setStepWidths(parameters.getDefaultStepWidth(), parameters.getMinStepWidth(), parameters.getMaxStepWidth());
            }

         }

         if (!isOpen)
            walk = false;
      }

      public boolean isOpen()
      {
         return isOpen;
      }

      public DesiredVelocityProvider createDesiredVelocityProvider()
      {
         return new DesiredVelocityProvider()
         {
            @Override
            public Vector2DReadOnly getDesiredVelocity()
            {
               return desiredVelocity;
            }

            @Override
            public boolean isUnitVelocity()
            {
               return isUnitVelocities;
            }
         };
      }

      public DesiredTurningVelocityProvider createDesiredTurningVelocityProvider()
      {
         return new DesiredTurningVelocityProvider()
         {
            @Override
            public double getTurningVelocity()
            {
               return turningVelocity;
            }

            @Override
            public boolean isUnitVelocity()
            {
               return isUnitVelocities;
            }
         };
      }

      public BooleanProvider createWalkInputProvider()
      {
         return () -> walk;
      }
   }
}
