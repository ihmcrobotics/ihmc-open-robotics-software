package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGeneratorParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DesiredTurningVelocityProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DesiredVelocityProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScript;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.StopWalkingMessenger;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
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
   private final OptionalFactoryField<HeightMap> heightMapField = new OptionalFactoryField<>("heightMap");
   private final OptionalFactoryField<CSGCommandInputManager> csgCommandInputManagerField = new OptionalFactoryField<>("csgCommandInputManagerField");
   private final OptionalFactoryField<Boolean> createSupportFootBasedFootstepAdjustment = new OptionalFactoryField<>("csgCreateSupportFootBasedFootstepAdjustment");
   /** This is used only when the support foot based footstep adjustment is created. */
   private final OptionalFactoryField<Boolean> adjustPitchAndRoll = new OptionalFactoryField<>("csgSupportFootBasedFootstepAdjustmentAdjustPitchAndRoll");
   private final OptionalFactoryField<FootstepAdjustment> secondaryFootstepAdjusterField = new OptionalFactoryField<>("csgFootstepAdjusterField");

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

   public void setHeightMap(HeightMap heightMap)
   {
      heightMapField.set(heightMap);
   }

   public void setFootStepAdjustment(FootstepAdjustment footStepAdjustment)
   {
      secondaryFootstepAdjusterField.set(footStepAdjustment);
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
      this.csgCommandInputManagerField.set(csgCommandInputManager);
      return csgCommandInputManager;
   }

   @Override
   public ComponentBasedFootstepDataMessageGenerator buildPlugin(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      HighLevelHumanoidControllerToolbox controllerToolbox = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox();

      return buildPlugin(controllerToolbox.getReferenceFrames(),
                         controllerFactoryHelper.getWalkingControllerParameters(),
                         controllerFactoryHelper.getStatusMessageOutputManager(),
                         controllerFactoryHelper.getCommandInputManager(),
                         controllerToolbox.getContactableFeet(),
                         controllerToolbox.getYoTime(),
                         controllerToolbox.getControlDT(),
                         controllerToolbox.getYoGraphicsListRegistry());
   }

   public ComponentBasedFootstepDataMessageGenerator buildPlugin(CommonHumanoidReferenceFrames referenceFrames,
                                                                 WalkingControllerParameters walkingControllerParameters,
                                                                 StatusMessageOutputManager statusMessageOutputManager,
                                                                 CommandInputManager commandInputManager,
                                                                 SideDependentList<? extends ContactableBody> contactableFeet,
                                                                 DoubleProvider timeProvider,
                                                                 double updateDT,
                                                                 YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (!registryField.hasValue())
         setRegistry();

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator(registryField.get());

      if (createSupportFootBasedFootstepAdjustment.hasValue() && createSupportFootBasedFootstepAdjustment.get())
         continuousStepGenerator.setSupportFootBasedFootstepAdjustment(adjustPitchAndRoll.hasValue() && adjustPitchAndRoll.get());
      if (secondaryFootstepAdjusterField.hasValue())
         continuousStepGenerator.addFootstepAdjustment(secondaryFootstepAdjusterField.get());
      continuousStepGenerator.setFootstepStatusListener(statusMessageOutputManager);
      continuousStepGenerator.setFrameBasedFootPoseProvider(referenceFrames.getSoleZUpFrames());
      continuousStepGenerator.configureWith(walkingControllerParameters);
      continuousStepGenerator.setStopWalkingMessenger(new StopWalkingMessenger()
      {
         PauseWalkingMessage message = HumanoidMessageTools.createPauseWalkingMessage(true);

         @Override
         public void submitStopWalkingRequest()
         {
            commandInputManager.submitMessage(message);
         }
      });
      continuousStepGenerator.setFootstepMessenger(commandInputManager::submitMessage);

      List<Updatable> updatables = new ArrayList<>();

      if (yoGraphicsListRegistry != null)
         continuousStepGenerator.setupVisualization(contactableFeet, yoGraphicsListRegistry);
      if (heightMapField.hasValue() && heightMapField.get() != null)
         continuousStepGenerator.setHeightMapBasedFootstepAdjustment(heightMapField.get());

      if (csgCommandInputManagerField.hasValue())
      {
         continuousStepGenerator.setDesiredVelocityProvider(csgCommandInputManagerField.get().createDesiredVelocityProvider());
         continuousStepGenerator.setDesiredTurningVelocityProvider(csgCommandInputManagerField.get().createDesiredTurningVelocityProvider());
         continuousStepGenerator.setWalkInputProvider(csgCommandInputManagerField.get().createWalkInputProvider());
         statusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class,
                                                                csgCommandInputManagerField.get()::setHighLevelStateChangeStatusMessage);
         updatables.add(csgCommandInputManagerField.get());

         //this is probably not the way the class was intended to be modified.
         csgCommandInputManagerField.get().setCSG(continuousStepGenerator);

      }
      else if (useHeadingAndVelocityScriptField.get())
      {
         HeadingAndVelocityEvaluationScript script = new HeadingAndVelocityEvaluationScript(updateDT,
                                                                                            timeProvider,
                                                                                            headingAndVelocityEvaluationScriptParametersField.get(),
                                                                                            registryField.get());
//         continuousStepGenerator.setDesiredTurningVelocityProvider(script.getDesiredTurningVelocityProvider());
         continuousStepGenerator.setDesiredVelocityProvider(new DesiredVelocityProvider()
                                                            {
            @Override
            public Vector2DReadOnly getDesiredVelocity()
            {
               return new Vector2D(0.4, 0.0);
            }
            
                                                            });
         updatables.add(script);
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
         return commands;
      }

      private void setHighLevelStateChangeStatusMessage(HighLevelStateChangeStatusMessage message)
      {
         currentController = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
      }

      @Override
      public void update(double time)
      {
         isOpen = currentController == HighLevelControllerName.WALKING;

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
