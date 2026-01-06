package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepValidityIndicator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeightMapCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class JoystickBasedSteppingPluginFactory implements HumanoidSteppingPluginFactory
{
   private final ComponentBasedFootstepDataMessageGeneratorFactory csgPluginFactory;
   private final VelocityBasedSteppingPluginFactory velocityPluginFactory;
   private final StepGeneratorCommandInputManager commandInputManager = new StepGeneratorCommandInputManager();
   private final StatusMessageOutputManager statusMessageOutputManager = new StatusMessageOutputManager(StepGeneratorAPIDefinition.getStepGeneratorSupportedStatusMessages());
   private final List<Updatable> updatables = new ArrayList<>();
   private final List<Consumer<HeightMapCommand>> heightMapCommandConsumers = new ArrayList<>();

   public JoystickBasedSteppingPluginFactory()
   {
      this.csgPluginFactory = new ComponentBasedFootstepDataMessageGeneratorFactory();
      this.velocityPluginFactory = new VelocityBasedSteppingPluginFactory();

      csgPluginFactory.setStepGeneratorCommandInputManager(commandInputManager);
      csgPluginFactory.setStepGeneratorStatusMessageOutputManager(statusMessageOutputManager);

      velocityPluginFactory.setStepGeneratorCommandInputManager(commandInputManager);
      velocityPluginFactory.setStepGeneratorStatusMessageOutputManager(statusMessageOutputManager);
   }

   public void setVelocitySteppingInputParameters(VelocityBasedSteppingParameters parameters)
   {
      velocityPluginFactory.setInputParameters(parameters);
   }

   @Override
   public void setFootStepAdjustment(FootstepAdjustment footstepAdjustment)
   {
      csgPluginFactory.setFootStepAdjustment(footstepAdjustment);
      velocityPluginFactory.setFootStepAdjustment(footstepAdjustment);
   }

   @Override
   public void addFootstepValidityIndicator(FootstepValidityIndicator footstepValidityIndicator)
   {
      csgPluginFactory.addFootstepValidityIndicator(footstepValidityIndicator);
      velocityPluginFactory.addFootstepValidityIndicator(footstepValidityIndicator);
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

   @Override
   public StepGeneratorCommandInputManager getStepGeneratorCommandInputManager()
   {
      return commandInputManager;
   }

   @Override
   public StatusMessageOutputManager getStepGeneratorStatusMessageOutputManager()
   {
      return statusMessageOutputManager;
   }

   @Override
   public JoystickBasedSteppingPlugin buildPlugin(FullHumanoidRobotModel robotModel,
                                                  CommonHumanoidReferenceFrames referenceFrames,
                                                  double updateDT,
                                                  WalkingControllerParameters walkingControllerParameters,
                                                  StatusMessageOutputManager walkingStatusMessageOutputManager,
                                                  CommandInputManager walkingCommandInputManager,
                                                  SideDependentList<? extends ContactableBody> contactableFeet,
                                                  DoubleProvider timeProvider)
   {
      ComponentBasedFootstepDataMessageGenerator csgFootstepGenerator = csgPluginFactory.buildPlugin(robotModel,
                                                                                                     referenceFrames,
                                                                                                     updateDT,
                                                                                                     walkingControllerParameters,
                                                                                                     walkingStatusMessageOutputManager,
                                                                                                     walkingCommandInputManager,
                                                                                                     contactableFeet,
                                                                                                     timeProvider);

      VelocityBasedSteppingPlugin fastWalkingPlugin = velocityPluginFactory.buildPlugin(robotModel,
                                                                                        referenceFrames,
                                                                                        updateDT,
                                                                                        walkingControllerParameters,
                                                                                        walkingStatusMessageOutputManager,
                                                                                        walkingCommandInputManager,
                                                                                        contactableFeet,
                                                                                        timeProvider);

      csgFootstepGenerator.setDesiredVelocityProvider(commandInputManager.createDesiredVelocityProvider());
      csgFootstepGenerator.setDesiredTurningVelocityProvider(commandInputManager.createDesiredTurningVelocityProvider());
      csgFootstepGenerator.setWalkInputProvider(commandInputManager.createWalkInputProvider());

      fastWalkingPlugin.setDesiredVelocityProvider(commandInputManager.createDesiredVelocityProvider());
      fastWalkingPlugin.setDesiredTurningVelocityProvider(commandInputManager.createDesiredTurningVelocityProvider());
      fastWalkingPlugin.setWalkInputProvider(commandInputManager.createWalkInputProvider());
      fastWalkingPlugin.setSwingHeightInputProvider(commandInputManager.createSwingHeightProvider());

      walkingStatusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class,
                                                                    commandInputManager::setHighLevelStateChangeStatusMessage);
      walkingStatusMessageOutputManager.attachStatusMessageListener(WalkingStatusMessage.class,
                                                                    commandInputManager::setWalkingStatus);
      walkingStatusMessageOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, commandInputManager::consumeFootstepStatus);

      //this is probably not the way the class was intended to be modified.
      commandInputManager.setCSG(csgFootstepGenerator.getContinuousStepGenerator());

      for (Consumer<HeightMapCommand> heightMapCommandConsumer : heightMapCommandConsumers)
         commandInputManager.addHeightMapCommandConsumer(heightMapCommandConsumer);

      JoystickBasedSteppingPlugin joystickBasedSteppingPlugin = new JoystickBasedSteppingPlugin(csgFootstepGenerator, fastWalkingPlugin, updatables);
      joystickBasedSteppingPlugin.setHighLevelStateChangeStatusListener(walkingStatusMessageOutputManager);

      return joystickBasedSteppingPlugin;
   }
}
