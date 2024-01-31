package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepPlanAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepValidityIndicator;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
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
   private final List<Updatable> updatables = new ArrayList<>();
   private final List<Consumer<PlanarRegionsListCommand>> planarRegionsListCommandConsumers = new ArrayList<>();


   public JoystickBasedSteppingPluginFactory()
   {
      this.csgPluginFactory = new ComponentBasedFootstepDataMessageGeneratorFactory();
      this.velocityPluginFactory = new VelocityBasedSteppingPluginFactory();

      //      csgPluginFactory.setStepGeneratorCommandInputManager(commandInputManager);
//      velocityPluginFactory.setStepGeneratorCommandInputManager(commandInputManager);
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
   public void setFootStepPlanAdjustment(FootstepPlanAdjustment footStepAdjustment)
   {
      csgPluginFactory.setFootStepPlanAdjustment(footStepAdjustment);
      velocityPluginFactory.setFootStepPlanAdjustment(footStepAdjustment);
   }

   @Override
   public void addFootstepValidityIndicator(FootstepValidityIndicator footstepValidityIndicator)
   {
      csgPluginFactory.addFootstepValidityIndicator(footstepValidityIndicator);
      velocityPluginFactory.addFootstepValidityIndicator(footstepValidityIndicator);
   }

   @Override
   public void addPlanarRegionsListCommandConsumer(Consumer<PlanarRegionsListCommand> planarRegionsListCommandConsumer)
   {
      planarRegionsListCommandConsumers.add(planarRegionsListCommandConsumer);
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
   public JoystickBasedSteppingPlugin buildPlugin(CommonHumanoidReferenceFrames referenceFrames,
                                                  double updateDT,
                                                  WalkingControllerParameters walkingControllerParameters,
                                                  StatusMessageOutputManager walkingStatusMessageOutputManager,
                                                  CommandInputManager walkingCommandInputManager,
                                                  YoGraphicsListRegistry yoGraphicsListRegistry,
                                                  SideDependentList<? extends ContactableBody> contactableFeet,
                                                  DoubleProvider timeProvider)
   {
      ComponentBasedFootstepDataMessageGenerator csgFootstepGenerator = csgPluginFactory.buildPlugin(referenceFrames,
                                                                                                     updateDT,
                                                                                                     walkingControllerParameters,
                                                                                                     walkingStatusMessageOutputManager,
                                                                                                     walkingCommandInputManager,
                                                                                                     yoGraphicsListRegistry,
                                                                                                     contactableFeet,
                                                                                                     timeProvider);
      VelocityBasedSteppingPlugin fastWalkingPlugin = velocityPluginFactory.buildPlugin(referenceFrames,
                                                                                        updateDT,
                                                                                        walkingControllerParameters,
                                                                                        walkingStatusMessageOutputManager,
                                                                                        walkingCommandInputManager,
                                                                                        yoGraphicsListRegistry,
                                                                                        contactableFeet,
                                                                                        timeProvider);

      csgFootstepGenerator.setDesiredVelocityProvider(commandInputManager.createDesiredVelocityProvider());
      csgFootstepGenerator.setDesiredTurningVelocityProvider(commandInputManager.createDesiredTurningVelocityProvider());
      csgFootstepGenerator.setWalkInputProvider(commandInputManager.createWalkInputProvider());
      csgFootstepGenerator.setSwingHeightInputProvider(commandInputManager.createSwingHeightProvider());


      fastWalkingPlugin.setDesiredVelocityProvider(commandInputManager.createDesiredVelocityProvider());
      fastWalkingPlugin.setDesiredTurningVelocityProvider(commandInputManager.createDesiredTurningVelocityProvider());
      fastWalkingPlugin.setWalkInputProvider(commandInputManager.createWalkInputProvider());
      fastWalkingPlugin.setSwingHeightInputProvider(commandInputManager.createSwingHeightProvider());

      walkingStatusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class,
                                                                    commandInputManager::setHighLevelStateChangeStatusMessage);
      walkingStatusMessageOutputManager.attachStatusMessageListener(WalkingStatusMessage.class,
                                                                    commandInputManager::setWalkingStatus);
      walkingStatusMessageOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, commandInputManager::consumeFootstepStatus);

      updatables.add(commandInputManager);

      //this is probably not the way the class was intended to be modified.
      commandInputManager.setCSG(csgFootstepGenerator.getContinuousStepGenerator());

      for (Consumer<PlanarRegionsListCommand> planarRegionsListCommandConsumer : planarRegionsListCommandConsumers)
         commandInputManager.addPlanarRegionsListCommandConsumer(planarRegionsListCommandConsumer);

      JoystickBasedSteppingPlugin joystickBasedSteppingPlugin = new JoystickBasedSteppingPlugin(csgFootstepGenerator, fastWalkingPlugin, updatables);
      joystickBasedSteppingPlugin.setHighLevelStateChangeStatusListener(walkingStatusMessageOutputManager);

      return joystickBasedSteppingPlugin;
   }
}
