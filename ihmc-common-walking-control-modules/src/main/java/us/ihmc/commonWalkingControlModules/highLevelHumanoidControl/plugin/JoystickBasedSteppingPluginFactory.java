package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class JoystickBasedSteppingPluginFactory implements SteppingPluginFactory
{
   private final ComponentBasedFootstepDataMessageGeneratorFactory csgPluginFactory;
   private final VelocityBasedSteppingGeneratorFactory velocityPluginFactory;
   private final StepGeneratorCommandInputManager commandInputManager = new StepGeneratorCommandInputManager();

   public JoystickBasedSteppingPluginFactory()
   {
      this.csgPluginFactory = new ComponentBasedFootstepDataMessageGeneratorFactory();
      this.velocityPluginFactory = new VelocityBasedSteppingGeneratorFactory();

      csgPluginFactory.setStepGeneratorCommandInputManager(commandInputManager);
      velocityPluginFactory.setStepGeneratorCommandInputManager(commandInputManager);
   }

   public void setFastWalkingInputParameters(VelocityBasedSteppingParameters parameters)
   {
      velocityPluginFactory.setSteppingParameters(parameters);
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
      VelocityBasedSteppingGenerator fastWalkingPlugin = velocityPluginFactory.buildPlugin(referenceFrames,
                                                                                           updateDT,
                                                                                           walkingControllerParameters,
                                                                                           walkingStatusMessageOutputManager,
                                                                                           walkingCommandInputManager,
                                                                                           yoGraphicsListRegistry,
                                                                                           contactableFeet,
                                                                                           timeProvider);

      JoystickBasedSteppingPlugin joystickBasedSteppingPlugin = new JoystickBasedSteppingPlugin(csgFootstepGenerator, fastWalkingPlugin);
      joystickBasedSteppingPlugin.setHighLevelStateChangeStatusListener(walkingStatusMessageOutputManager);

      return joystickBasedSteppingPlugin;
   }
}
