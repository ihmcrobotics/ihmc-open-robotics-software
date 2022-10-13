package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class JoystickBasedSteppingPluginFactory implements HighLevelHumanoidControllerPluginFactory
{
   private final ComponentBasedFootstepDataMessageGeneratorFactory csgPluginFactory;
   private final VelocityBasedSteppingGeneratorFactory velocityPluginFactory;
   private final StepGeneratorCommandInputManager commandInputManager = new StepGeneratorCommandInputManager();

   public JoystickBasedSteppingPluginFactory()
   {
      this.csgPluginFactory = new ComponentBasedFootstepDataMessageGeneratorFactory();
      this.velocityPluginFactory = new VelocityBasedSteppingGeneratorFactory();

      csgPluginFactory.setCSGCommandInputManager(commandInputManager);
      velocityPluginFactory.setCSGCommandInputManager(commandInputManager);
   }

   public void setFastWalkingInputParameters(VelocityBasedSteppingParameters parameters)
   {
      velocityPluginFactory.setSteppingParameters(parameters);
   }

   public StepGeneratorCommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }

   @Override
   public JoystickBasedSteppingPlugin buildPlugin(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      HighLevelHumanoidControllerToolbox controllerToolbox = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox();

      return buildPlugin(controllerToolbox.getReferenceFrames(),
                         controllerToolbox.getControlDT(),
                         controllerFactoryHelper.getWalkingControllerParameters(),
                         controllerFactoryHelper.getStatusMessageOutputManager(),
                         controllerFactoryHelper.getCommandInputManager(),
                         controllerToolbox.getYoGraphicsListRegistry(),
                         controllerToolbox.getContactableFeet(),
                         controllerToolbox.getYoTime());
   }

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
      VelocityBasedSteppingGenerator fastWalkingPlugin = velocityPluginFactory.buildPlugin(walkingStatusMessageOutputManager,
                                                                                           walkingCommandInputManager);

      JoystickBasedSteppingPlugin joystickBasedSteppingPlugin = new JoystickBasedSteppingPlugin(csgFootstepGenerator, fastWalkingPlugin);
      joystickBasedSteppingPlugin.setHighLevelStateChangeStatusListener(walkingStatusMessageOutputManager);

      return joystickBasedSteppingPlugin;
   }
}
