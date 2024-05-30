package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import java.util.function.Consumer;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepPlanAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepValidityIndicator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.providers.DoubleProvider;

public interface HumanoidSteppingPluginFactory extends HighLevelHumanoidControllerPluginFactory
{
   StepGeneratorCommandInputManager getStepGeneratorCommandInputManager();

   void setFootStepAdjustment(FootstepAdjustment footstepAdjustment);

   void setFootStepPlanAdjustment(FootstepPlanAdjustment footstepAdjustment);

   void addFootstepValidityIndicator(FootstepValidityIndicator footstepValidityIndicator);

   void addPlanarRegionsListCommandConsumer(Consumer<PlanarRegionsListCommand> planarRegionsListCommandConsumer);

   void addUpdatable(Updatable updatable);

   default void createStepGeneratorNetworkSubscriber(String robotName, RealtimeROS2Node realtimeROS2Node)
   {
      ROS2Topic<?> inputTopic = HumanoidControllerAPI.getInputTopic(robotName);
      StepGeneratorNetworkSubscriber stepGeneratorNetworkSubscriber = new StepGeneratorNetworkSubscriber(inputTopic,
                                                                                                      getStepGeneratorCommandInputManager(),
                                                                                                      realtimeROS2Node);

      stepGeneratorNetworkSubscriber.addMessageValidator(ControllerAPIDefinition.createDefaultMessageValidation());
   }

   @Override
   default HumanoidSteppingPlugin buildPlugin(HighLevelControllerFactoryHelper controllerFactoryHelper)
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

   HumanoidSteppingPlugin buildPlugin(CommonHumanoidReferenceFrames referenceFrames,
                                      double updateDT,
                                      WalkingControllerParameters walkingControllerParameters,
                                      StatusMessageOutputManager walkingStatusMessageOutputManager,
                                      CommandInputManager walkingCommandInputManager,
                                      YoGraphicsListRegistry yoGraphicsListRegistry,
                                      SideDependentList<? extends ContactableBody> contactableFeet,
                                      DoubleProvider timeProvider);
}
