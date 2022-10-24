package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.providers.DoubleProvider;

public interface HumanoidSteppingPluginFactory extends HighLevelHumanoidControllerPluginFactory
{
   StepGeneratorCommandInputManager getStepGeneratorCommandInputManager();

   void setFootStepAdjustment(FootstepAdjustment footstepAdjustment);


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
