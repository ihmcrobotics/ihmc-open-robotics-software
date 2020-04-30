package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTSleepState;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTStreamingState;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Map;

import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState.SLEEP;
import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState.STREAMING;
import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

public class StepConstraintToolboxController extends ToolboxController
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final StepConstraintCalculator stepConstraintCalculator;

   private final YoDouble time = new YoDouble("time", registry);

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   public StepConstraintToolboxController(StatusMessageOutputManager statusOutputManager,
                                          WalkingControllerParameters walkingControllerParameters,
                                          FullHumanoidRobotModel fullRobotModel,
                                          double gravityZ,
                                          YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.fullRobotModel = fullRobotModel;
      stepConstraintCalculator = new StepConstraintCalculator(walkingControllerParameters, fullRobotModel, time, gravityZ);

      isDone.set(false);
   }

   @Override
   public boolean initialize()
   {
      isDone.set(false);
      return true;
   }

   private long initialTimestamp = -1L;

   @Override
   public void updateInternal()
   {
      try
      {
         if (initialTimestamp == -1L)
            initialTimestamp = System.nanoTime();
         time.set(Conversions.nanosecondsToSeconds(System.nanoTime() - initialTimestamp));

         stepConstraintCalculator.update();
      }
      catch (Throwable e)
      {
         e.printStackTrace();

         try
         {
            reportMessage(MessageTools.createControllerCrashNotificationPacket(null, e));
         }
         catch (Exception e1)
         {
            e1.printStackTrace();
         }

         isDone.set(true);
      }
   }

   @Override
   public void notifyToolboxStateChange(ToolboxState newState)
   {
   }

   @Override
   public boolean isDone()
   {
      return isDone.getValue();
   }

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      stepConstraintCalculator.updateRobotConfigurationData(newConfigurationData);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      stepConstraintCalculator.updateCapturabilityBasedStatus(newStatus);
   }

   public void updateFootstepStatus(FootstepStatusMessage footstepStatusMessage)
   {
      stepConstraintCalculator.updateFootstepStatus(footstepStatusMessage);
   }

   public void updatePlanarRegions(PlanarRegionsListMessage planarRegionsListMessage)
   {
      stepConstraintCalculator.updatePlanarRegions(planarRegionsListMessage);
   }

   public double getTime()
   {
      return time.getDoubleValue();
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }
}
