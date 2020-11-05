package us.ihmc.commonWalkingControlModules.capturePoint;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * this class manages the center of mass height or the pelvis height using the PelvisHeightTrajectoryCommand and PelvisTrajectoryCommand respectively
 * PelvisTrajectoryCommand is considered a special user command which gets forwarded to PelvisHeightControlState,
 * In this user mode, the controller won't change the height of the pelvis to ensure the legs don't reach singularities while swinging. You must
 * take the robot's configuration into account while using this command. The PelvisTrajectoryCommand also allows the user to enable User Pelvis Control During Walking.
 * If this is turned off, the controller will switch back to CenterOfMassHeightControlState during walking
 * Only the Z component of the PelvisTrajectoryCommand is used to control the pelvis height.
 *
 * The PelvisHeightTrajectoryCommand uses a pdController to compute the Linear Momentum Z and sends a momentum command to the controller core
 * If you want to the controller to manage the pelvis height while walking use the PelvisHeightTrajectoryCommand.
 */
public class CenterOfMassHeightManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final StateMachine<PelvisHeightControlMode, PelvisAndCenterOfMassHeightControlState> stateMachine;
   private final YoEnum<PelvisHeightControlMode> requestedState;

   /** Manages the height of the robot by default, Tries to adjust the pelvis based on the nominal height requested **/
   private final CenterOfMassHeightControlState centerOfMassHeightControlState;

   /** User Controlled Pelvis Height Mode, tries to achieve a desired pelvis height regardless of the robot configuration**/
   private final PelvisHeightControlState pelvisHeightControlState;

   /** if the manager is in user mode before walking then stay in it while walking (PelvisHeightControlState) **/
   private final YoBoolean enableUserPelvisControlDuringWalking = new YoBoolean("centerOfMassHeightManagerEnableUserPelvisControlDuringWalking", registry);
   private final YoBoolean doPrepareForLocomotion = new YoBoolean("doPrepareCenterOfMassHeightForLocomotion", registry);


   public CenterOfMassHeightManager(HighLevelHumanoidControllerToolbox controllerToolbox,
                                    WalkingControllerParameters walkingControllerParameters,
                                    YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      // User mode
      pelvisHeightControlState = new PelvisHeightControlState(controllerToolbox, registry);

      // Normal control during walking
      YoDouble yoTime = controllerToolbox.getYoTime();
      String namePrefix = getClass().getSimpleName();
      requestedState = new YoEnum<>(namePrefix + "RequestedControlMode", registry, PelvisHeightControlMode.class, true);
      centerOfMassHeightControlState = new CenterOfMassHeightControlState(controllerToolbox, walkingControllerParameters, registry);
      stateMachine = setupStateMachine(namePrefix, yoTime);
   }

   private StateMachine<PelvisHeightControlMode, PelvisAndCenterOfMassHeightControlState> setupStateMachine(String namePrefix, DoubleProvider timeProvider)
   {
      StateMachineFactory<PelvisHeightControlMode, PelvisAndCenterOfMassHeightControlState> factory = new StateMachineFactory<>(PelvisHeightControlMode.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(timeProvider);

      factory.addState(PelvisHeightControlMode.WALKING_CONTROLLER, centerOfMassHeightControlState);
      factory.addState(PelvisHeightControlMode.USER, pelvisHeightControlState);

      for (PelvisHeightControlMode from : PelvisHeightControlMode.values())
         factory.addRequestedTransition(from, requestedState);

      return factory.build(PelvisHeightControlMode.WALKING_CONTROLLER);
   }

   public void initialize()
   {
      enableUserPelvisControlDuringWalking.set(false);

      stateMachine.resetToInitialState();
      centerOfMassHeightControlState.initialize();
      pelvisHeightControlState.initialize();
   }

   /**
    * set the weights for user mode, CenterOfMassHeightControlState does not use this weight
    * @param weight
    */
   public void setPelvisTaskspaceWeights(Vector3DReadOnly weight)
   {
      pelvisHeightControlState.setWeights(weight);
   }

   public void setPrepareForLocomotion(boolean value)
   {
      doPrepareForLocomotion.set(value);
   }

   public void compute(FrameVector2DReadOnly desiredICPVelocity,
                       FrameVector2DReadOnly desiredCoMVelocity,
                       boolean isInDoubleSupport,
                       double omega0,
                       boolean isRecoveringFromPush,
                       FeetManager feetManager)
   {
      stateMachine.doActionAndTransition();
      stateMachine.getCurrentState()
                  .computeCoMHeightCommand(desiredICPVelocity,
                                           desiredCoMVelocity,
                                           isInDoubleSupport,
                                           omega0,
                                           isRecoveringFromPush,
                                           feetManager);
   }

   /**
    * sets the height manager up for walking
    * If we're in user mode and not allowed to stay that way while walking then switch out of user mode
    */
   public void prepareForLocomotion()
   {
      if (!doPrepareForLocomotion.getValue())
         return;

      if (enableUserPelvisControlDuringWalking.getBooleanValue())
         return;

      if (stateMachine.getCurrentStateKey().equals(PelvisHeightControlMode.USER))
      {
         //need to check if setting the actual to the desireds here is a bad idea, might be better to go from desired to desired
         centerOfMassHeightControlState.initializeDesiredHeightToCurrent();
         requestState(PelvisHeightControlMode.WALKING_CONTROLLER);
      }
   }

   private void requestState(PelvisHeightControlMode state)
   {
      if (stateMachine.getCurrentStateKey() != state)
      {
         requestedState.set(state);
      }
   }

   public void initializeDesiredHeightToCurrent()
   {
      stateMachine.getCurrentState().initializeDesiredHeightToCurrent();
   }

   /**
    * checks that the command is valid and switches to user mode
    * The controller will try to achieve the pelvis height regardless of the robot configuration
    * @param command - only the linear z portion of this command is used
    */
   public void handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      if (command.isEnableUserPelvisControl())
      {
         enableUserPelvisControlDuringWalking.set(command.isEnableUserPelvisControlDuringWalking());
         if (pelvisHeightControlState.handlePelvisTrajectoryCommand(command))
         {
            requestState(PelvisHeightControlMode.USER);
            return;
         }

         LogTools.info("pelvisHeightControlState failed to handle PelvisTrajectoryCommand");
         return;
      }

      centerOfMassHeightControlState.handlePelvisTrajectoryCommand(command);
      requestState(PelvisHeightControlMode.WALKING_CONTROLLER);
   }

   /**
    * switches to center of mass height controller, this is the standard height manager
    * the height in this command will be adjusted based on the legs
    * @param command
    */
   public void handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command)
   {
      if (command.isEnableUserPelvisControl())
      {
         enableUserPelvisControlDuringWalking.set(command.isEnableUserPelvisControlDuringWalking());
         if (pelvisHeightControlState.handlePelvisHeightTrajectoryCommand(command))
         {
            requestState(PelvisHeightControlMode.USER);
            return;
         }
         LogTools.info("pelvisHeightControlState failed to handle PelvisTrajectoryCommand");
         return;
      }

      centerOfMassHeightControlState.handlePelvisHeightTrajectoryCommand(command);
      requestState(PelvisHeightControlMode.WALKING_CONTROLLER);
   }

   /**
    * set the desired height to walkingControllerParameters.nominalHeightAboveAnkle()
    * @param trajectoryTime
    */
   public void goHome(double trajectoryTime)
   {
      stateMachine.getCurrentState().goHome(trajectoryTime);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      stateMachine.getCurrentState().handleStopAllTrajectoryCommand(command);
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      centerOfMassHeightControlState.setSupportLeg(supportLeg);
   }

   public void initialize(NewTransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
   {
      centerOfMassHeightControlState.initialize(transferToAndNextFootstepsData, extraToeOffHeight);
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   public FeedbackControlCommand<?> getHeightControlCommand()
   {
      return stateMachine.getCurrentState().getHeightControlCommand();
   }

   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (PelvisHeightControlMode mode : PelvisHeightControlMode.values())
      {
         PelvisAndCenterOfMassHeightControlState state = stateMachine.getState(mode);
         if (state != null && state.getFeedbackControlCommand() != null)
            ret.addCommand(state.getFeedbackControlCommand());
      }
      return ret;
   }

   public boolean getControlHeightWithMomentum()
   {
      return stateMachine.getCurrentStateKey().equals(PelvisHeightControlMode.WALKING_CONTROLLER);
   }

   public void setComHeightGains(PIDGainsReadOnly walkingControllerComHeightGains,
                                 DoubleProvider walkingControllerMaxComHeightVelocity,
                                 PIDGainsReadOnly userModeCoMHeightGains)
   {
      centerOfMassHeightControlState.setGains(walkingControllerComHeightGains, walkingControllerMaxComHeightVelocity);
      pelvisHeightControlState.setGains(userModeCoMHeightGains);
   }

   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return stateMachine.getCurrentState().pollStatusToReport();
   }
}
