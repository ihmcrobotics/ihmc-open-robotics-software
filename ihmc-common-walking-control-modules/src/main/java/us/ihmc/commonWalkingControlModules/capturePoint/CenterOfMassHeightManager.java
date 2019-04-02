package us.ihmc.commonWalkingControlModules.capturePoint;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.CenterOfMassHeightControlState;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisAndCenterOfMassHeightControlState;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisHeightControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisHeightControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
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
import us.ihmc.yoVariables.registry.YoVariableRegistry;
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
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final StateMachine<PelvisHeightControlMode, PelvisAndCenterOfMassHeightControlState> stateMachine;
   private final YoEnum<PelvisHeightControlMode> requestedState;

   /** Manages the height of the robot by default, Tries to adjust the pelvis based on the nominal height requested **/
   private final CenterOfMassHeightControlState centerOfMassHeightControlState;

   /** User Controlled Pelvis Height Mode, tries to achieve a desired pelvis height regardless of the robot configuration**/
   private final PelvisHeightControlState pelvisHeightControlState;

   /** if the manager is in user mode before walking then stay in it while walking (PelvisHeightControlState) **/
   private final YoBoolean enableUserPelvisControlDuringWalking = new YoBoolean("centerOfMassHeightManagerEnableUserPelvisControlDuringWalking", registry);
   private final YoBoolean doPrepareForLocomotion = new YoBoolean("doPrepareCenterOfMassHeightForLocomotion", registry);

   private final boolean useStateMachine;

   public CenterOfMassHeightManager(HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters,
                                    YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      useStateMachine = !walkingControllerParameters.usePelvisHeightControllerOnly();

      // User mode
      pelvisHeightControlState = new PelvisHeightControlState(controllerToolbox, registry);

      if (useStateMachine)
      {
         // Normal control during walking
         YoDouble yoTime = controllerToolbox.getYoTime();
         String namePrefix = getClass().getSimpleName();
         requestedState = new YoEnum<>(namePrefix + "RequestedControlMode", registry, PelvisHeightControlMode.class, true);
         centerOfMassHeightControlState = new CenterOfMassHeightControlState(controllerToolbox, walkingControllerParameters, registry);

         stateMachine = setupStateMachine(namePrefix, yoTime);
      }
      else
      {
         requestedState = null;
         centerOfMassHeightControlState = null;
         stateMachine = null;
      }
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

      if (useStateMachine)
      {
         stateMachine.resetToInitialState();
         centerOfMassHeightControlState.initialize();
      }

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

   public void compute()
   {
      if (useStateMachine)
      {
         stateMachine.doActionAndTransition();
      }
      else
      {
         pelvisHeightControlState.doAction(Double.NaN);
      }
   }

   /**
    * sets the height manager up for walking
    * If we're in user mode and not allowed to stay that way while walking then switch out of user mode
    */
   public void prepareForLocomotion()
   {
      if (!doPrepareForLocomotion.getValue())
         return;

      if (!useStateMachine)
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
      if (useStateMachine)
      {
         stateMachine.getCurrentState().initializeDesiredHeightToCurrent();
      }
      else
      {
         pelvisHeightControlState.initializeDesiredHeightToCurrent();
      }
   }

   /**
    * checks that the command is valid and switches to user mode
    * The controller will try to achieve the pelvis height regardless of the robot configuration
    * @param command - only the linear z portion of this command is used
    */
   public void handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      if (useStateMachine)
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
      else
      {
         enableUserPelvisControlDuringWalking.set(command.isEnableUserPelvisControlDuringWalking());
         pelvisHeightControlState.handlePelvisTrajectoryCommand(command);
      }
   }

   /**
    * switches to center of mass height controller, this is the standard height manager
    * the height in this command will be adjusted based on the legs
    * @param command
    */
   public void handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command)
   {
      if (useStateMachine)
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
      else
      {
         enableUserPelvisControlDuringWalking.set(command.isEnableUserPelvisControlDuringWalking());
         pelvisHeightControlState.handlePelvisHeightTrajectoryCommand(command);
      }
   }

   /**
    * set the desired height to walkingControllerParameters.nominalHeightAboveAnkle()
    * @param trajectoryTime
    */
   public void goHome(double trajectoryTime)
   {
      if (useStateMachine)
      {
         stateMachine.getCurrentState().goHome(trajectoryTime);
      }
      else
      {
         pelvisHeightControlState.goHome(trajectoryTime);
      }
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      if (useStateMachine)
      {
         stateMachine.getCurrentState().handleStopAllTrajectoryCommand(command);
      }
      else
      {
         pelvisHeightControlState.handleStopAllTrajectoryCommand(command);
      }
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      if (useStateMachine)
      {
         centerOfMassHeightControlState.setSupportLeg(supportLeg);
      }
   }

   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
   {
      if (useStateMachine)
      {
         centerOfMassHeightControlState.initialize(transferToAndNextFootstepsData, extraToeOffHeight);
      }
   }

   public double computeDesiredCoMHeightAcceleration(FrameVector2D desiredICPVelocity, boolean isInDoubleSupport, double omega0, boolean isRecoveringFromPush,
                                                     FeetManager feetManager)
   {
      if (useStateMachine)
      {
         return stateMachine.getCurrentState().computeDesiredCoMHeightAcceleration(desiredICPVelocity, isInDoubleSupport, omega0, isRecoveringFromPush,
                                                                                   feetManager);
      }
      else
      {
         return pelvisHeightControlState.computeDesiredCoMHeightAcceleration(desiredICPVelocity, isInDoubleSupport, omega0, isRecoveringFromPush, feetManager);
      }
   }

   public boolean hasBeenInitializedWithNextStep()
   {
      if (useStateMachine)
      {
         return centerOfMassHeightControlState.hasBeenInitializedWithNextStep();
      }
      else
      {
         return true;
      }
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      if (useStateMachine)
      {
         return stateMachine.getCurrentState().getFeedbackControlCommand();
      }
      else
      {
         return pelvisHeightControlState.getFeedbackControlCommand();
      }
   }

   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      if (useStateMachine)
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
      else
      {
         return pelvisHeightControlState.getFeedbackControlCommand();
      }
   }

   public boolean getControlHeightWithMomentum()
   {
      if (useStateMachine)
      {
         return stateMachine.getCurrentStateKey().equals(PelvisHeightControlMode.WALKING_CONTROLLER);
      }
      else
      {
         return false;
      }
   }

   public void setComHeightGains(PIDGainsReadOnly walkingControllerComHeightGains, DoubleProvider walkingControllerMaxComHeightVelocity,
                                 PIDGainsReadOnly userModeComHeightGains)
   {
      if (useStateMachine)
      {
         centerOfMassHeightControlState.setGains(walkingControllerComHeightGains, walkingControllerMaxComHeightVelocity);
      }
      pelvisHeightControlState.setGains(userModeComHeightGains);
   }

   public void step(Point3DReadOnly stanceFootPosition, Point3DReadOnly touchdownPosition, double swingTime, RobotSide swingSide, double toeOffHeight)
   {
      if (useStateMachine || enableUserPelvisControlDuringWalking.getBooleanValue())
      {
         return;
      }

      pelvisHeightControlState.step(stanceFootPosition, touchdownPosition, swingTime, swingSide, toeOffHeight);
   }

   public void transfer(Point3DReadOnly transferPosition, double transferTime)
   {
      transfer(transferPosition, transferTime, null, 0.0);
   }

   public void transfer(Point3DReadOnly transferPosition, double transferTime, RobotSide swingSide, double toeOffHeight)
   {
      if (useStateMachine || enableUserPelvisControlDuringWalking.getBooleanValue())
      {
         return;
      }

      pelvisHeightControlState.transfer(transferPosition, transferTime, swingSide, toeOffHeight);
   }

   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      if (useStateMachine)
         return stateMachine.getCurrentState().pollStatusToReport();
      else
         return pelvisHeightControlState.pollStatusToReport();
   }
}
