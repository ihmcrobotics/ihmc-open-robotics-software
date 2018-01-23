package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.ArrayList;
import java.util.List;

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
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.SymmetricYoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;
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
   private final GenericStateMachine<PelvisHeightControlMode, PelvisAndCenterOfMassHeightControlState> stateMachine;
   private final YoEnum<PelvisHeightControlMode> requestedState;

   /** Manages the height of the robot by default, Tries to adjust the pelvis based on the nominal height requested **/
   private final CenterOfMassHeightControlState centerOfMassHeightControlState;

   /** User Controlled Pelvis Height Mode, tries to achieve a desired pelvis height regardless of the robot configuration**/
   private final PelvisHeightControlState pelvisHeightControlState;

   /** if the manager is in user mode before walking then stay in it while walking (PelvisHeightControlState) **/
   private final YoBoolean enableUserPelvisControlDuringWalking = new YoBoolean("centerOfMassHeightManagerEnableUserPelvisControlDuringWalking", registry);

   private final YoPDGains comHeightGains;

   private final FramePose3D tempPose = new FramePose3D();
   private final FramePoint3D tempPosition = new FramePoint3D();

   public CenterOfMassHeightManager(HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters,
         YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      YoDouble yoTime = controllerToolbox.getYoTime();
      String namePrefix = getClass().getSimpleName();
      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", PelvisHeightControlMode.class, yoTime, registry);
      requestedState = new YoEnum<>(namePrefix + "RequestedControlMode", registry, PelvisHeightControlMode.class, true);

      PDGains gains = walkingControllerParameters.getCoMHeightControlGains();
      comHeightGains = new YoPDGains("_CoMHeight", registry);
      comHeightGains.createDerivativeGainUpdater(true);
      comHeightGains.set(gains);

      //some nasty copying, There is a gain frame issue in the feedback controller so we need to set the gains for x, y, and z
      SymmetricYoPIDSE3Gains pidGains = new SymmetricYoPIDSE3Gains("pelvisHeightManager", registry);
      pidGains.setProportionalGains(comHeightGains.getKp());
      pidGains.setDampingRatios(comHeightGains.getZeta());

      //this affects tracking in sim, not sure if it will be needed for the real robot
//      pidGains.setMaxFeedbackAndFeedbackRate(pdGains.getMaximumFeedback(), pdGains.getMaximumFeedbackRate());

      //User mode
      pelvisHeightControlState = new PelvisHeightControlState(pidGains, controllerToolbox, walkingControllerParameters, registry);

      //normal control
      centerOfMassHeightControlState = new CenterOfMassHeightControlState(comHeightGains, controllerToolbox, walkingControllerParameters, registry);

      setupStateMachine();
      enableUserPelvisControlDuringWalking.set(false);
   }

   private void setupStateMachine()
   {
      List<PelvisAndCenterOfMassHeightControlState> states = new ArrayList<>();
      states.add(centerOfMassHeightControlState);
      states.add(pelvisHeightControlState);

      for (PelvisAndCenterOfMassHeightControlState fromState : states)
      {
         for (PelvisAndCenterOfMassHeightControlState toState : states)
         {
            StateMachineTools.addRequestedStateTransition(requestedState, false, fromState, toState);
         }
         stateMachine.addState(fromState);
      }
   }

   public void initialize()
   {
      requestState(centerOfMassHeightControlState.getStateEnum());
   }

   /**
    * set the weights for user mode, CenterOfMassHeightControlState does not use this weight
    * @param weight
    */
   public void setPelvisTaskspaceWeights(Vector3DReadOnly weight)
   {
      pelvisHeightControlState.setWeights(weight);
   }

   public void compute()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   /**
    * sets the height manager up for walking
    * If we're in user mode and not allowed to stay that way while walking then switch out of user mode
    */
   public void prepareForLocomotion()
   {
      if (enableUserPelvisControlDuringWalking.getBooleanValue())
         return;

      if(stateMachine.getCurrentStateEnum().equals(PelvisHeightControlMode.USER))
      {
         //need to check if setting the actual to the desireds here is a bad idea, might be better to go from desired to desired
         centerOfMassHeightControlState.initializeDesiredHeightToCurrent();
         requestState(centerOfMassHeightControlState.getStateEnum());
      }
   }

   private void requestState(PelvisHeightControlMode state)
   {
      if (stateMachine.getCurrentStateEnum() != state)
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
      if(command.isEnableUserPelvisControl())
      {
         enableUserPelvisControlDuringWalking.set(command.isEnableUserPelvisControlDuringWalking());
         stateMachine.getCurrentState().getCurrentDesiredHeightOfDefaultControlFrame(tempPosition);

         tempPose.setToZero(tempPosition.getReferenceFrame());
         tempPose.setPosition(tempPosition);

         if (pelvisHeightControlState.handlePelvisTrajectoryCommand(command, tempPose))
         {
            requestState(pelvisHeightControlState.getStateEnum());
            return;
         }

         PrintTools.info("pelvisHeightControlState failed to handle PelvisTrajectoryCommand");
         return;
      }

      centerOfMassHeightControlState.handlePelvisTrajectoryCommand(command);
      requestState(centerOfMassHeightControlState.getStateEnum());
   }

   /**
    * switches to center of mass height controller, this is the standard height manager
    * the height in this command will be adjusted based on the legs
    * @param command
    */
   public void handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command)
   {
      if(command.isEnableUserPelvisControl())
      {
         enableUserPelvisControlDuringWalking.set(command.isEnableUserPelvisControlDuringWalking());
         stateMachine.getCurrentState().getCurrentDesiredHeightOfDefaultControlFrame(tempPosition);

         tempPose.setToZero(tempPosition.getReferenceFrame());
         tempPose.setPosition(tempPosition);

         if (pelvisHeightControlState.handlePelvisHeightTrajectoryCommand(command, tempPose))
         {
            requestState(pelvisHeightControlState.getStateEnum());
            return;
         }
         PrintTools.info("pelvisHeightControlState failed to handle PelvisTrajectoryCommand");
         return;
      }

      centerOfMassHeightControlState.handlePelvisHeightTrajectoryCommand(command);
      requestState(centerOfMassHeightControlState.getStateEnum());
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

   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
   {
      centerOfMassHeightControlState.initialize(transferToAndNextFootstepsData, extraToeOffHeight);
   }

   /**
    * The Desired acceleration of the COM. User mode returns 0, while the center of mass height manager returns the action from the internal pd controller over the height
    * @return
    */
   public double computeDesiredCoMHeightAcceleration(FrameVector2D desiredICPVelocity, boolean isInDoubleSupport, double omega0, boolean isRecoveringFromPush,
         FeetManager feetManager)
   {
      return stateMachine.getCurrentState().computeDesiredCoMHeightAcceleration(desiredICPVelocity, isInDoubleSupport, omega0, isRecoveringFromPush, feetManager);
   }

   public boolean hasBeenInitializedWithNextStep()
   {
      return centerOfMassHeightControlState.hasBeenInitializedWithNextStep();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
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

   /**
    * The center of mass height manager can control the pelvis in taskspace or the height of the Center of mass. When controlling
    * the pelvis height we don't need to control the height with a momentum command. If we are using the center of mass height control state
    * we do. When used in conjunction with the balance manager this will enable the Z component of the MomentumRateCommand that is sent
    * to the controller core
    * @return
    */
   public boolean getControlHeightWithMomentum()
   {
      // GW: revert this from returning true always for now to fix a test.
      return stateMachine.getCurrentStateEnum().equals(PelvisHeightControlMode.WALKING_CONTROLLER);
   }

   public YoPDGains getComHeightGains()
   {
      return comHeightGains;
   }
}
