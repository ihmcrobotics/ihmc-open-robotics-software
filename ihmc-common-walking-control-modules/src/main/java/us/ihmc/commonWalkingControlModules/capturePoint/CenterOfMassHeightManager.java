package us.ihmc.commonWalkingControlModules.capturePoint;

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
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
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
   public static final String CONTROLLER_GAIN_SUFFIX = "_CoMHeight";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final StateMachine<PelvisHeightControlMode, PelvisAndCenterOfMassHeightControlState> stateMachine;
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
      requestedState = new YoEnum<>(namePrefix + "RequestedControlMode", registry, PelvisHeightControlMode.class, true);

      PDGains defaultGains = walkingControllerParameters.getCoMHeightControlGains();
      comHeightGains = new YoPDGains(CONTROLLER_GAIN_SUFFIX, registry);
      comHeightGains.createDerivativeGainUpdater(true);
      comHeightGains.set(defaultGains);

      // Some nasty copying: there is a gain frame issue in the feedback controller so we turn the height gain into a symmetric 3D gain.
      DefaultPID3DGains defaultGains3D = new DefaultPID3DGains();
      defaultGains3D.setProportionalGains(defaultGains.getKp());
      defaultGains3D.setDampingRatios(defaultGains.getZeta());
      defaultGains3D.setMaxFeedbackAndFeedbackRate(defaultGains.getMaximumFeedback(), defaultGains.getMaximumFeedbackRate());
      ParameterizedPID3DGains gains3D = new ParameterizedPID3DGains("UserPelvisHeight", GainCoupling.XYZ, false, defaultGains3D, registry);

      // User mode
      pelvisHeightControlState = new PelvisHeightControlState(gains3D, controllerToolbox, walkingControllerParameters, registry);

      // Normal control during walking
      centerOfMassHeightControlState = new CenterOfMassHeightControlState(comHeightGains, controllerToolbox, walkingControllerParameters, registry);

      stateMachine = setupStateMachine(namePrefix, yoTime);
      enableUserPelvisControlDuringWalking.set(false);
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
      stateMachine.resetToInitialState();
//      requestState(PelvisHeightControlMode.WALKING_CONTROLLER);
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
      stateMachine.doActionAndTransition();
   }

   /**
    * sets the height manager up for walking
    * If we're in user mode and not allowed to stay that way while walking then switch out of user mode
    */
   public void prepareForLocomotion()
   {
      if (enableUserPelvisControlDuringWalking.getBooleanValue())
         return;

      if(stateMachine.getCurrentStateKey().equals(PelvisHeightControlMode.USER))
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
      if(command.isEnableUserPelvisControl())
      {
         enableUserPelvisControlDuringWalking.set(command.isEnableUserPelvisControlDuringWalking());
         stateMachine.getCurrentState().getCurrentDesiredHeightOfDefaultControlFrame(tempPosition);

         tempPose.setToZero(tempPosition.getReferenceFrame());
         tempPose.setPosition(tempPosition);

         if (pelvisHeightControlState.handlePelvisTrajectoryCommand(command, tempPose))
         {
            requestState(PelvisHeightControlMode.USER);
            return;
         }

         PrintTools.info("pelvisHeightControlState failed to handle PelvisTrajectoryCommand");
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
      if(command.isEnableUserPelvisControl())
      {
         enableUserPelvisControlDuringWalking.set(command.isEnableUserPelvisControlDuringWalking());
         stateMachine.getCurrentState().getCurrentDesiredHeightOfDefaultControlFrame(tempPosition);

         tempPose.setToZero(tempPosition.getReferenceFrame());
         tempPose.setPosition(tempPosition);

         if (pelvisHeightControlState.handlePelvisHeightTrajectoryCommand(command, tempPose))
         {
            requestState(PelvisHeightControlMode.USER);
            return;
         }
         PrintTools.info("pelvisHeightControlState failed to handle PelvisTrajectoryCommand");
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
      return stateMachine.getCurrentStateKey().equals(PelvisHeightControlMode.WALKING_CONTROLLER);
   }

   public YoPDGains getComHeightGains()
   {
      return comHeightGains;
   }
}
