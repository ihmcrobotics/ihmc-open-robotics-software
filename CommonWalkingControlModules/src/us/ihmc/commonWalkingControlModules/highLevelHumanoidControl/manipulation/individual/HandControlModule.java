package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual;

import static us.ihmc.robotics.stateMachines.StateMachineTools.addRequestedStateTransition;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ArmDesiredAccelerationsControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ArmTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HandComplianceControlParametersControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HandTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.ManipulationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.HandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.JointSpaceHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.LoadBearingHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TaskspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.UserControlModeState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage.ArmControlMode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.ConstantPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.DoubleTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.WrapperForPositionAndOrientationTrajectoryGenerators;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3TrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointListInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.stateMachines.GenericStateMachine;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateChangedListener;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.io.printing.PrintTools;

public class HandControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // JPratt. February 27, 2015: Added this since new Atlas was having trouble with network stuff.
   // It was sending 14,000 variables. This and others reduces it a bit when set to false.
   private static final boolean REGISTER_YOVARIABLES = true;

   private static final boolean DEBUG = false;

   private final YoVariableRegistry registry;

   private final GenericStateMachine<HandControlMode, HandControlState> stateMachine;
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);

   private final Map<OneDoFJoint, MultipleWaypointsTrajectoryGenerator> jointTrajectoryGenerators;

   private final ConstantPoseTrajectoryGenerator holdPoseTrajectoryGenerator;
   private final Map<OneDoFJoint, Double> defaultArmJointPositions;

   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator;
   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private final WrapperForPositionAndOrientationTrajectoryGenerators poseTrajectoryGenerator;

   private final TaskspaceHandPositionControlState taskSpacePositionControlState;
   private final JointSpaceHandControlState jointSpaceHandControlState;
   private final LoadBearingHandControlState loadBearingControlState;
   private final UserControlModeState userControlModeState;

   private final EnumYoVariable<HandControlMode> requestedState;
   private final OneDoFJoint[] jointsOriginal;
   /**
    * These joints are cloned from {@link #jointsOriginal}.
    * Their positions and velocities are set to match the current desireds.
    * They are used as an easy way to figure out what would be the desired hand pose when the arm is controlled in jointspace.
    * They are updated by calling {@link #updateJointsAtDesiredPosition()}.
    */
   private final OneDoFJoint[] jointsAtDesiredPosition;
   private LowLevelOneDoFJointDesiredDataHolderReadOnly newJointDesiredData = null;

   private final Map<OneDoFJoint, BooleanYoVariable> areJointsEnabled;
   private final String name;
   private final RobotSide robotSide;
   private final RigidBody chest, hand;

   private final FullHumanoidRobotModel fullRobotModel;

   private final double controlDT;

   private final StateChangedListener<HandControlMode> stateChangedlistener;
   private final BooleanYoVariable hasHandPoseStatusBeenSent;

   private final BooleanYoVariable areAllArmJointEnabled;

   private final boolean doPositionControl;

   private final FramePose currentDesiredPose = new FramePose();

   private final ReferenceFrame chestFrame;
   private final ReferenceFrame handControlFrame;

   private final Map<BaseForControl, ReferenceFrame> baseForControlToReferenceFrameMap = new HashMap<>();

   public HandControlModule(RobotSide robotSide, MomentumBasedController momentumBasedController, ArmControllerParameters armControlParameters,
         YoPIDGains jointspaceGains, YoSE3PIDGainsInterface taskspaceGains, YoVariableRegistry parentRegistry)
   {
      YoGraphicsListRegistry yoGraphicsListRegistry;
      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      name = namePrefix + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);

      if (REGISTER_YOVARIABLES)
      {
         yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
         parentRegistry.addChild(registry);
      }
      else
      {
         yoGraphicsListRegistry = null;
      }

      hasHandPoseStatusBeenSent = new BooleanYoVariable(namePrefix + "HasHandPoseStatusBeenSent", registry);
      hasHandPoseStatusBeenSent.set(false);

      areAllArmJointEnabled = new BooleanYoVariable(namePrefix + "AreAllArmJointEnabled", registry);
      areAllArmJointEnabled.set(true);

      stateChangedlistener = new StateChangedListener<HandControlMode>()
      {
         @Override
         public void stateChanged(State<HandControlMode> oldState, State<HandControlMode> newState, double time)
         {
            hasHandPoseStatusBeenSent.set(false);
         }
      };

      this.controlDT = momentumBasedController.getControlDT();

      fullRobotModel = momentumBasedController.getFullRobotModel();
      hand = fullRobotModel.getHand(robotSide);
      chest = fullRobotModel.getChest();

      chestFrame = chest.getBodyFixedFrame();
      handControlFrame = fullRobotModel.getHandControlFrame(robotSide);

      this.robotSide = robotSide;

      jointsOriginal = ScrewTools.createOneDoFJointPath(chest, hand);
      jointsAtDesiredPosition = ScrewTools.cloneOneDoFJointPath(chest, hand);

      requestedState = new EnumYoVariable<HandControlMode>(name + "RequestedState", "", registry, HandControlMode.class, true);
      requestedState.set(null);

      defaultArmJointPositions = armControlParameters.getDefaultArmJointPositions(fullRobotModel, robotSide);

      areJointsEnabled = new LinkedHashMap<>();
      for (OneDoFJoint oneDoFJoint : jointsOriginal)
      {
         areJointsEnabled.put(oneDoFJoint, new BooleanYoVariable(namePrefix + oneDoFJoint.getName() + "IsEnabled", registry));
      }

      jointTrajectoryGenerators = new LinkedHashMap<>();

      for (OneDoFJoint oneDoFJoint : jointsOriginal)
      {
         MultipleWaypointsTrajectoryGenerator multiWaypointTrajectoryGenerator = new MultipleWaypointsTrajectoryGenerator(oneDoFJoint.getName(), registry);
         jointTrajectoryGenerators.put(oneDoFJoint, multiWaypointTrajectoryGenerator);
      }

      DoubleYoVariable yoTime = momentumBasedController.getYoTime();
      stateMachine = new GenericStateMachine<>(name, name + "SwitchTime", HandControlMode.class, yoTime, registry);

      holdPoseTrajectoryGenerator = new ConstantPoseTrajectoryGenerator(name + "Hold", true, worldFrame, parentRegistry);

      positionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(namePrefix + "Hand", true, worldFrame, registry);
      orientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator(namePrefix + "Hand", true, worldFrame, registry);
      poseTrajectoryGenerator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);
      positionTrajectoryGenerator.registerNewTrajectoryFrame(chestFrame);
      orientationTrajectoryGenerator.registerNewTrajectoryFrame(chestFrame);

      doPositionControl = armControlParameters.doLowLevelPositionControl();
      String stateNamePrefix = namePrefix + "Hand";
      jointSpaceHandControlState = new JointSpaceHandControlState(stateNamePrefix, jointsOriginal, doPositionControl, momentumBasedController, jointspaceGains,
            controlDT, registry);
      taskSpacePositionControlState = new TaskspaceHandPositionControlState(stateNamePrefix, HandControlMode.TASK_SPACE_POSITION, doPositionControl,
            jointsOriginal, chest, hand, taskspaceGains, yoGraphicsListRegistry, registry);

      if (doPositionControl)
      {
         // TODO Not implemented for position control.
         loadBearingControlState = null;
         userControlModeState = null;
      }
      else
      {
         userControlModeState = new UserControlModeState(stateNamePrefix, robotSide, jointsOriginal, momentumBasedController, registry);
         RigidBody elevator = fullRobotModel.getElevator();
         loadBearingControlState = new LoadBearingHandControlState(stateNamePrefix, HandControlMode.LOAD_BEARING, robotSide, momentumBasedController, elevator, hand, registry);
      }

      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      baseForControlToReferenceFrameMap.put(BaseForControl.CHEST, chestFrame);
      baseForControlToReferenceFrameMap.put(BaseForControl.WALKING_PATH, referenceFrames.getMidFeetUnderPelvisFrame());
      baseForControlToReferenceFrameMap.put(BaseForControl.WORLD, worldFrame);

      for (ReferenceFrame frameToRegister : baseForControlToReferenceFrameMap.values())
      {
         positionTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
         orientationTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
      }

      setupStateMachine();
   }

   private void setupStateMachine()
   {
      addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, jointSpaceHandControlState);
      addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, taskSpacePositionControlState);

      addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, taskSpacePositionControlState);
      addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, jointSpaceHandControlState);

      stateMachine.addState(jointSpaceHandControlState);
      stateMachine.addState(taskSpacePositionControlState);

      if (!doPositionControl)
      {
         addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, loadBearingControlState);
         addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, loadBearingControlState);

         addRequestedStateTransition(requestedState, false, loadBearingControlState, taskSpacePositionControlState);
         addRequestedStateTransition(requestedState, false, loadBearingControlState, jointSpaceHandControlState);

         addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, userControlModeState);
         addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, userControlModeState);

         addRequestedStateTransition(requestedState, false, userControlModeState, taskSpacePositionControlState);
         addRequestedStateTransition(requestedState, false, userControlModeState, jointSpaceHandControlState);

         stateMachine.addState(loadBearingControlState);
         stateMachine.addState(userControlModeState);
      }

      stateMachine.attachStateChangedListener(stateChangedlistener);
   }

   public void setTaskspaceWeight(double weight)
   {
      taskSpacePositionControlState.setWeight(weight);
   }

   public void setJointspaceWeight(double weight)
   {
      jointSpaceHandControlState.setWeight(weight);
   }

   public void setUserModeWeight(double weight)
   {
      if (userControlModeState != null)
         userControlModeState.setWeight(weight);
   }

   public void initialize()
   {
   }

   public void doControl()
   {
      boolean isAtLeastOneJointDisabled = checkIfAtLeastOneJointIsDisabled();

      if (isAtLeastOneJointDisabled && areAllArmJointEnabled.getBooleanValue())
      {
         if (ManipulationControlModule.HOLD_POSE_IN_JOINT_SPACE)
            holdPositionInJointSpace();
         else
            holdPositionInBase();

         areAllArmJointEnabled.set(false);
      }
      else if (!isAtLeastOneJointDisabled)
      {
         areAllArmJointEnabled.set(true);
      }

      if (stateMachine.getCurrentStateEnum() == HandControlMode.USER_CONTROL_MODE && userControlModeState.isAbortUserControlModeRequested())
      {
         holdPositionInJointSpace();
      }

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   private boolean checkIfAtLeastOneJointIsDisabled()
   {
      for (int i = 0; i < jointsOriginal.length; i++)
      {
         areJointsEnabled.get(jointsOriginal[i]).set(jointsOriginal[i].isEnabled());
         if (!jointsOriginal[i].isEnabled())
            return true;
      }
      return false;
   }

   /**
    * In a best effort of having continuity in desireds between states, the low-level data can be used to update the {@link HandControlModule} with 
    * the most recent desired joint positions and velocities.
    * @param lowLevelOneDoFJointDesiredDataHolder Data that will be used to update the arm desired configuration. Only a read-only access is needed.
    */
   public void submitNewArmJointDesiredConfiguration(LowLevelOneDoFJointDesiredDataHolderReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      newJointDesiredData = lowLevelOneDoFJointDesiredDataHolder;
   }

   public boolean isDone()
   {
      return stateMachine.getCurrentState().isDone();
   }

   public void executeTaskspaceTrajectory(PoseTrajectoryGenerator poseTrajectory)
   {
      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      executeTaskSpaceTrajectory(poseTrajectory, selectionMatrix);
   }

   public void executeTaskSpaceTrajectory(PoseTrajectoryGenerator poseTrajectory, DenseMatrix64F selectionMatrix)
   {
      executeTaskSpaceTrajectory(poseTrajectory, selectionMatrix, Double.NaN, Double.NaN);
   }

   public void executeTaskSpaceTrajectory(PoseTrajectoryGenerator poseTrajectory, DenseMatrix64F selectionMatrix,
         double percentOfTrajectoryWithOrientationBeingControlled, double trajectoryTime)
   {
      taskSpacePositionControlState.setTrajectoryWithAngularControlQuality(poseTrajectory, percentOfTrajectoryWithOrientationBeingControlled, trajectoryTime);
      taskSpacePositionControlState.setSelectionMatrix(selectionMatrix);
      taskSpacePositionControlState.setControlFrameFixedInEndEffector(handControlFrame);

      requestedState.set(taskSpacePositionControlState.getStateEnum());
      stateMachine.checkTransitionConditions();
   }

   public void handleHandTrajectoryMessage(HandTrajectoryControllerCommand handTrajectoryMessage)
   {
      if (handTrajectoryMessage.getRobotSide() != robotSide)
      {
         PrintTools.warn(this, "Received a " + handTrajectoryMessage.getClass().getSimpleName() + " for the wrong side.");
         return;
      }

      handleHandTrajectoryMessage(handTrajectoryMessage.getBase(), handTrajectoryMessage);
   }

   private void handleHandTrajectoryMessage(BaseForControl base, TrajectoryPointListInterface<?, ? extends SE3TrajectoryPointInterface<?>> trajectoryPointList)
   {
      ReferenceFrame trajectoryFrame = baseForControlToReferenceFrameMap.get(base);
      if (trajectoryFrame == null)
         throw new RuntimeException("The base: " + base + " is not handled.");
      else if (DEBUG)
         PrintTools.info(this, "Executing hand trajectory in: " + base + ", found corresponding frame: " + trajectoryFrame);

      if (trajectoryPointList.getTrajectoryPoint(0).getTime() > 1.0e-5)
      {
         computeDesiredFramePose(trajectoryFrame, currentDesiredPose);
         currentDesiredPose.getPoseIncludingFrame(tempPosition, tempOrientation);
         tempPosition.changeFrame(worldFrame);
         tempLinearVelocity.setToZero(worldFrame);
         tempOrientation.changeFrame(worldFrame);
         tempAngularVelocity.setToZero(worldFrame);

         positionTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
         orientationTrajectoryGenerator.switchTrajectoryFrame(worldFrame);

         positionTrajectoryGenerator.clear();
         orientationTrajectoryGenerator.clear();

         positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempLinearVelocity);
         orientationTrajectoryGenerator.appendWaypoint(0.0, tempOrientation, tempAngularVelocity);
      }
      else
      {
         positionTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
         orientationTrajectoryGenerator.switchTrajectoryFrame(worldFrame);

         positionTrajectoryGenerator.clear();
         orientationTrajectoryGenerator.clear();
      }

      positionTrajectoryGenerator.appendWaypoints(trajectoryPointList);
      orientationTrajectoryGenerator.appendWaypoints(trajectoryPointList);

      positionTrajectoryGenerator.changeFrame(trajectoryFrame);
      orientationTrajectoryGenerator.changeFrame(trajectoryFrame);

      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();

      executeTaskspaceTrajectory(poseTrajectoryGenerator);
   }

   public void handleArmTrajectoryMessage(ArmTrajectoryControllerCommand armTrajectoryMessage)
   {
      if (armTrajectoryMessage.getRobotSide() != robotSide)
      {
         PrintTools.warn(this, "Received a " + armTrajectoryMessage.getClass().getSimpleName() + " for the wrong side.");
         return;
      }

      if (!checkJointspaceTrajectoryPointLists(armTrajectoryMessage.getTrajectoryPointLists()))
         return;

      updateJointsAtDesiredPosition();

      int numberOfJoints = armTrajectoryMessage.getNumberOfJoints();

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJoint joint = jointsOriginal[jointIndex];
         MultipleWaypointsTrajectoryGenerator trajectoryGenerator = jointTrajectoryGenerators.get(joint);
         trajectoryGenerator.clear();

         if (armTrajectoryMessage.getJointTrajectoryPoint(jointIndex, 0).getTime() > 1.0e-5)
         {
            double initialPosition = jointsAtDesiredPosition[jointIndex].getQ();
            double initialVelocity = jointsAtDesiredPosition[jointIndex].getQd();
            trajectoryGenerator.appendWaypoint(0.0, initialPosition, initialVelocity);
         }

         SimpleTrajectoryPoint1DList jointTrajectory = armTrajectoryMessage.getJointTrajectoryPointList(jointIndex);
         trajectoryGenerator.appendWaypoints(jointTrajectory);
         trajectoryGenerator.initialize();
      }

      executeJointspaceTrajectory(jointTrajectoryGenerators);
   }

   public void handleArmDesiredAccelerationsMessage(ArmDesiredAccelerationsControllerCommand armDesiredAccelerationsMessage)
   {
      if (!checkArmDesiredAccelerationsMessage(armDesiredAccelerationsMessage))
         return;

      if (userControlModeState == null)
         return;

      switch (armDesiredAccelerationsMessage.getArmControlMode())
      {
      case IHMC_CONTROL_MODE:
         if (stateMachine.getCurrentStateEnum() == HandControlMode.USER_CONTROL_MODE)
            holdPositionInJointSpace();
         return;
      case USER_CONTROL_MODE:
         userControlModeState.handleArmDesiredAccelerationsMessage(armDesiredAccelerationsMessage);
         requestedState.set(userControlModeState.getStateEnum());
         stateMachine.checkTransitionConditions();
         return;
      default:
         throw new RuntimeException("Unknown ArmControlMode: " + armDesiredAccelerationsMessage.getArmControlMode());
      }
   }

   public void handleHandComplianceControlParametersMessage(HandComplianceControlParametersControllerCommand message)
   {
      PrintTools.error(this, "HandComplianceControlParametersControllerCommand is not supported anymore. Needs to be reimplememted.");
   }

   private <T extends TrajectoryPointListInterface<?, ? extends OneDoFTrajectoryPointInterface<?>>> boolean checkJointspaceTrajectoryPointLists(RecyclingArrayList<T> trajectoryPointLists)
   {
      if (trajectoryPointLists.size() != jointsOriginal.length)
         return false;

      for (int jointIndex = 0; jointIndex < trajectoryPointLists.size(); jointIndex++)
      {
         if (!checkJointspaceTrajectoryPointList(jointsOriginal[jointIndex], trajectoryPointLists.get(jointIndex)))
            return false;
      }

      return true;
   }

   private boolean checkJointspaceTrajectoryPointList(OneDoFJoint joint, TrajectoryPointListInterface<?, ? extends OneDoFTrajectoryPointInterface<?>> trajectoryPointList)
   {
      for (int i = 0; i < trajectoryPointList.getNumberOfTrajectoryPoints(); i++)
      {
         double waypointPosition = trajectoryPointList.getTrajectoryPoint(i).getPosition();
         double jointLimitLower = joint.getJointLimitLower();
         double jointLimitUpper = joint.getJointLimitUpper();
         if (!MathTools.isInsideBoundsInclusive(waypointPosition, jointLimitLower, jointLimitUpper))
            return false;
      }
      return true;
   }

   private boolean checkArmDesiredAccelerationsMessage(ArmDesiredAccelerationsControllerCommand armDesiredAccelerationsMessage)
   {
      if (armDesiredAccelerationsMessage.getRobotSide() != robotSide)
      {
         PrintTools.warn(this, "Received a " + armDesiredAccelerationsMessage.getClass().getSimpleName() + " for the wrong side.");
         return false;
      }

      if (armDesiredAccelerationsMessage.getArmControlMode() == ArmControlMode.USER_CONTROL_MODE
            && armDesiredAccelerationsMessage.getNumberOfJoints() != jointsOriginal.length)
         return false;

      return true;
   }

   private final FramePoint tempPosition = new FramePoint();
   private final FrameVector tempLinearVelocity = new FrameVector();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameOrientation tempOrientation = new FrameOrientation();

   private void computeDesiredFramePose(ReferenceFrame trajectoryFrame, FramePose desiredPoseToPack)
   {
      computeDesiredFramePose(trajectoryFrame, handControlFrame, desiredPoseToPack);
   }

   private final RigidBodyTransform oldTrackingFrameTransform = new RigidBodyTransform();
   private final RigidBodyTransform newTrackingFrameTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformFromNewTrackingFrameToOldTrackingFrame = new RigidBodyTransform();

   private void computeDesiredFramePose(ReferenceFrame trajectoryFrame, ReferenceFrame newControlFrame, FramePose desiredPoseToPack)
   {
      ReferenceFrame oldControlFrame;

      if (stateMachine.getCurrentState() instanceof TaskspaceHandPositionControlState)
      {
         taskSpacePositionControlState.getDesiredPose(desiredPoseToPack);
         oldControlFrame = taskSpacePositionControlState.getTrackingFrame();
      }
      else if (stateMachine.getCurrentState() instanceof JointSpaceHandControlState)
      {
         updateJointsAtDesiredPosition();
         ReferenceFrame desiredEndEffectorFrame = jointsAtDesiredPosition[jointsAtDesiredPosition.length - 1].getSuccessor().getBodyFixedFrame();
         oldControlFrame = desiredEndEffectorFrame;
         desiredPoseToPack.setToZero(desiredEndEffectorFrame);
      }
      else
      {
         oldControlFrame = newControlFrame;
         desiredPoseToPack.setToZero(newControlFrame);
      }

      if (newControlFrame != oldControlFrame)
      {
         desiredPoseToPack.getPose(oldTrackingFrameTransform);
         newControlFrame.getTransformToDesiredFrame(transformFromNewTrackingFrameToOldTrackingFrame, oldControlFrame);
         newTrackingFrameTransform.multiply(oldTrackingFrameTransform, transformFromNewTrackingFrameToOldTrackingFrame);
         desiredPoseToPack.setPose(newTrackingFrameTransform);
      }

      desiredPoseToPack.changeFrame(trajectoryFrame);
   }

   private void updateJointsAtDesiredPosition()
   {
      if (newJointDesiredData != null)
      {
         for (int i = 0; i < jointsAtDesiredPosition.length; i++)
         {
            OneDoFJoint jointAtDesiredPosition = jointsAtDesiredPosition[i];
            double q = jointsOriginal[i].getQ();
            double qd = jointsOriginal[i].getQd();

            if (newJointDesiredData.hasDataForJoint(jointsOriginal[i]))
            {
               LowLevelJointDataReadOnly jointDesiredData = newJointDesiredData.getLowLevelJointData(jointsOriginal[i]);
               
               double qDesired = jointDesiredData.hasDesiredPosition() ? jointDesiredData.getDesiredPosition() : q;
               jointAtDesiredPosition.setQ(qDesired);
               
               double qdDesired = jointDesiredData.hasDesiredVelocity() ? jointDesiredData.getDesiredVelocity() : qd;
               jointAtDesiredPosition.setQd(qdDesired);
            }
            else
            {
               jointAtDesiredPosition.setQ(q);
               jointAtDesiredPosition.setQd(qd);
            }
         }

         newJointDesiredData = null;
      }
      else if (stateMachine.getCurrentStateEnum() == HandControlMode.JOINT_SPACE && jointSpaceHandControlState.hasJointTrajectories())
      {
         for (int i = 0; i < jointsAtDesiredPosition.length; i++)
         {
            jointsAtDesiredPosition[i].setQ(jointSpaceHandControlState.getJointDesiredPosition(jointsOriginal[i]));
            jointsAtDesiredPosition[i].setQd(jointSpaceHandControlState.getJointDesiredVelocity(jointsOriginal[i]));
         }
      }
      else
      {
         for (int i = 0; i < jointsAtDesiredPosition.length; i++)
         {
            jointsAtDesiredPosition[i].setQ(jointsOriginal[i].getQ());
            jointsAtDesiredPosition[i].setQd(jointsOriginal[i].getQd());
         }
      }

      jointsAtDesiredPosition[0].updateFramesRecursively();
   }

   public void requestLoadBearing()
   {
      if (doPositionControl)
      {
         PrintTools.error("Cannot do load bearing when the arms are position controlled.");
         return;
      }
      requestedState.set(loadBearingControlState.getStateEnum());
   }

   private void executeJointspaceTrajectory(Map<OneDoFJoint, ? extends DoubleTrajectoryGenerator> trajectoryGenrators)
   {
      jointSpaceHandControlState.setTrajectories(trajectoryGenrators);
      requestedState.set(jointSpaceHandControlState.getStateEnum());
      stateMachine.checkTransitionConditions();
   }

   public boolean isLoadBearing()
   {
      if (doPositionControl)
         return false;
      return stateMachine.getCurrentStateEnum() == HandControlMode.LOAD_BEARING;
   }

   public void holdPositionInBase()
   {
      computeDesiredFramePose(chest.getBodyFixedFrame(), currentDesiredPose);

      holdPoseTrajectoryGenerator.registerAndSwitchFrame(chest.getBodyFixedFrame());
      holdPoseTrajectoryGenerator.setConstantPose(currentDesiredPose);

      executeTaskspaceTrajectory(holdPoseTrajectoryGenerator);
   }

   public void holdPositionInJointSpace()
   {
      for (OneDoFJoint oneDoFJoint : jointsOriginal)
      {
         MultipleWaypointsTrajectoryGenerator trajectory = jointTrajectoryGenerators.get(oneDoFJoint);
         trajectory.clear();
         trajectory.appendWaypoint(0.0, oneDoFJoint.getQ(), 0.0);
         trajectory.initialize();
      }
      executeJointspaceTrajectory(jointTrajectoryGenerators);
   }

   public void goToDefaultState(double trajectoryTime)
   {
      updateJointsAtDesiredPosition();

      for (int jointIndex = 0; jointIndex < jointsOriginal.length; jointIndex++)
      {
         OneDoFJoint oneDoFJoint = jointsOriginal[jointIndex];
         MultipleWaypointsTrajectoryGenerator trajectory = jointTrajectoryGenerators.get(oneDoFJoint);
         trajectory.clear();
         double initialPosition = jointsAtDesiredPosition[jointIndex].getQ();
         double initialVelocity = jointsAtDesiredPosition[jointIndex].getQd();
         trajectory.appendWaypoint(0.0, initialPosition, initialVelocity);
         trajectory.appendWaypoint(trajectoryTime, defaultArmJointPositions.get(oneDoFJoint), 0.0);
         trajectory.initialize();
      }
      executeJointspaceTrajectory(jointTrajectoryGenerators);
   }

   public void resetJointIntegrators()
   {
      for (OneDoFJoint joint : jointsOriginal)
         joint.resetIntegrator();
   }

   public boolean isControllingPoseInWorld()
   {
      State<HandControlMode> currentState = stateMachine.getCurrentState();

      if (currentState == taskSpacePositionControlState)
         return taskSpacePositionControlState.getReferenceFrame() == worldFrame;

      return false;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return stateMachine.getCurrentState().getInverseDynamicsCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   public LowLevelOneDoFJointDesiredDataHolderReadOnly getLowLevelJointDesiredData()
   {
      return stateMachine.getCurrentState().getLowLevelJointDesiredData();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (HandControlMode mode : HandControlMode.values())
      {
         HandControlState state = stateMachine.getState(mode);
         if (state != null && state.getFeedbackControlCommand() != null)
            ret.addCommand(state.getFeedbackControlCommand());
      }
      return ret;
   }
}
