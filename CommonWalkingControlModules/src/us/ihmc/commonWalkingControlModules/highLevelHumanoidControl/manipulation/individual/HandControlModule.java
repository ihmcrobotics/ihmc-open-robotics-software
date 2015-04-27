package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual;

import static us.ihmc.yoUtilities.stateMachines.StateMachineTools.addRequestedStateTransition;

import java.util.LinkedHashMap;
import java.util.Map;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.ManipulationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.JointSpaceHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.LoadBearingHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TaskspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TaskspaceToJointspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TrajectoryBasedTaskspaceHandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetProducers.HandPoseStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameMatrix3D;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.CurrentPositionProvider;
import us.ihmc.utilities.math.trajectories.providers.PositionProvider;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.controllers.YoPIDGains;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.yoUtilities.math.trajectories.CirclePoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.ConstantPoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.FinalApproachPoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.InitialClearancePoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.LeadInOutPoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.MultipleWaypointsOneDoFJointTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.OneDoFJointQuinticTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.OneDoFJointWayPointTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.StraightLinePoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.WrapperForPositionAndOrientationTrajectoryGenerators;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.yoUtilities.stateMachines.State;
import us.ihmc.yoUtilities.stateMachines.StateChangedListener;
import us.ihmc.yoUtilities.stateMachines.StateMachine;
import us.ihmc.yoUtilities.stateMachines.StateTransition;
import us.ihmc.yoUtilities.stateMachines.StateTransitionAction;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;

public class HandControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // JPratt. February 27, 2015: Added this since new Atlas was having trouble with network stuff.
   // It was sending 14,000 variables. This and others reduces it a bit when set to false.
   private static final boolean REGISTER_YOVARIABLES = true;

   private final YoVariableRegistry registry;

   private final StateMachine<HandControlState> stateMachine;
   private final RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule;
   private final TaskspaceToJointspaceCalculator handTaskspaceToJointspaceCalculator;
   private final FrameMatrix3D selectionFrameMatrix = new FrameMatrix3D();
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);

   private final Map<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator> quinticPolynomialTrajectoryGenerators;
   private final Map<OneDoFJoint, OneDoFJointWayPointTrajectoryGenerator> waypointsPolynomialTrajectoryGenerators;
   private final Map<OneDoFJoint, MultipleWaypointsOneDoFJointTrajectoryGenerator> wholeBodyWaypointsPolynomialTrajectoryGenerators;

   private final ConstantPoseTrajectoryGenerator holdPoseTrajectoryGenerator;
   private final StraightLinePoseTrajectoryGenerator straightLinePoseTrajectoryGenerator;
   private final CirclePoseTrajectoryGenerator circularPoseTrajectoryGenerator;
   private final FinalApproachPoseTrajectoryGenerator finalApproachPoseTrajectoryGenerator;
   private final InitialClearancePoseTrajectoryGenerator initialClearancePoseTrajectoryGenerator;
   private final LeadInOutPoseTrajectoryGenerator leadInOutPoseTrajectoryGenerator;

   private final PositionProvider currentHandPosition;
   private final YoVariableDoubleProvider trajectoryTimeProvider;
   private final MultipleWaypointsPositionTrajectoryGenerator waypointPositionTrajectoryGenerator;
   private final MultipleWaypointsOrientationTrajectoryGenerator waypointOrientationTrajectoryGenerator;
   private final WrapperForPositionAndOrientationTrajectoryGenerators wayPointPositionAndOrientationTrajectoryGenerator;


   private final BooleanYoVariable isExecutingHandStep;

   private final Map<OneDoFJoint, Double> jointCurrentPositionMap;

   private final TrajectoryBasedTaskspaceHandControlState taskSpacePositionControlState;
   private final JointSpaceHandControlState jointSpaceHandControlState;
   private final LoadBearingHandControlState loadBearingControlState;

   private final EnumYoVariable<HandControlState> requestedState;
   private final OneDoFJoint[] oneDoFJoints;
   private final String name;
   private final RobotSide robotSide;
   private final TwistCalculator twistCalculator;
   private final RigidBody chest, hand;

   private final FullRobotModel fullRobotModel;

   private final double controlDT;

   private final YoSE3PIDGains taskspaceGains, taskspaceLoadBearingGains;

   private final StateChangedListener<HandControlState> stateChangedlistener;
   private final HandPoseStatusProducer handPoseStatusProducer;
   private final BooleanYoVariable hasHandPoseStatusBeenSent;

   private final BooleanYoVariable areAllArmJointEnabled;

   public HandControlModule(RobotSide robotSide, MomentumBasedController momentumBasedController, ArmControllerParameters armControlParameters,
                            YoPIDGains jointspaceGains, YoSE3PIDGains taskspaceGains, YoSE3PIDGains taskspaceLoadBearingGains,
                            ControlStatusProducer controlStatusProducer, HandPoseStatusProducer handPoseStatusProducer, YoVariableRegistry parentRegistry)
   {
      YoGraphicsListRegistry yoGraphicsListRegistry;
      boolean visualize;

      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      name = namePrefix + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);

      if (REGISTER_YOVARIABLES)
      {
         yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
         visualize = true;
         parentRegistry.addChild(registry);
      }
      else
      {
         yoGraphicsListRegistry = null;
         visualize = false;
      }

      this.handPoseStatusProducer = handPoseStatusProducer;

      hasHandPoseStatusBeenSent = new BooleanYoVariable(namePrefix + "HasHandPoseStatusBeenSent", registry);
      hasHandPoseStatusBeenSent.set(false);

      areAllArmJointEnabled = new BooleanYoVariable(namePrefix + "AreAllArmJointEnabled", registry);
      areAllArmJointEnabled.set(true);

      stateChangedlistener = new StateChangedListener<HandControlState>()
      {
         @Override
         public void stateChanged(State<HandControlState> oldState, State<HandControlState> newState, double time)
         {
            hasHandPoseStatusBeenSent.set(false);
         }
      };

      this.controlDT = momentumBasedController.getControlDT();

      this.taskspaceGains = taskspaceGains;
      this.taskspaceLoadBearingGains = taskspaceLoadBearingGains;

      fullRobotModel = momentumBasedController.getFullRobotModel();
      hand = fullRobotModel.getHand(robotSide);
      chest = fullRobotModel.getChest();
      ReferenceFrame handFrame = hand.getBodyFixedFrame();
      int jacobianId = momentumBasedController.getOrCreateGeometricJacobian(chest, hand, handFrame);

      this.robotSide = robotSide;
      twistCalculator = momentumBasedController.getTwistCalculator();

      oneDoFJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(chest, hand), OneDoFJoint.class);

      requestedState = new EnumYoVariable<HandControlState>(name + "RequestedState", "", registry, HandControlState.class, true);
      requestedState.set(null);

      trajectoryTimeProvider = new YoVariableDoubleProvider(name + "TrajectoryTime", registry);
      currentHandPosition = new CurrentPositionProvider(handFrame);

      quinticPolynomialTrajectoryGenerators = new LinkedHashMap<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator>();

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         OneDoFJointQuinticTrajectoryGenerator trajectoryGenerator = new OneDoFJointQuinticTrajectoryGenerator(oneDoFJoint.getName() + "Trajectory",
                                                                        oneDoFJoint, trajectoryTimeProvider, registry);
         quinticPolynomialTrajectoryGenerators.put(oneDoFJoint, trajectoryGenerator);
      }

      waypointsPolynomialTrajectoryGenerators = new LinkedHashMap<>();

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         OneDoFJointWayPointTrajectoryGenerator trajectoryGenerator = new OneDoFJointWayPointTrajectoryGenerator(oneDoFJoint.getName() + "Trajectory",
                                                                         oneDoFJoint, trajectoryTimeProvider, 15, registry);
         waypointsPolynomialTrajectoryGenerators.put(oneDoFJoint, trajectoryGenerator);
      }

      wholeBodyWaypointsPolynomialTrajectoryGenerators = new LinkedHashMap<OneDoFJoint, MultipleWaypointsOneDoFJointTrajectoryGenerator>();

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         MultipleWaypointsOneDoFJointTrajectoryGenerator multiWaypointTrajectoryGenerator =
            new MultipleWaypointsOneDoFJointTrajectoryGenerator(oneDoFJoint.getName(), oneDoFJoint, registry);
         wholeBodyWaypointsPolynomialTrajectoryGenerators.put(oneDoFJoint, multiWaypointTrajectoryGenerator);
      }

      DoubleYoVariable simulationTime = momentumBasedController.getYoTime();
      stateMachine = new StateMachine<HandControlState>(name, name + "SwitchTime", HandControlState.class, simulationTime, registry);

      handSpatialAccelerationControlModule = new RigidBodySpatialAccelerationControlModule(name, twistCalculator, hand,
              fullRobotModel.getHandControlFrame(robotSide), controlDT, registry);
      handSpatialAccelerationControlModule.setGains(taskspaceGains);

      handTaskspaceToJointspaceCalculator = new TaskspaceToJointspaceCalculator(namePrefix, chest, hand, controlDT, registry);
      handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(fullRobotModel.getHandControlFrame(robotSide));
      handTaskspaceToJointspaceCalculator.setFullyConstrained();
      handTaskspaceToJointspaceCalculator.setPrivilegedJointPositionsToMidRange();
      // TODO These need to be extracted
      handTaskspaceToJointspaceCalculator.setJointAngleRegularizationWeight(5.0);
      handTaskspaceToJointspaceCalculator.setMaximumJointVelocities(5.0);
      handTaskspaceToJointspaceCalculator.setMaximumJointAccelerations(50.0);
      handTaskspaceToJointspaceCalculator.setMaximumTaskspaceVelocity(1.5, 0.5);

      holdPoseTrajectoryGenerator = new ConstantPoseTrajectoryGenerator(name + "Hold", true, worldFrame, parentRegistry);
      straightLinePoseTrajectoryGenerator = new StraightLinePoseTrajectoryGenerator(name + "StraightLine", true, worldFrame, registry, visualize,
              yoGraphicsListRegistry);
      finalApproachPoseTrajectoryGenerator = new FinalApproachPoseTrajectoryGenerator(name, true, worldFrame, registry, visualize, yoGraphicsListRegistry);
      initialClearancePoseTrajectoryGenerator = new InitialClearancePoseTrajectoryGenerator(name + "MoveAway", true, worldFrame, registry, visualize,
              yoGraphicsListRegistry);
      leadInOutPoseTrajectoryGenerator = new LeadInOutPoseTrajectoryGenerator(name + "Swing", true, worldFrame, registry, visualize, yoGraphicsListRegistry);

      circularPoseTrajectoryGenerator = new CirclePoseTrajectoryGenerator(name + "Circular", worldFrame, trajectoryTimeProvider, registry,
            yoGraphicsListRegistry);

      boolean doVelocityAtWaypoints = false;
      waypointPositionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator("handWayPointPosition", worldFrame, currentHandPosition, registry);
      waypointOrientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("handWayPointOrientation", 15, doVelocityAtWaypoints, true, worldFrame, registry);
      wayPointPositionAndOrientationTrajectoryGenerator = new WrapperForPositionAndOrientationTrajectoryGenerators(waypointPositionTrajectoryGenerator,
              waypointOrientationTrajectoryGenerator);

      loadBearingControlState = new LoadBearingHandControlState(namePrefix, HandControlState.LOAD_BEARING, robotSide, momentumBasedController,
              fullRobotModel.getElevator(), hand, jacobianId, registry);

      boolean doPositionControl = armControlParameters.doLowLevelPositionControl();
      jointSpaceHandControlState = new JointSpaceHandControlState(namePrefix, HandControlState.JOINT_SPACE, robotSide, oneDoFJoints, doPositionControl,
              momentumBasedController, armControlParameters, jointspaceGains, controlDT, registry);

      if (armControlParameters.useInverseKinematicsTaskspaceControl())
      {
         if (doPositionControl)
         {
            taskSpacePositionControlState = TaskspaceToJointspaceHandPositionControlState.createControlStateForPositionControlledJoints(namePrefix, chest, hand, jacobianId, jointspaceGains, registry);
         }
         else
         {
            taskSpacePositionControlState = TaskspaceToJointspaceHandPositionControlState.createControlStateForForceControlledJoints(namePrefix, momentumBasedController, chest, hand, jacobianId, jointspaceGains, registry);
         }
      }
      else
      {
         taskSpacePositionControlState = new TaskspaceHandPositionControlState(namePrefix, HandControlState.TASK_SPACE_POSITION, momentumBasedController,
                 jacobianId, chest, hand, yoGraphicsListRegistry, registry);
      }

      setupStateMachine();

      jointCurrentPositionMap = new LinkedHashMap<OneDoFJoint, Double>();

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         jointCurrentPositionMap.put(oneDoFJoint, oneDoFJoint.getQ());
      }

      isExecutingHandStep = new BooleanYoVariable(namePrefix + "DoingHandstep", registry);

   }

   private void setupStateMachine()
   {
      addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, jointSpaceHandControlState);
      addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, taskSpacePositionControlState);
      addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, loadBearingControlState);

      addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, taskSpacePositionControlState);
      addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, jointSpaceHandControlState);
      addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, loadBearingControlState);

      addRequestedStateTransition(requestedState, false, loadBearingControlState, taskSpacePositionControlState);
      addRequestedStateTransition(requestedState, false, loadBearingControlState, jointSpaceHandControlState);

      setupTransitionToSupport(taskSpacePositionControlState);

      stateMachine.addState(jointSpaceHandControlState);
      stateMachine.addState(taskSpacePositionControlState);
      stateMachine.addState(loadBearingControlState);

      stateMachine.attachStateChangedListener(stateChangedlistener);
   }

   private void setupTransitionToSupport(final State<HandControlState> fromState)
   {
      StateTransitionCondition swingToSupportCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return isExecutingHandStep.getBooleanValue() && fromState.isDone();
         }
      };
      StateTransitionAction swingToSupportAction = new StateTransitionAction()
      {
         @Override
         public void doTransitionAction()
         {
            isExecutingHandStep.set(false);
            handSpatialAccelerationControlModule.setGains(taskspaceLoadBearingGains);
         }
      };
      StateTransition<HandControlState> swingToSupportTransition = new StateTransition<HandControlState>(HandControlState.LOAD_BEARING,
                                                                      swingToSupportCondition, swingToSupportAction);
      fromState.addStateTransition(swingToSupportTransition);
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

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      checkAndSendHandPoseStatusIsCompleted();
   }

   private boolean checkIfAtLeastOneJointIsDisabled()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         if (!oneDoFJoints[i].isEnabled())
            return true;
      }
      return false;
   }

   private void checkAndSendHandPoseStatusIsCompleted()
   {
      boolean isExecutingHandPose = stateMachine.getCurrentStateEnum() == HandControlState.TASK_SPACE_POSITION;
      isExecutingHandPose |= stateMachine.getCurrentStateEnum() == HandControlState.JOINT_SPACE;

      if ((handPoseStatusProducer != null) && isExecutingHandPose && isDone() &&!hasHandPoseStatusBeenSent.getBooleanValue())
      {
         handPoseStatusProducer.sendCompletedStatus(robotSide);
         hasHandPoseStatusBeenSent.set(true);
      }
   }

   public boolean isDone()
   {
      return stateMachine.getCurrentState().isDone();
   }

   public void executeTaskSpaceTrajectory(PoseTrajectoryGenerator poseTrajectory)
   {
      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      executeTaskSpaceTrajectory(poseTrajectory, selectionMatrix);
   }

   public void executeTaskSpaceTrajectory(PoseTrajectoryGenerator poseTrajectory, DenseMatrix64F selectionMatrix)
   {
      executeTaskSpaceTrajectory(poseTrajectory, selectionMatrix, Double.NaN, Double.NaN);
   }

   public void executeTaskSpaceTrajectory(PoseTrajectoryGenerator poseTrajectory, DenseMatrix64F selectionMatrix, double percentOfTrajectoryWithOrientationBeingControlled, double trajectoryTime)
   {
      handSpatialAccelerationControlModule.setGains(taskspaceGains);
      taskSpacePositionControlState.setTrajectoryWithAngularControlQuality(poseTrajectory, percentOfTrajectoryWithOrientationBeingControlled, trajectoryTime);
      taskSpacePositionControlState.setControlModuleForForceControl(handSpatialAccelerationControlModule);
      taskSpacePositionControlState.setControlModuleForPositionControl(handTaskspaceToJointspaceCalculator);
      taskSpacePositionControlState.setSelectionMatrix(selectionMatrix);
      requestedState.set(taskSpacePositionControlState.getStateEnum());
      stateMachine.checkTransitionConditions();
      isExecutingHandStep.set(false);

      if (handPoseStatusProducer != null)
         handPoseStatusProducer.sendStartedStatus(robotSide);
   }

   private final FramePoint tempPosition = new FramePoint();
   private final FramePoint tempPositionOld = new FramePoint();
   private final FrameVector tempVelocity = new FrameVector();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameOrientation tempOrientationOld = new FrameOrientation();
   private final OrientationInterpolationCalculator orientationInterpolationCalculator = new OrientationInterpolationCalculator();

   public void moveInStraightLinesViaWayPoints(FramePose[] desiredPoses, double totalTrajectoryTime, ReferenceFrame trajectoryFrame)
   {
      waypointOrientationTrajectoryGenerator.registerAndSwitchFrame(trajectoryFrame);
      waypointPositionTrajectoryGenerator.clear();
      waypointOrientationTrajectoryGenerator.clear();

      tempVelocity.setToZero(worldFrame);
      tempAngularVelocity.setToZero(worldFrame);

      int numberOfPoses = desiredPoses.length;
      double trajectoryTimeOfEachPose = totalTrajectoryTime / numberOfPoses;
      double elapsedTimeSinceTrajectoryStart = 0.0;

      for (int i = 0; i < numberOfPoses; i++)
      {
         FramePose desiredPose = desiredPoses[i];
         desiredPose.getPoseIncludingFrame(tempPosition, tempOrientation);

         if ((i > 0) && (i < desiredPoses.length - 1))
         {
            tempVelocity.sub(tempPosition, tempPositionOld);
            tempVelocity.scale(1.0 / trajectoryTimeOfEachPose);

            orientationInterpolationCalculator.computeAngularVelocity(tempAngularVelocity, tempOrientationOld, tempOrientation, 1.0 / trajectoryTimeOfEachPose);
         }
         else
         {
            tempVelocity.scale(0.0);
            tempAngularVelocity.scale(0.0);
         }

         tempPositionOld.set(tempPosition);
         tempOrientationOld.set(tempOrientation);

         elapsedTimeSinceTrajectoryStart += trajectoryTimeOfEachPose;

         waypointPositionTrajectoryGenerator.appendWaypoint(elapsedTimeSinceTrajectoryStart, tempPosition, tempVelocity);
         waypointOrientationTrajectoryGenerator.appendWaypoint(elapsedTimeSinceTrajectoryStart, tempOrientation, tempAngularVelocity);
      }

      waypointPositionTrajectoryGenerator.initialize();
      waypointOrientationTrajectoryGenerator.initialize();

      executeTaskSpaceTrajectory(wayPointPositionAndOrientationTrajectoryGenerator);
   }

   public void moveInCircle(Point3d rotationAxisOriginInWorld, Vector3d rotationAxisInWorld, double rotationAngle, boolean controlHandAngleAboutAxis, double time)
   {
      // Limit arm joint motor speed based on Boston Dynamics Limit
      // of 700 rad/s input to the transmission.
      // For now, just set move time to at least 2 seconds
      if (time < 2.0)
         time = 2.0;

      trajectoryTimeProvider.set(time);

      circularPoseTrajectoryGenerator.setDesiredRotationAngle(rotationAngle);
      circularPoseTrajectoryGenerator.updateCircleFrame(rotationAxisOriginInWorld, rotationAxisInWorld);
      circularPoseTrajectoryGenerator.setInitialPoseToMatchReferenceFrame(fullRobotModel.getHandControlFrame(robotSide));
      circularPoseTrajectoryGenerator.setControlHandAngleAboutAxis(controlHandAngleAboutAxis);

      if (!controlHandAngleAboutAxis)
      {
         selectionFrameMatrix.setToZero(circularPoseTrajectoryGenerator.getCircleFrame());
         selectionFrameMatrix.setElement(0, 0, 1.0);
         selectionFrameMatrix.setElement(1, 1, 1.0);
         selectionFrameMatrix.setElement(2, 2, 0.0);
         selectionFrameMatrix.changeFrame(fullRobotModel.getHandControlFrame(robotSide));

         selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
         CommonOps.setIdentity(selectionMatrix);
         selectionFrameMatrix.getDenseMatrix(selectionMatrix, 0, 0);
      }

      executeTaskSpaceTrajectory(circularPoseTrajectoryGenerator, selectionMatrix);
   }

   public void moveInStraightLine(FramePose finalDesiredPose, double time, ReferenceFrame trajectoryFrame, boolean[] controlledOrientationAxes, double percentOfTrajectoryWithOrientationBeingControlled, double swingClearance)
   {
      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);

      if (controlledOrientationAxes != null)
      {
         for (int i = 2; i >= 0; i--)
         {
            if (!controlledOrientationAxes[i])
               MatrixTools.removeRow(selectionMatrix, i);
         }
      }

      if (stateMachine.getCurrentStateEnum() != HandControlState.LOAD_BEARING)
      {
         FramePose pose = computeDesiredFramePose(trajectoryFrame);
         straightLinePoseTrajectoryGenerator.registerAndSwitchFrame(trajectoryFrame);
         straightLinePoseTrajectoryGenerator.setInitialPose(pose);
         straightLinePoseTrajectoryGenerator.setFinalPose(finalDesiredPose);
         straightLinePoseTrajectoryGenerator.setTrajectoryTime(time);
         executeTaskSpaceTrajectory(straightLinePoseTrajectoryGenerator, selectionMatrix, percentOfTrajectoryWithOrientationBeingControlled, time);
      }
      else
      {
         loadBearingControlState.getContactNormalVector(initialDirection);
         moveAwayObject(finalDesiredPose, initialDirection, swingClearance, time, trajectoryFrame);
      }
   }

   private final FrameVector initialDirection = new FrameVector();
   private final FrameVector finalDirection = new FrameVector();


   public void moveTowardsObjectAndGoToSupport(FramePose finalDesiredPose, FrameVector surfaceNormal, double clearance, double time,
           ReferenceFrame trajectoryFrame, boolean goToSupportWhenDone, double holdPositionDuration)
   {
      finalDirection.setIncludingFrame(surfaceNormal);
      finalDirection.negate();

      moveTowardsObject(finalDesiredPose, finalDirection, clearance, time, trajectoryFrame);
      loadBearingControlState.setContactNormalVector(surfaceNormal);
      loadBearingControlState.setControlModuleForForceControl(handSpatialAccelerationControlModule);
      taskSpacePositionControlState.setHoldPositionDuration(holdPositionDuration);
      isExecutingHandStep.set(goToSupportWhenDone);
   }

   private void moveTowardsObject(FramePose finalDesiredPose, FrameVector finalDirection, double clearance, double time, ReferenceFrame trajectoryFrame)
   {
      if (stateMachine.getCurrentStateEnum() != HandControlState.LOAD_BEARING)
      {
         FramePose pose = computeDesiredFramePose(trajectoryFrame);
         finalApproachPoseTrajectoryGenerator.registerAndSwitchFrame(trajectoryFrame);
         finalApproachPoseTrajectoryGenerator.setInitialPose(pose);
         finalApproachPoseTrajectoryGenerator.setFinalPose(finalDesiredPose);
         finalApproachPoseTrajectoryGenerator.setTrajectoryTime(time);
         finalApproachPoseTrajectoryGenerator.setFinalApproach(finalDirection, clearance);
         executeTaskSpaceTrajectory(finalApproachPoseTrajectoryGenerator);
      }
      else
      {
         loadBearingControlState.getContactNormalVector(initialDirection);
         moveAwayObjectTowardsOtherObject(finalDesiredPose, initialDirection, finalDirection, clearance, time, trajectoryFrame);
      }
   }

   public void moveAwayObject(FramePose finalDesiredPose, FrameVector initialDirection, double clearance, double time, ReferenceFrame trajectoryFrame)
   {
      FramePose pose = computeDesiredFramePose(trajectoryFrame);
      initialClearancePoseTrajectoryGenerator.registerAndSwitchFrame(trajectoryFrame);
      initialClearancePoseTrajectoryGenerator.setInitialPose(pose);
      initialClearancePoseTrajectoryGenerator.setFinalPose(finalDesiredPose);
      initialClearancePoseTrajectoryGenerator.setInitialClearance(initialDirection, clearance);
      initialClearancePoseTrajectoryGenerator.setTrajectoryTime(time);
      executeTaskSpaceTrajectory(initialClearancePoseTrajectoryGenerator);
   }

   private void moveAwayObjectTowardsOtherObject(FramePose finalDesiredPose, FrameVector initialDirection, FrameVector finalDirection, double clearance,
           double time, ReferenceFrame trajectoryFrame)
   {
      FramePose pose = computeDesiredFramePose(trajectoryFrame);
      leadInOutPoseTrajectoryGenerator.registerAndSwitchFrame(trajectoryFrame);
      leadInOutPoseTrajectoryGenerator.setInitialLeadOut(pose, initialDirection, clearance);
      leadInOutPoseTrajectoryGenerator.setFinalLeadIn(finalDesiredPose, finalDirection, clearance);
      leadInOutPoseTrajectoryGenerator.setTrajectoryTime(time);
      executeTaskSpaceTrajectory(leadInOutPoseTrajectoryGenerator);
   }

   private FramePose computeDesiredFramePose(ReferenceFrame trajectoryFrame)
   {
      FramePose pose;
      if (stateMachine.getCurrentState() instanceof TrajectoryBasedTaskspaceHandControlState)
      {
         pose = taskSpacePositionControlState.getDesiredPose();
      }
      else
      {
         // FIXME: make this be based on desired joint angles
         pose = new FramePose(fullRobotModel.getHandControlFrame(robotSide));
      }

      pose.changeFrame(trajectoryFrame);

      return pose;
   }

   public void requestLoadBearing()
   {
      requestedState.set(loadBearingControlState.getStateEnum());
      loadBearingControlState.setControlModuleForForceControl(handSpatialAccelerationControlModule);
   }

   public void moveUsingQuinticSplines(Map<OneDoFJoint, Double> desiredJointPositions, double time)
   {
      moveUsingQuinticSplines(desiredJointPositions, time, true);
   }

   private void moveUsingQuinticSplines(Map<OneDoFJoint, Double> desiredJointPositions, double time, boolean checkTrajectoryTime)
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         if (!desiredJointPositions.containsKey(oneDoFJoints[i]))
            throw new RuntimeException("not all joint positions specified");
      }

      // Limit arm joint motor speed based on Boston Dynamics Limit
      // of 700 rad/s input to the transmission.
      // For now, just set move time to at least 2 seconds
      if (checkTrajectoryTime)
         time = Math.max(2.0, time);
      trajectoryTimeProvider.set(time);

      for (OneDoFJoint oneDoFJoint : desiredJointPositions.keySet())
      {
         double desiredPositions = desiredJointPositions.get(oneDoFJoint);

         // make sure that we do not set the desired position outside the joint limit
         desiredPositions = MathTools.clipToMinMax(desiredPositions, oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper());

         quinticPolynomialTrajectoryGenerators.get(oneDoFJoint).setFinalPosition(desiredPositions);
      }

      jointSpaceHandControlState.setTrajectories(quinticPolynomialTrajectoryGenerators);
      requestedState.set(jointSpaceHandControlState.getStateEnum());
      stateMachine.checkTransitionConditions();

      if (handPoseStatusProducer != null)
         handPoseStatusProducer.sendStartedStatus(robotSide);
   }
   
   public void initializeDesiredToCurrent()
   {
      trajectoryTimeProvider.set( 1.0 );

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         // check if the desired is in the limits
         OneDoFJoint oneDoFJoint = oneDoFJoints[i];
         double desiredPosition = oneDoFJoint.getqDesired();

         jointCurrentPositionMap.put( oneDoFJoint, oneDoFJoint.getQ() );

         quinticPolynomialTrajectoryGenerators.get(oneDoFJoint).setFinalPosition(desiredPosition);
         quinticPolynomialTrajectoryGenerators.get(oneDoFJoint).initialize();
      }

      jointSpaceHandControlState.setTrajectories(quinticPolynomialTrajectoryGenerators);
      requestedState.set(jointSpaceHandControlState.getStateEnum());
      stateMachine.checkTransitionConditions();

      if (handPoseStatusProducer != null)
         handPoseStatusProducer.sendStartedStatus(robotSide);
   }

   public void moveUsingCubicTrajectory(ArmJointTrajectoryPacket trajectoryPacket)
   {
      int armJoints = trajectoryPacket.trajectoryPoints[0].positions.length;
      if (armJoints != oneDoFJoints.length)
      {
         System.out.println(this.getClass().getSimpleName() + ": in ArmJointTrajectoryPacket packet contains " + armJoints + " joints - expected was "
                            + oneDoFJoints.length);
         return;
      }

      boolean success = true;
      int waypoints = trajectoryPacket.trajectoryPoints.length;
      for (int jointIdx = 0; jointIdx < oneDoFJoints.length; jointIdx++)
      {
         MultipleWaypointsOneDoFJointTrajectoryGenerator trajectoryGenerator = wholeBodyWaypointsPolynomialTrajectoryGenerators.get(oneDoFJoints[jointIdx]);
         trajectoryGenerator.clear();

         for (int i = 0; i < waypoints; i++)
         {
            double position = trajectoryPacket.trajectoryPoints[i].positions[jointIdx];
            double velocity = trajectoryPacket.trajectoryPoints[i].velocities[jointIdx];
            double time = trajectoryPacket.trajectoryPoints[i].time;

            // Safety for the new arms Boston Dynamic limit
            // ----------------------------------------------------
            // 1. limit the positions to be within joint limits
            position = MathTools.clipToMinMax(position, oneDoFJoints[jointIdx].getJointLimitLower(), oneDoFJoints[jointIdx].getJointLimitUpper());

            if (position != trajectoryPacket.trajectoryPoints[i].positions[jointIdx])
            {
               System.out.println(this.getClass().getSimpleName() + " limited the angle because of new arms safety");
            }

            // 2. limit the velocities at the waypoints to a maximum of 7rad/s
            velocity = MathTools.clipToMinMax(velocity, -7.0, 7.0);

            if (velocity != trajectoryPacket.trajectoryPoints[i].velocities[jointIdx])
            {
               System.out.println(this.getClass().getSimpleName() + " limited the joint velocity because of new arms safety");
            }

            // 3. make sure there are at least two seconds in between all waypoints
            double MAX_JOINT_VEL = 1.0;
            double maxDisplacement = 0.0;
            for (int j = 0; j < oneDoFJoints.length; j++)
            {
               double displacement;
               if (i == 0)
               {
                  ArmJointName jointName = fullRobotModel.getRobotSpecificJointNames().getArmJointNames()[j];
                  double current = fullRobotModel.getArmJoint(robotSide, jointName).getQ();
                  displacement = current - trajectoryPacket.trajectoryPoints[i].positions[j];
               }
               else
               {
                  displacement = trajectoryPacket.trajectoryPoints[i].positions[j] - trajectoryPacket.trajectoryPoints[i - 1].positions[j];
               }

               maxDisplacement = Math.max(maxDisplacement, Math.abs(displacement));
            }

            double minDeltaTime = maxDisplacement / MAX_JOINT_VEL;

            if (i == 0)
            {
               double deltaTime = trajectoryPacket.trajectoryPoints[i].time;
               if (deltaTime < minDeltaTime)
               {
                  time = minDeltaTime;

                  if (jointIdx == 0)
                  {
                     System.out.println(this.getClass().getSimpleName() + " set time in between waypoints to " + minDeltaTime + " seconds "
                                        + "for safety (this only affects the arms)");
                  }
               }
            }
            else
            {
               double deltaTime = trajectoryPacket.trajectoryPoints[i].time - trajectoryPacket.trajectoryPoints[i - 1].time;
               if (deltaTime < minDeltaTime)
               {
                  time = trajectoryPacket.trajectoryPoints[i - 1].time + minDeltaTime;

                  if (jointIdx == 0)
                  {
                     System.out.println(this.getClass().getSimpleName() + " set time in between waypoints to " + minDeltaTime + " seconds"
                                        + "for new arm safety (this only affects the arms)");
                  }
               }
            }

            // ----------------------------------------------------

            success &= trajectoryGenerator.appendWaypoint(time, position, velocity);
         }
      }

      if (success)
      {
         jointSpaceHandControlState.setTrajectories(wholeBodyWaypointsPolynomialTrajectoryGenerators);
         requestedState.set(jointSpaceHandControlState.getStateEnum());
         stateMachine.checkTransitionConditions();
      }
   }

   @Deprecated
   public void moveJointspaceWithWaypoints(Map<OneDoFJoint, double[]> desiredJointPositions, double time)
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         if (!desiredJointPositions.containsKey(oneDoFJoints[i]))
            throw new RuntimeException("not all joint positions specified");
      }

      // Limit arm joint motor speed based on Boston Dynamics Limit
      // of 700 rad/s input to the transmission.
      // For now, just set move time to at least 2 seconds
      // Note that for way points, this will be really really slow because
      // it will be TWO seconds between each way point
      time = Math.max(2.0, time);
      trajectoryTimeProvider.set(time);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         // check if the desired is in the limits
         OneDoFJoint oneDoFJoint = oneDoFJoints[i];
         double[] desiredPositions = desiredJointPositions.get(oneDoFJoint);

         for (int j = 0; j < desiredPositions.length; j++)
         {
            desiredPositions[j] = MathTools.clipToMinMax(desiredPositions[j], oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper());
         }

         waypointsPolynomialTrajectoryGenerators.get(oneDoFJoint).setDesiredPositions(desiredPositions);
      }

      jointSpaceHandControlState.setTrajectories(waypointsPolynomialTrajectoryGenerators);
      requestedState.set(jointSpaceHandControlState.getStateEnum());
      stateMachine.checkTransitionConditions();

//    if (handPoseStatusProducer != null)
//       handPoseStatusProducer.sendStartedStatus(robotSide);
   }

   public boolean isLoadBearing()
   {
      return stateMachine.getCurrentStateEnum() == HandControlState.LOAD_BEARING;
   }

   public void holdPositionInBase()
   {
      FramePose currentDesiredHandPose = computeDesiredFramePose(chest.getBodyFixedFrame());

      holdPoseTrajectoryGenerator.registerAndSwitchFrame(chest.getBodyFixedFrame());
      holdPoseTrajectoryGenerator.setConstantPose(currentDesiredHandPose);

      executeTaskSpaceTrajectory(holdPoseTrajectoryGenerator);
   }

   public void holdPositionInJointSpace()
   {
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         jointCurrentPositionMap.put(oneDoFJoint, oneDoFJoint.getQ());
      }

      moveUsingQuinticSplines(jointCurrentPositionMap, 0.0, false);
   }

   public boolean isControllingPoseInWorld()
   {
      State<HandControlState> currentState = stateMachine.getCurrentState();

      if (currentState == taskSpacePositionControlState)
         return taskSpacePositionControlState.getReferenceFrame() == worldFrame;

      return false;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }
}
