package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual;

import static us.ihmc.robotics.stateMachines.StateMachineTools.addRequestedStateTransition;

import java.util.LinkedHashMap;
import java.util.Map;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.ManipulationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.JointSpaceHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.LoadBearingHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TaskspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TaskspaceToJointspaceHandForcefeedbackControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TaskspaceToJointspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TrajectoryBasedTaskspaceHandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetProducers.HandPoseStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.commonWalkingControlModules.trajectories.CirclePoseTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.FinalApproachPoseTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.InitialClearancePoseTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.LeadInOutPoseTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePoseTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.VelocityConstrainedPoseTrajectoryGenerator;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameMatrix3D;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.math.trajectories.ConstantPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsOneDoFJointTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.OneDoFJointQuinticTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.OneDoFJointTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.OneDoFJointWayPointTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.WrapperForPositionAndOrientationTrajectoryGenerators;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateChangedListener;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionAction;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.robotics.trajectories.providers.CurrentPositionProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.io.printing.PrintTools;

public class HandControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // JPratt. February 27, 2015: Added this since new Atlas was having trouble with network stuff.
   // It was sending 14,000 variables. This and others reduces it a bit when set to false.
   private static final boolean REGISTER_YOVARIABLES = true;

   private static final boolean USE_VELOCITYCONSTRAINED_INSTEADOF_STRAIGHTLINE = false;

   private final YoVariableRegistry registry;

   private final StateMachine<HandControlState> stateMachine;
   private final RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule;
   private final TaskspaceToJointspaceCalculator handTaskspaceToJointspaceCalculator;
   private final FrameMatrix3D selectionFrameMatrix = new FrameMatrix3D();
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);

   private final Map<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator> quinticPolynomialTrajectoryGenerators;
   private final Map<OneDoFJoint, OneDoFJointWayPointTrajectoryGenerator> waypointsPolynomialTrajectoryGenerators;
   private final Map<OneDoFJoint, MultipleWaypointsOneDoFJointTrajectoryGenerator> wholeBodyWaypointsPolynomialTrajectoryGenerators;

   private final EnumYoVariable<HandTrajectoryType> activeTrajectory;

   private final ConstantPoseTrajectoryGenerator holdPoseTrajectoryGenerator;
   private final StraightLinePoseTrajectoryGenerator straightLinePoseTrajectoryGenerator;
   private final CirclePoseTrajectoryGenerator circularPoseTrajectoryGenerator;
   private final VelocityConstrainedPoseTrajectoryGenerator velocityConstrainedPoseTrajectoryGenerator;
   private final FinalApproachPoseTrajectoryGenerator finalApproachPoseTrajectoryGenerator;
   private final InitialClearancePoseTrajectoryGenerator initialClearancePoseTrajectoryGenerator;
   private final LeadInOutPoseTrajectoryGenerator leadInOutPoseTrajectoryGenerator;

   private final PositionProvider currentHandPosition;
   private final YoVariableDoubleProvider trajectoryTimeProvider;
   private final MultipleWaypointsPositionTrajectoryGenerator waypointPositionTrajectoryGenerator;
   private final MultipleWaypointsOrientationTrajectoryGenerator waypointOrientationTrajectoryGenerator;
   private final WrapperForPositionAndOrientationTrajectoryGenerators wayPointPositionAndOrientationTrajectoryGenerator;

   private final Map<OneDoFJoint, DoubleYoVariable> qDesireds = new LinkedHashMap<>();
   private final Map<OneDoFJoint, DoubleYoVariable> qdDesireds = new LinkedHashMap<>();

   private final BooleanYoVariable isExecutingHandStep;

   private final Map<OneDoFJoint, Double> jointCurrentPositionMap;

   private final TrajectoryBasedTaskspaceHandControlState taskSpacePositionControlState;
   private final JointSpaceHandControlState jointSpaceHandControlState;
   private final LoadBearingHandControlState loadBearingControlState;
   private final TaskspaceToJointspaceHandForcefeedbackControlState taskspaceToJointspaceHandForcefeedbackControlState;

   private final EnumYoVariable<HandControlState> requestedState;
   private final OneDoFJoint[] oneDoFJoints;
   private final Map<OneDoFJoint, BooleanYoVariable> areJointsEnabled;
   private final String name;
   private final RobotSide robotSide;
   private final TwistCalculator twistCalculator;
   private final RigidBody chest, hand;

   private final FullHumanoidRobotModel fullRobotModel;

   private final double controlDT;

   private final YoSE3PIDGains taskspaceGains, taskspaceLoadBearingGains;

   private final StateChangedListener<HandControlState> stateChangedlistener;
   private final HandPoseStatusProducer handPoseStatusProducer;
   private final BooleanYoVariable hasHandPoseStatusBeenSent;

   private final BooleanYoVariable areAllArmJointEnabled;

   private final boolean doPositionControl;

   private final ReferenceFrame handControlFrame;
   private final PoseReferenceFrame optionalHandControlFrame;

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

      activeTrajectory = new EnumYoVariable<>(namePrefix + "ActiveTrajectory", registry, HandTrajectoryType.class);

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

      handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      optionalHandControlFrame = new PoseReferenceFrame("optionalHandControlFrame", handControlFrame);

      this.robotSide = robotSide;
      twistCalculator = momentumBasedController.getTwistCalculator();

      oneDoFJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(chest, hand), OneDoFJoint.class);

      for (OneDoFJoint joint : oneDoFJoints)
      {
         qDesireds.put(joint, new DoubleYoVariable("q_d_" + joint.getName() + "HandControlModule", registry));
         qdDesireds.put(joint, new DoubleYoVariable("qd_d_" + joint.getName() + "HandControlModule", registry));
      }

      requestedState = new EnumYoVariable<HandControlState>(name + "RequestedState", "", registry, HandControlState.class, true);
      requestedState.set(null);

      trajectoryTimeProvider = new YoVariableDoubleProvider(name + "TrajectoryTime", registry);
      currentHandPosition = new CurrentPositionProvider(handFrame);

      quinticPolynomialTrajectoryGenerators = new LinkedHashMap<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator>();
      areJointsEnabled = new LinkedHashMap<>();

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         OneDoFJointQuinticTrajectoryGenerator trajectoryGenerator = new OneDoFJointQuinticTrajectoryGenerator(oneDoFJoint.getName() + "Trajectory",
                                                                        oneDoFJoint, trajectoryTimeProvider, registry);
         quinticPolynomialTrajectoryGenerators.put(oneDoFJoint, trajectoryGenerator);
         areJointsEnabled.put(oneDoFJoint, new BooleanYoVariable(namePrefix + oneDoFJoint.getName() + "IsEnabled", registry));
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

      DoubleYoVariable yoTime = momentumBasedController.getYoTime();
      stateMachine = new StateMachine<HandControlState>(name, name + "SwitchTime", HandControlState.class, yoTime, registry);

      handSpatialAccelerationControlModule = new RigidBodySpatialAccelerationControlModule(name, twistCalculator, hand,
              handControlFrame, controlDT, registry);
      handSpatialAccelerationControlModule.setGains(taskspaceGains);

      handTaskspaceToJointspaceCalculator = new TaskspaceToJointspaceCalculator(namePrefix + "Hand", chest, hand, controlDT, registry);
      handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(handControlFrame);
      handTaskspaceToJointspaceCalculator.setupWithDefaultParameters();

      holdPoseTrajectoryGenerator = new ConstantPoseTrajectoryGenerator(name + "Hold", true, worldFrame, parentRegistry);
      
      if (USE_VELOCITYCONSTRAINED_INSTEADOF_STRAIGHTLINE)
      {
         straightLinePoseTrajectoryGenerator = null;
         velocityConstrainedPoseTrajectoryGenerator = new VelocityConstrainedPoseTrajectoryGenerator(name + "VelocityConstrained", true, worldFrame, registry,
               visualize, yoGraphicsListRegistry);
      }
      else
      {
         straightLinePoseTrajectoryGenerator = new StraightLinePoseTrajectoryGenerator(name + "StraightLine", true, worldFrame, registry, visualize,
               yoGraphicsListRegistry);
         velocityConstrainedPoseTrajectoryGenerator = null;
      }
      
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


      doPositionControl = armControlParameters.doLowLevelPositionControl();
      String stateNamePrefix = namePrefix + "Hand";
      jointSpaceHandControlState = new JointSpaceHandControlState(stateNamePrefix, oneDoFJoints, doPositionControl, momentumBasedController, jointspaceGains, controlDT, registry);
      if (momentumBasedController.getWristForceSensors() != null)
      {
         taskspaceToJointspaceHandForcefeedbackControlState = new TaskspaceToJointspaceHandForcefeedbackControlState(stateNamePrefix,
               HandControlState.TASK_SPACE_FORCE_FEEDBACK_CONTROL, robotSide, momentumBasedController, jacobianId, chest, hand, doPositionControl, jointspaceGains, registry);
      }
      else
      {
         taskspaceToJointspaceHandForcefeedbackControlState = null;
      }

      if (doPositionControl)
      {
         // TODO Not implemented for position control.
         loadBearingControlState = null;
         taskSpacePositionControlState = TaskspaceToJointspaceHandPositionControlState.createControlStateForPositionControlledJoints(stateNamePrefix, robotSide, momentumBasedController, chest, hand, registry);
      }
      else
      {
         loadBearingControlState = new LoadBearingHandControlState(stateNamePrefix, HandControlState.LOAD_BEARING, robotSide, momentumBasedController, fullRobotModel.getElevator(), hand, jacobianId, registry);

         if (armControlParameters.useInverseKinematicsTaskspaceControl())
         {
            taskSpacePositionControlState = TaskspaceToJointspaceHandPositionControlState.createControlStateForForceControlledJoints(stateNamePrefix, robotSide, momentumBasedController, chest, hand, jointspaceGains, registry);
         }
         else
         {
            taskSpacePositionControlState = new TaskspaceHandPositionControlState(stateNamePrefix, HandControlState.TASK_SPACE_POSITION, momentumBasedController, jacobianId, chest, hand, yoGraphicsListRegistry, registry);
         }
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

      addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, taskSpacePositionControlState);
      addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, jointSpaceHandControlState);
      
      
      stateMachine.addState(jointSpaceHandControlState);
      stateMachine.addState(taskSpacePositionControlState);

      if (taskspaceToJointspaceHandForcefeedbackControlState != null)
      {
         addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, taskspaceToJointspaceHandForcefeedbackControlState);
         addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, taskspaceToJointspaceHandForcefeedbackControlState);
         addRequestedStateTransition(requestedState, false, taskspaceToJointspaceHandForcefeedbackControlState, taskSpacePositionControlState);
         addRequestedStateTransition(requestedState, false, taskspaceToJointspaceHandForcefeedbackControlState, jointSpaceHandControlState);
         addRequestedStateTransition(requestedState, false, taskspaceToJointspaceHandForcefeedbackControlState, taskspaceToJointspaceHandForcefeedbackControlState);
         stateMachine.addState(taskspaceToJointspaceHandForcefeedbackControlState);
      }

      if (!doPositionControl)
      {
         addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, loadBearingControlState);
         addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, loadBearingControlState);
         addRequestedStateTransition(requestedState, false, loadBearingControlState, taskSpacePositionControlState);
         addRequestedStateTransition(requestedState, false, loadBearingControlState, jointSpaceHandControlState);
         if (taskspaceToJointspaceHandForcefeedbackControlState != null)
         {
            addRequestedStateTransition(requestedState, false, loadBearingControlState, taskspaceToJointspaceHandForcefeedbackControlState);
            addRequestedStateTransition(requestedState, false, taskspaceToJointspaceHandForcefeedbackControlState, loadBearingControlState);
         }

         setupTransitionToSupport(taskSpacePositionControlState);
         stateMachine.addState(loadBearingControlState);
      }

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

   public void initialize()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         qDesireds.get(joint).set(joint.getQ());
         qdDesireds.get(joint).set(joint.getQd());
      }
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

      for (OneDoFJoint joint : oneDoFJoints)
      {
         qDesireds.get(joint).set(joint.getqDesired());
         qdDesireds.get(joint).set(joint.getQdDesired());
      }
   }

   private boolean checkIfAtLeastOneJointIsDisabled()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         areJointsEnabled.get(oneDoFJoints[i]).set(oneDoFJoints[i].isEnabled());
         if (!oneDoFJoints[i].isEnabled())
            return true;
      }
      return false;
   }

   private void checkAndSendHandPoseStatusIsCompleted()
   {
      boolean isExecutingHandPose = stateMachine.getCurrentStateEnum() == HandControlState.TASK_SPACE_POSITION;
      isExecutingHandPose |= stateMachine.getCurrentStateEnum() == HandControlState.JOINT_SPACE;
      isExecutingHandPose |= stateMachine.getCurrentStateEnum() == HandControlState.TASK_SPACE_FORCE_FEEDBACK_CONTROL;

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

      if (stateMachine.getCurrentState() instanceof JointSpaceHandControlState)
      {
         handTaskspaceToJointspaceCalculator.initializeFromDesiredJointAngles();
      }
      else if (stateMachine.getCurrentStateEnum() == HandControlState.LOAD_BEARING)
      {
         handTaskspaceToJointspaceCalculator.initializeFromCurrentJointAngles();
      }

      if (handPoseStatusProducer != null)
         handPoseStatusProducer.sendStartedStatus(robotSide);

      handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(handControlFrame);
   }
   
   public void executeForceControlledTaskSpaceTrajectory(PoseTrajectoryGenerator poseTrajectory, DenseMatrix64F selectionMatrix, Point3d rotationAxisOriginInWorld, Vector3d rotationAxisInWorld)
   {
      if (taskspaceToJointspaceHandForcefeedbackControlState == null)
         return;

	   handSpatialAccelerationControlModule.setGains(taskspaceGains);
	   taskspaceToJointspaceHandForcefeedbackControlState.initialize(rotationAxisOriginInWorld, rotationAxisInWorld);
	   taskspaceToJointspaceHandForcefeedbackControlState.setTrajectoryWithAngularControlQuality(poseTrajectory, Double.NaN, Double.NaN);
	   taskspaceToJointspaceHandForcefeedbackControlState.setControlModuleForPositionControl(handTaskspaceToJointspaceCalculator);
	   taskspaceToJointspaceHandForcefeedbackControlState.setSelectionMatrix(selectionMatrix);
	   requestedState.set(taskspaceToJointspaceHandForcefeedbackControlState.getStateEnum());
	   stateMachine.checkTransitionConditions();
	   isExecutingHandStep.set(false);

	   handTaskspaceToJointspaceCalculator.initializeFromDesiredJointAngles();

	   if (handPoseStatusProducer != null)
	   {
	      handPoseStatusProducer.sendStartedStatus(robotSide);
	   }
	   handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(handControlFrame);
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

   private final FramePoint rotationAxisOrigin = new FramePoint();
   private final FrameVector rotationAxis = new FrameVector();
   
   public void moveInCircle(Point3d rotationAxisOriginInWorld, Vector3d rotationAxisInWorld, double rotationAngle, boolean controlHandAngleAboutAxis, double graspOffsetFromControlFrame, double time)
   {
      optionalHandControlFrame.setX(graspOffsetFromControlFrame);
      optionalHandControlFrame.update();

      trajectoryTimeProvider.set(time);

      rotationAxisOrigin.setIncludingFrame(worldFrame, rotationAxisOriginInWorld);
      rotationAxis.setIncludingFrame(worldFrame, rotationAxisInWorld);

      circularPoseTrajectoryGenerator.setControlledFrame(optionalHandControlFrame);

      circularPoseTrajectoryGenerator.setDesiredRotationAngle(rotationAngle);
      circularPoseTrajectoryGenerator.updateCircleFrame(rotationAxisOrigin, rotationAxis);
      FramePose desiredFramePose = computeDesiredFramePose(worldFrame, optionalHandControlFrame, HandTrajectoryType.CIRCULAR);
      circularPoseTrajectoryGenerator.setInitialPose(desiredFramePose);
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
      handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(optionalHandControlFrame);
   }
   
   public void moveInCircleForceControl(Point3d rotationAxisOriginInWorld, Vector3d rotationAxisInWorld, double rotationAngle, boolean controlHandAngleAboutAxis, double graspOffsetFromControlFrame, double time,
		   Vector3d forceConstraintVector, double desiredTangentialForce)
   {
      if (taskspaceToJointspaceHandForcefeedbackControlState == null)
      {
         PrintTools.error(this, "The hand force control controller was not setup. Ignoring the command.");
         return;
      }

	   optionalHandControlFrame.setX(graspOffsetFromControlFrame);
	   optionalHandControlFrame.update();

	   trajectoryTimeProvider.set(time);

	   rotationAxisOrigin.setIncludingFrame(worldFrame, rotationAxisOriginInWorld);
	   rotationAxis.setIncludingFrame(worldFrame, rotationAxisInWorld);

	   circularPoseTrajectoryGenerator.setControlledFrame(optionalHandControlFrame);

	   circularPoseTrajectoryGenerator.setDesiredRotationAngle(rotationAngle);
	   circularPoseTrajectoryGenerator.updateCircleFrame(rotationAxisOrigin, rotationAxis);
	   FramePose desiredFramePose = computeDesiredFramePose(worldFrame, optionalHandControlFrame, HandTrajectoryType.CIRCULAR);
	   circularPoseTrajectoryGenerator.setInitialPose(desiredFramePose);

	   executeForceControlledTaskSpaceTrajectory(circularPoseTrajectoryGenerator, selectionMatrix, rotationAxisOriginInWorld, rotationAxisInWorld);
	   handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(optionalHandControlFrame);
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

      if (!isLoadBearing())
      {
         FramePose pose = computeDesiredFramePose(trajectoryFrame, HandTrajectoryType.STRAIGHT_LINE);
         PoseTrajectoryGenerator straightLineTrajectoryToUse = setupStraightLinePoseTrajectory(finalDesiredPose, time, trajectoryFrame, pose);
         executeTaskSpaceTrajectory(straightLineTrajectoryToUse, selectionMatrix, percentOfTrajectoryWithOrientationBeingControlled, time);
      }
      else
      {
         loadBearingControlState.getContactNormalVector(initialDirection);
         moveAwayObject(finalDesiredPose, initialDirection, swingClearance, time, trajectoryFrame);
      }
   }

   private PoseTrajectoryGenerator setupStraightLinePoseTrajectory(FramePose finalDesiredPose, double time, ReferenceFrame trajectoryFrame, FramePose pose)
   {
      if (USE_VELOCITYCONSTRAINED_INSTEADOF_STRAIGHTLINE)
      {

         FrameVector initialHandPoseVelocity = handTaskspaceToJointspaceCalculator.getInitialHandPoseVelocity(trajectoryFrame);
         FrameVector initialHandPoseAngularVelocity = handTaskspaceToJointspaceCalculator.getInitialHandPoseAngularVelocity(trajectoryFrame);

         velocityConstrainedPoseTrajectoryGenerator.registerAndSwitchFrame(trajectoryFrame);
         velocityConstrainedPoseTrajectoryGenerator.setInitialPoseWithInitialVelocity(pose, initialHandPoseVelocity, initialHandPoseAngularVelocity);
         velocityConstrainedPoseTrajectoryGenerator.setFinalPoseWithoutFinalVelocity(finalDesiredPose);
         velocityConstrainedPoseTrajectoryGenerator.setTrajectoryTime(time);

         return velocityConstrainedPoseTrajectoryGenerator;
      }
      else
      {
         straightLinePoseTrajectoryGenerator.registerAndSwitchFrame(trajectoryFrame);
         straightLinePoseTrajectoryGenerator.setInitialPose(pose);
         straightLinePoseTrajectoryGenerator.setFinalPose(finalDesiredPose);
         straightLinePoseTrajectoryGenerator.setTrajectoryTime(time);

         return straightLinePoseTrajectoryGenerator;
      }
   }

   private final FrameVector initialDirection = new FrameVector();
   private final FrameVector finalDirection = new FrameVector();

   public void moveTowardsObjectAndGoToSupport(FramePose finalDesiredPose, FrameVector surfaceNormal, double clearance, double time,
           ReferenceFrame trajectoryFrame, boolean goToSupportWhenDone, double holdPositionDuration)
   {
      if (doPositionControl)
      {
         PrintTools.error("Cannot do load bearing when the arms are position controlled.");
         return;
      }

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
      if (!isLoadBearing())
      {
         FramePose pose = computeDesiredFramePose(trajectoryFrame, HandTrajectoryType.STRAIGHT_LINE);
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
      FramePose pose = computeDesiredFramePose(trajectoryFrame, HandTrajectoryType.STRAIGHT_LINE);
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
      FramePose pose = computeDesiredFramePose(trajectoryFrame, HandTrajectoryType.STRAIGHT_LINE);
      leadInOutPoseTrajectoryGenerator.registerAndSwitchFrame(trajectoryFrame);
      leadInOutPoseTrajectoryGenerator.setInitialLeadOut(pose, initialDirection, clearance);
      leadInOutPoseTrajectoryGenerator.setFinalLeadIn(finalDesiredPose, finalDirection, clearance);
      leadInOutPoseTrajectoryGenerator.setTrajectoryTime(time);
      executeTaskSpaceTrajectory(leadInOutPoseTrajectoryGenerator);
   }

   private final FramePose currentDesiredPose = new FramePose();

   private FramePose computeDesiredFramePose(ReferenceFrame trajectoryFrame, HandTrajectoryType newTrajectoryType)
   {
      return computeDesiredFramePose(trajectoryFrame, handControlFrame, newTrajectoryType);
   }

   private final RigidBodyTransform oldTrackingFrameTransform = new RigidBodyTransform();
   private final RigidBodyTransform newTrackingFrameTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformFromNewTrackingFrameToOldTrackingFrame = new RigidBodyTransform();

   private FramePose computeDesiredFramePose(ReferenceFrame trajectoryFrame, ReferenceFrame newControlFrame, HandTrajectoryType newTrajectoryType)
   {
      FramePose pose;

      if (stateMachine.getCurrentState() instanceof TaskspaceHandPositionControlState)
      {
         pose = taskSpacePositionControlState.getDesiredPose();
         ReferenceFrame oldControlFrame = handSpatialAccelerationControlModule.getTrackingFrame();
         if (newControlFrame != oldControlFrame)
         {
            pose.getPose(oldTrackingFrameTransform);
            newControlFrame.getTransformToDesiredFrame(transformFromNewTrackingFrameToOldTrackingFrame, oldControlFrame);
            newTrackingFrameTransform.multiply(oldTrackingFrameTransform, transformFromNewTrackingFrameToOldTrackingFrame);
            pose.setPose(newTrackingFrameTransform);
         }
      }
      else if (newTrajectoryType == HandTrajectoryType.STRAIGHT_LINE || stateMachine.getCurrentState() instanceof JointSpaceHandControlState)
      {
         handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(newControlFrame);
         handTaskspaceToJointspaceCalculator.initializeFromDesiredJointAngles();
         handTaskspaceToJointspaceCalculator.getDesiredEndEffectorPoseFromQDesireds(currentDesiredPose, trajectoryFrame);
         pose = currentDesiredPose;

         if (taskSpacePositionControlState instanceof TaskspaceToJointspaceHandPositionControlState)
            ((TaskspaceToJointspaceHandPositionControlState) taskSpacePositionControlState).resetCompliantControl();
      }
      else if (newTrajectoryType == HandTrajectoryType.CIRCULAR || stateMachine.getCurrentState() instanceof TaskspaceToJointspaceHandPositionControlState)
      {
         pose = taskSpacePositionControlState.getDesiredPose();

         ReferenceFrame oldControlFrame = handTaskspaceToJointspaceCalculator.getControlFrame();
         if (oldControlFrame != newControlFrame)
         {
            handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(newControlFrame);
            handTaskspaceToJointspaceCalculator.getDesiredEndEffectorPoseFromQDesireds(currentDesiredPose, trajectoryFrame);
            pose = currentDesiredPose;

            if (taskSpacePositionControlState instanceof TaskspaceToJointspaceHandPositionControlState)
               ((TaskspaceToJointspaceHandPositionControlState) taskSpacePositionControlState).resetCompliantControl();
         }
      }
      else
      {
         currentDesiredPose.setToZero(newControlFrame);
         pose = currentDesiredPose;
      }

      pose.changeFrame(trajectoryFrame);

      activeTrajectory.set(newTrajectoryType);

      return pose;
   }

   public void requestLoadBearing()
   {
      if (doPositionControl)
      {
         PrintTools.error("Cannot do load bearing when the arms are position controlled.");
         return;
      }
      requestedState.set(loadBearingControlState.getStateEnum());
      loadBearingControlState.setControlModuleForForceControl(handSpatialAccelerationControlModule);
   }

   public void moveUsingQuinticSplines(Map<OneDoFJoint, Double> desiredJointPositions, double time)
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         if (!desiredJointPositions.containsKey(oneDoFJoints[i]))
            throw new RuntimeException("not all joint positions specified");
      }

      trajectoryTimeProvider.set(time);

      for (OneDoFJoint oneDoFJoint : desiredJointPositions.keySet())
      {
         double desiredPositions = desiredJointPositions.get(oneDoFJoint);

         // make sure that we do not set the desired position outside the joint limit
         desiredPositions = MathTools.clipToMinMax(desiredPositions, oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper());
         quinticPolynomialTrajectoryGenerators.get(oneDoFJoint).setFinalPosition(desiredPositions);
      }

      executeJointspaceTrajectory(quinticPolynomialTrajectoryGenerators);
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
      }

      executeJointspaceTrajectory(quinticPolynomialTrajectoryGenerators, true);
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

            if (!MathTools.isInsideBoundsInclusive(position, oneDoFJoints[jointIdx].getJointLimitLower(), oneDoFJoints[jointIdx].getJointLimitUpper()))
            {
               PrintTools.warn(this, "Waypoint " + i + " is out of the joint position limits, cancelling action.");
               trajectoryGenerator.clear();
               return;
            }

            boolean success = trajectoryGenerator.appendWaypoint(time, position, velocity);
            if (!success)
               return;
         }
      }

      executeJointspaceTrajectory(wholeBodyWaypointsPolynomialTrajectoryGenerators);
   }

   @Deprecated
   public void moveJointspaceWithWaypoints(Map<OneDoFJoint, double[]> desiredJointPositions, double time)
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         if (!desiredJointPositions.containsKey(oneDoFJoints[i]))
            throw new RuntimeException("not all joint positions specified");
      }

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

      executeJointspaceTrajectory(waypointsPolynomialTrajectoryGenerators);
   }

   private void executeJointspaceTrajectory(Map<OneDoFJoint, ? extends OneDoFJointTrajectoryGenerator> trajectoryGenrators)
   {
      executeJointspaceTrajectory(trajectoryGenrators, false);
   }

   private void executeJointspaceTrajectory(Map<OneDoFJoint, ? extends OneDoFJointTrajectoryGenerator> trajectoryGenrators, boolean initializeToCurrent)
   {
      if (!initializeToCurrent && stateMachine.getCurrentState() == jointSpaceHandControlState)
      {
         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            OneDoFJoint joint = oneDoFJoints[i];
            double initialPosition = qDesireds.get(joint).getDoubleValue();
            double initialVelocity = qdDesireds.get(joint).getDoubleValue();
            trajectoryGenrators.get(joint).initialize(initialPosition, initialVelocity);
         }
      }
      else
      {
         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            OneDoFJoint joint = oneDoFJoints[i];
            trajectoryGenrators.get(joint).initialize();
         }
      }

      jointSpaceHandControlState.setTrajectories(trajectoryGenrators);
      requestedState.set(jointSpaceHandControlState.getStateEnum());
      stateMachine.checkTransitionConditions();

      if (handPoseStatusProducer != null)
         handPoseStatusProducer.sendStartedStatus(robotSide);
   }

   public boolean isLoadBearing()
   {
      if (doPositionControl)
         return false;
      return stateMachine.getCurrentStateEnum() == HandControlState.LOAD_BEARING;
   }

   public void holdPositionInBase()
   {
      FramePose currentDesiredHandPose = computeDesiredFramePose(chest.getBodyFixedFrame(), HandTrajectoryType.STRAIGHT_LINE);

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

      moveUsingQuinticSplines(jointCurrentPositionMap, 0.0);
   }

   public void resetJointIntegrators()
   {
      for (OneDoFJoint joint : oneDoFJoints)
         joint.resetIntegrator();
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

   public void setEnableCompliantControl(boolean enable, boolean[] enableLinearCompliance, boolean[] enableAngularCompliance, Vector3d desiredForce, Vector3d desiredTorque, double forceDeadzone, double torqueDeadzone)
   {
      if (taskSpacePositionControlState instanceof TaskspaceToJointspaceHandPositionControlState)
      {
         TaskspaceToJointspaceHandPositionControlState taskspaceToJointspaceHandPositionControlState = (TaskspaceToJointspaceHandPositionControlState) taskSpacePositionControlState;
         taskspaceToJointspaceHandPositionControlState.setEnableCompliantControl(true, enableLinearCompliance, enableAngularCompliance, desiredForce, desiredTorque, forceDeadzone, torqueDeadzone);
      }
   }
}
