package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.*;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.sensors.MassMatrixEstimatingToolRigidBody;
import us.ihmc.commonWalkingControlModules.trajectories.*;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import java.util.LinkedHashMap;
import java.util.Map;

import static com.yobotics.simulationconstructionset.util.statemachines.StateMachineTools.addRequestedStateTransition;

public class IndividualHandControlModule
{
   private final YoVariableRegistry registry;

   private final StateMachine<IndividualHandControlState> stateMachine;
   private final Map<ReferenceFrame, RigidBodySpatialAccelerationControlModule> handSpatialAccelerationControlModules;
   private final MassMatrixEstimatingToolRigidBody toolBody;

   private final ChangeableConfigurationProvider currentConfigurationProvider;
   private final ChangeableConfigurationProvider desiredConfigurationProvider;

   private final Map<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator> quinticPolynomialTrajectoryGenerators;

   private final Map<ReferenceFrame, StraightLinePositionTrajectoryGenerator> straightLinePositionWorldTrajectoryGenerators;
   private final Map<ReferenceFrame, OrientationInterpolationTrajectoryGenerator> orientationInterpolationWorldTrajectoryGenerators;
   private final YoVariableDoubleProvider trajectoryTimeProvider;

   private final ConstantPositionTrajectoryGenerator holdPositionInBaseTrajectoryGenerator;
   private final ConstantOrientationTrajectoryGenerator holdOrientationInBaseTrajectoryGenerator;

   private final TaskspaceHandPositionControlState taskspaceHandPositionControlState;
   private final JointSpaceHandControlControlState jointSpaceHandControlState;
   private final LoadBearingCylindricalHandControlState loadBearingCylindricalState;
   private final ObjectManipulationState objectManipulationState;
   private final LoadBearingPlaneHandControlState loadBearingPlaneState;

   private final EnumYoVariable<IndividualHandControlState> requestedState;
   private final HandControllerInterface handController;
   private final OneDoFJoint[] oneDoFJoints;
   private final GeometricJacobian jacobian;
   private final String name;
   private final TwistCalculator twistCalculator;

   public IndividualHandControlModule(final DoubleYoVariable simulationTime, final RobotSide robotSide, final FullRobotModel fullRobotModel,
                                      final TwistCalculator twistCalculator, final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                      HandControllerInterface handController, double gravity, final double controlDT,
                                      MomentumBasedController momentumBasedController, GeometricJacobian jacobian, final YoVariableRegistry parentRegistry)
   {
      RigidBody endEffector = jacobian.getEndEffector();

      name = endEffector.getName() + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);
      this.twistCalculator = twistCalculator;

      this.handController = handController;

      oneDoFJoints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);

      requestedState = new EnumYoVariable<IndividualHandControlState>(name + "RequestedState", "", registry, IndividualHandControlState.class, true);
      requestedState.set(null);

      trajectoryTimeProvider = new YoVariableDoubleProvider(name + "TrajectoryTime", registry);

      quinticPolynomialTrajectoryGenerators = new LinkedHashMap<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator>();

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         OneDoFJointQuinticTrajectoryGenerator trajectoryGenerator = new OneDoFJointQuinticTrajectoryGenerator(oneDoFJoint.getName() + "Trajectory",
                                                                        oneDoFJoint, trajectoryTimeProvider, registry);
         quinticPolynomialTrajectoryGenerators.put(oneDoFJoint, trajectoryGenerator);
      }

      this.toolBody = new MassMatrixEstimatingToolRigidBody(name + "Tool", handController.getWristJoint(), fullRobotModel, gravity, controlDT, registry,
              dynamicGraphicObjectsListRegistry);

      this.jacobian = jacobian;

      stateMachine = new StateMachine<IndividualHandControlState>(name, name + "SwitchTime", IndividualHandControlState.class, simulationTime, registry);

      handSpatialAccelerationControlModules = new LinkedHashMap<ReferenceFrame, RigidBodySpatialAccelerationControlModule>();

      ReferenceFrame endEffectorFrame = jacobian.getEndEffectorFrame();
      currentConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(endEffectorFrame));
      desiredConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(endEffectorFrame));    // FIXME: make Yo, but is difficult because frame can change

      straightLinePositionWorldTrajectoryGenerators = new LinkedHashMap<ReferenceFrame, StraightLinePositionTrajectoryGenerator>();
      orientationInterpolationWorldTrajectoryGenerators = new LinkedHashMap<ReferenceFrame, OrientationInterpolationTrajectoryGenerator>();

      holdPositionInBaseTrajectoryGenerator = new ConstantPositionTrajectoryGenerator(name + "HoldPosition", jacobian.getBaseFrame(),
              currentConfigurationProvider, 0.0, registry);
      holdOrientationInBaseTrajectoryGenerator = new ConstantOrientationTrajectoryGenerator(name + "HoldOrientation", jacobian.getBaseFrame(),
              currentConfigurationProvider, 0.0, registry);

      loadBearingCylindricalState = new LoadBearingCylindricalHandControlState(IndividualHandControlState.LOAD_BEARING_CYLINDRICAL, momentumBasedController,
              jacobian, parentRegistry, robotSide);

      loadBearingPlaneState = new LoadBearingPlaneHandControlState(IndividualHandControlState.LOAD_BEARING_PLANE, robotSide, momentumBasedController, jacobian,
              handController, registry);

      jointSpaceHandControlState = new JointSpaceHandControlControlState(IndividualHandControlState.JOINT_SPACE, robotSide, jacobian, momentumBasedController,
              registry, 1.0);

      objectManipulationState = new ObjectManipulationState(IndividualHandControlState.OBJECT_MANIPULATION, robotSide, momentumBasedController, jacobian,
              handController, toolBody, dynamicGraphicObjectsListRegistry, parentRegistry);

      taskspaceHandPositionControlState = new TaskspaceHandPositionControlState(IndividualHandControlState.TASK_SPACE_POSITION, robotSide,
              momentumBasedController, jacobian, dynamicGraphicObjectsListRegistry, registry);

      addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, taskspaceHandPositionControlState);
      addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, objectManipulationState);
      addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, jointSpaceHandControlState);

      addRequestedStateTransition(requestedState, false, taskspaceHandPositionControlState, objectManipulationState);
      addRequestedStateTransition(requestedState, false, taskspaceHandPositionControlState, jointSpaceHandControlState);
      addRequestedStateTransition(requestedState, false, taskspaceHandPositionControlState, taskspaceHandPositionControlState);

      addRequestedStateTransition(requestedState, false, objectManipulationState, jointSpaceHandControlState);
      addRequestedStateTransition(requestedState, false, objectManipulationState, taskspaceHandPositionControlState);
      addRequestedStateTransition(requestedState, false, objectManipulationState, objectManipulationState);

      addTransitionToCylindricalLoadBearing(requestedState, handController, jointSpaceHandControlState, loadBearingCylindricalState);
      addTransitionToCylindricalLoadBearing(requestedState, handController, taskspaceHandPositionControlState, loadBearingCylindricalState);

      addTransitionToPlaneLoadBearing(requestedState, handController, jointSpaceHandControlState, loadBearingPlaneState);
      addTransitionToPlaneLoadBearing(requestedState, handController, taskspaceHandPositionControlState, loadBearingPlaneState);

      stateMachine.addState(jointSpaceHandControlState);
      stateMachine.addState(taskspaceHandPositionControlState);
      stateMachine.addState(objectManipulationState);
      stateMachine.addState(loadBearingCylindricalState);
      stateMachine.addState(loadBearingPlaneState);

      parentRegistry.addChild(registry);
   }

   private static void addTransitionToCylindricalLoadBearing(final EnumYoVariable<IndividualHandControlState> requestedState,
           final HandControllerInterface handControllerInterface, State<IndividualHandControlState> fromState, final State<IndividualHandControlState> toState)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            boolean transitionRequested = requestedState.getEnumValue() == toState.getStateEnum();
            boolean ableToBearLoad = handControllerInterface.isAbleToBearLoad();

            return transitionRequested && ableToBearLoad;
         }
      };
      StateTransitionAction stateTransitionAction = new StateTransitionAction()
      {
         public void doTransitionAction()
         {
         }
      };
      StateTransition<IndividualHandControlState> stateTransition = new StateTransition<IndividualHandControlState>(toState.getStateEnum(),
                                                                       stateTransitionCondition, stateTransitionAction);
      fromState.addStateTransition(stateTransition);
   }

   private static void addTransitionToPlaneLoadBearing(final EnumYoVariable<IndividualHandControlState> requestedState,
           final HandControllerInterface handControllerInterface, State<IndividualHandControlState> fromState, final State<IndividualHandControlState> toState)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            boolean transitionRequested = requestedState.getEnumValue() == toState.getStateEnum();
            boolean ableToBearLoad = handControllerInterface.isOpen();

            return transitionRequested && ableToBearLoad;
         }
      };
      StateTransitionAction stateTransitionAction = new StateTransitionAction()
      {
         public void doTransitionAction()
         {
         }
      };
      StateTransition<IndividualHandControlState> stateTransition = new StateTransition<IndividualHandControlState>(toState.getStateEnum(),
                                                                       stateTransitionCondition, stateTransitionAction);
      fromState.addStateTransition(stateTransition);
   }

   public void doControl()
   {
      stateMachine.checkTransitionConditionsThoroughly();
      stateMachine.doAction();
   }

   public boolean isDone()
   {
      return stateMachine.getCurrentState().isDone();
   }

   public void executeTaskSpaceTrajectory(PositionTrajectoryGenerator positionTrajectory, OrientationTrajectoryGenerator orientationTrajectory,
           ReferenceFrame frameToControlPoseOf, RigidBody base, boolean estimateMassProperties)
   {
      TaskspaceHandPositionControlState state = estimateMassProperties ? objectManipulationState : taskspaceHandPositionControlState;
      RigidBodySpatialAccelerationControlModule rigidBodySpatialAccelerationControlModule =
         getOrCreateRigidBodySpatialAccelerationControlModule(frameToControlPoseOf);
      state.setTrajectory(positionTrajectory, orientationTrajectory, base, rigidBodySpatialAccelerationControlModule);
      requestedState.set(state.getStateEnum());
   }

   public void moveInStraightLine(FramePose finalDesiredPose, double time, RigidBody base, ReferenceFrame frameToControlPoseOf, ReferenceFrame trajectoryFrame,
                                  boolean holdObject)
   {
      currentConfigurationProvider.set(new FramePose(frameToControlPoseOf));
      desiredConfigurationProvider.set(finalDesiredPose);
      trajectoryTimeProvider.set(time);
      executeTaskSpaceTrajectory(getOrCreateStraightLinePositionTrajectoryGenerator(trajectoryFrame),
                                 getOrCreateOrientationInterpolationTrajectoryGenerator(trajectoryFrame), frameToControlPoseOf, base, holdObject);
   }

   public void requestLoadBearing()
   {
      if (handController.isClosed() || handController.isClosing())
         requestedState.set(loadBearingCylindricalState.getStateEnum());
      else
         requestedState.set(loadBearingPlaneState.getStateEnum());
   }

   public void executeJointSpaceTrajectory(Map<OneDoFJoint, ? extends DoubleTrajectoryGenerator> trajectories)
   {
      jointSpaceHandControlState.setTrajectories(trajectories);
   }

   public void moveUsingQuinticSplines(Map<OneDoFJoint, Double> desiredJointPositions, double time)
   {
      if (!desiredJointPositions.keySet().containsAll(quinticPolynomialTrajectoryGenerators.keySet()))
         throw new RuntimeException("not all joint positions specified");

      trajectoryTimeProvider.set(time);

      for (OneDoFJoint oneDoFJoint : desiredJointPositions.keySet())
      {
         quinticPolynomialTrajectoryGenerators.get(oneDoFJoint).setFinalPosition(desiredJointPositions.get(oneDoFJoint));
      }

      jointSpaceHandControlState.setTrajectories(quinticPolynomialTrajectoryGenerators);
      requestedState.set(jointSpaceHandControlState.getStateEnum());
   }

   public void moveJointsInRange(Map<OneDoFJoint, Double> minJointPositions, Map<OneDoFJoint, Double> maxJointPositions, double time)
   {
      checkLimitsValid(minJointPositions, maxJointPositions);

      Map<OneDoFJoint, Double> allJointPositions = new LinkedHashMap<OneDoFJoint, Double>();
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         double q = oneDoFJoint.getQ();
         double qFinal = q;
         Double minJointPosition = minJointPositions.get(oneDoFJoint);

         if ((minJointPosition != null) && (q < minJointPosition))
         {
            qFinal = minJointPosition;
         }

         Double maxJointPosition = maxJointPositions.get(oneDoFJoint);
         if ((maxJointPosition != null) && (q > maxJointPosition))
            qFinal = maxJointPosition;

         allJointPositions.put(oneDoFJoint, qFinal);
      }

      moveUsingQuinticSplines(allJointPositions, time);
   }

   public boolean isHoldingObject()
   {
      return stateMachine.getCurrentStateEnum() == IndividualHandControlState.OBJECT_MANIPULATION;
   }

   public void holdPositionInBase()
   {
      ReferenceFrame endEffectorFrame = jacobian.getEndEffectorFrame();
      currentConfigurationProvider.set(new FramePose(endEffectorFrame));
      PositionTrajectoryGenerator positionTrajectory = holdPositionInBaseTrajectoryGenerator;
      OrientationTrajectoryGenerator orientationTrajectory = holdOrientationInBaseTrajectoryGenerator;
      executeTaskSpaceTrajectory(positionTrajectory, orientationTrajectory, endEffectorFrame, jacobian.getBase(), isHoldingObject());
   }

   private void checkLimitsValid(Map<OneDoFJoint, Double> minJointPositions, Map<OneDoFJoint, Double> maxJointPositions)
   {
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         Double minJointPosition = minJointPositions.get(oneDoFJoint);
         Double maxJointPosition = maxJointPositions.get(oneDoFJoint);
         if ((minJointPosition != null) && (maxJointPosition != null) && (minJointPosition > maxJointPosition))
            throw new RuntimeException("min > max");
      }
   }

   private StraightLinePositionTrajectoryGenerator getOrCreateStraightLinePositionTrajectoryGenerator(ReferenceFrame referenceFrame)
   {
      StraightLinePositionTrajectoryGenerator ret = straightLinePositionWorldTrajectoryGenerators.get(referenceFrame);
      if (ret == null)
      {
         ret = new StraightLinePositionTrajectoryGenerator(name + referenceFrame.getName(), referenceFrame, trajectoryTimeProvider,
                 currentConfigurationProvider, desiredConfigurationProvider, registry);
         straightLinePositionWorldTrajectoryGenerators.put(referenceFrame, ret);
      }

      return ret;
   }

   private OrientationInterpolationTrajectoryGenerator getOrCreateOrientationInterpolationTrajectoryGenerator(ReferenceFrame referenceFrame)
   {
      OrientationInterpolationTrajectoryGenerator ret = orientationInterpolationWorldTrajectoryGenerators.get(referenceFrame);
      if (ret == null)
      {
         ret = new OrientationInterpolationTrajectoryGenerator(name + referenceFrame.getName(), referenceFrame, trajectoryTimeProvider,
                 currentConfigurationProvider, desiredConfigurationProvider, registry);
         orientationInterpolationWorldTrajectoryGenerators.put(referenceFrame, ret);
      }

      return ret;
   }

   private RigidBodySpatialAccelerationControlModule getOrCreateRigidBodySpatialAccelerationControlModule(ReferenceFrame handPositionControlFrame)
   {
      RigidBodySpatialAccelerationControlModule ret = handSpatialAccelerationControlModules.get(handPositionControlFrame);
      if (ret == null)
      {
         ret = new RigidBodySpatialAccelerationControlModule(name + handPositionControlFrame.getName(), twistCalculator, jacobian.getEndEffector(),
                 handPositionControlFrame, registry);

         ret.setPositionProportionalGains(100.0, 100.0, 100.0);
         ret.setPositionDerivativeGains(20.0, 20.0, 20.0);
         ret.setOrientationProportionalGains(100.0, 100.0, 100.0);
         ret.setOrientationDerivativeGains(20.0, 20.0, 20.0);
         handSpatialAccelerationControlModules.put(handPositionControlFrame, ret);
      }

      return ret;
   }

   public boolean isControllingInTaskSpace()
   {
      return stateMachine.getCurrentState() instanceof TaskspaceHandControlState;
   }
}
