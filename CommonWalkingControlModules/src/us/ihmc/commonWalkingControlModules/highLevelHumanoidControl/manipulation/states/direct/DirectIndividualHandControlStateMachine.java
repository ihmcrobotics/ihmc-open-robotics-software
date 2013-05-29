package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.states.direct;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.statemachines.*;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantDoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.configurations.ManipulationControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandPoseProvider;
import us.ihmc.commonWalkingControlModules.sensors.MassMatrixEstimatingToolRigidBody;
import us.ihmc.commonWalkingControlModules.trajectories.*;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;

public class DirectIndividualHandControlStateMachine
{
   private static final double TASK_SPACE_TRAJECTORY_TIME = 1.0;

   private final YoVariableRegistry registry;

   private final Collection<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();
   private final StateMachine<DirectIndividualHandControlState> stateMachine;
   private final RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule;
   private final BooleanYoVariable isReadyToBearLoad;
   final DesiredHandPoseProvider handPoseProvider;

   private final HandControllerInterface handController;
   private final MassMatrixEstimatingToolRigidBody toolBody;
   private BooleanYoVariable prepareForLocomotion;


   public DirectIndividualHandControlStateMachine(final DoubleYoVariable simulationTime, final RobotSide robotSide, final FullRobotModel fullRobotModel,
                                                  final ManipulationControllerParameters parameters, final TwistCalculator twistCalculator,
                                                  ReferenceFrame handPositionControlFrame,
                                                  final DesiredHandPoseProvider handPoseProvider, final DynamicGraphicObjectsListRegistry
         dynamicGraphicObjectsListRegistry,
                                                  HandControllerInterface handController, double gravity, final double controlDT, MomentumBasedController momentumBasedController,
                                                  GeometricJacobian jacobian, final YoVariableRegistry parentRegistry)
   {
      RigidBody endEffector = jacobian.getEndEffector();
      Map<OneDoFJoint, Double> defaultJointPositions = parameters.getDefaultArmJointPositions(fullRobotModel, robotSide);
      Map<OneDoFJoint, Double> minTaskSpacePositions = parameters.getMinTaskspaceArmJointPositions(fullRobotModel, robotSide);
      Map<OneDoFJoint, Double> maxTaskSpacePositions = parameters.getMaxTaskspaceArmJointPositions(fullRobotModel, robotSide);

      String name = endEffector.getName() + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);

      prepareForLocomotion = new BooleanYoVariable("prepareForLocomotion", registry);

      this.toolBody = new MassMatrixEstimatingToolRigidBody(name + "Tool", handController.getWristJoint(), fullRobotModel, gravity, controlDT, registry,
              dynamicGraphicObjectsListRegistry);


      this.handController = handController;

      this.isReadyToBearLoad = new BooleanYoVariable(name + "IsReadyToSwitchToLoadBearing", registry);
      stateMachine = new StateMachine<DirectIndividualHandControlState>(name, name + "SwitchTime", DirectIndividualHandControlState.class, simulationTime, registry);
      this.handPoseProvider = handPoseProvider;

      handSpatialAccelerationControlModule = new RigidBodySpatialAccelerationControlModule(endEffector.getName(), twistCalculator, endEffector,
              handPositionControlFrame, registry);

      handSpatialAccelerationControlModule.setPositionProportionalGains(100.0, 100.0, 100.0);
      handSpatialAccelerationControlModule.setPositionDerivativeGains(20.0, 20.0, 20.0);
      handSpatialAccelerationControlModule.setOrientationProportionalGains(100.0, 100.0, 100.0);
      handSpatialAccelerationControlModule.setOrientationDerivativeGains(20.0, 20.0, 20.0);

      final ConstantConfigurationProvider currentConfigurationProvider = new ConstantConfigurationProvider(new FramePose(handPositionControlFrame));
      final ChangeableConfigurationProvider desiredConfigurationProvider = new ChangeableConfigurationProvider(handPoseProvider.getDesiredHandPose(robotSide));

      final JointSpaceHandControlControlState defaultState = new JointSpaceHandControlControlState(DirectIndividualHandControlState.DEFAULT, simulationTime,
                                                                robotSide, jacobian, momentumBasedController, registry, 1.0);
      defaultState.setDesiredJointPositions(defaultJointPositions);

      final TaskspaceHandControlState objectManipulationInWorldState = createTaskspaceWorldHandControlState(DirectIndividualHandControlState.OBJECT_MANIPULATION,
                                                                          robotSide, fullRobotModel, dynamicGraphicObjectsListRegistry, handController,
                                                                          momentumBasedController, jacobian, parentRegistry, currentConfigurationProvider,
                                                                          desiredConfigurationProvider);

      final TaskspaceHandControlState objectManipulationInChestState =
         createTaskspaceChestHandControlState(DirectIndividualHandControlState.OBJECT_MANIPULATION_CHEST, robotSide, fullRobotModel,
            dynamicGraphicObjectsListRegistry, handController, momentumBasedController, jacobian, registry, currentConfigurationProvider);

      final MoveJointsInRangeState singularityEscapeState = new MoveJointsInRangeState(DirectIndividualHandControlState.SINGULARITY_ESCAPE, minTaskSpacePositions,
                                                               maxTaskSpacePositions, simulationTime, robotSide, jacobian, momentumBasedController, registry,
                                                               0.3);

      final LoadBearingCylindricalHandControlState loadBearingCylindricalState =
         new LoadBearingCylindricalHandControlState(DirectIndividualHandControlState.LOAD_BEARING_CYLINDRICAL, momentumBasedController, jacobian, parentRegistry,
            isReadyToBearLoad, dynamicGraphicObjectsListRegistry, robotSide, parameters);

//    final LoadBearingPlaneHandControlState loadBearingPlaneState = new LoadBearingPlaneHandControlState(DirectIndividualHandControlState.LOAD_BEARING_PLANE,
//          robotSide, handPositionControlFrame, handSpatialAccelerationControlModule, momentumBasedController, jacobian, jacobian.getBase(), handController,
//          isReadyToBearLoad, dynamicGraphicObjectsListRegistry, registry);

      addNewHandPoseAvailableTransition(robotSide, handPoseProvider, desiredConfigurationProvider, defaultState, objectManipulationInWorldState);
      addNewHandPoseAvailableTransition(robotSide, handPoseProvider, desiredConfigurationProvider, objectManipulationInChestState,
                                        objectManipulationInWorldState);
      addNewHandPoseAvailableTransition(robotSide, handPoseProvider, desiredConfigurationProvider, objectManipulationInWorldState,
                                        objectManipulationInWorldState);    // must be added after taskspaceToSingularityEscapeTransition

      addTransitionToCylindricalLoadBearing(defaultState);
      addTransitionToCylindricalLoadBearing(objectManipulationInWorldState);

      addTransitionToDefault(robotSide, objectManipulationInWorldState, defaultState);
      addTransitionToDefault(robotSide, objectManipulationInChestState, defaultState);

      // TODO: figure out singularity escape again
      addTransitionToSingularityEscape(robotSide, handPoseProvider, desiredConfigurationProvider, objectManipulationInWorldState, singularityEscapeState);
      addSingularityEscapeToTaskspaceTransition(objectManipulationInWorldState, singularityEscapeState);

      addLoadBearingToTaskspaceTransition(robotSide, loadBearingCylindricalState);

      addTaskspaceWorldToChestTransition(objectManipulationInWorldState, objectManipulationInChestState);

      addState(defaultState);
      addState(objectManipulationInWorldState);
      addState(objectManipulationInChestState);
      addState(singularityEscapeState);
      addState(loadBearingCylindricalState);

//    addState(loadBearingPlaneState);

      parentRegistry.addChild(registry);
   }

   private void addTaskspaceWorldToChestTransition(TaskspaceHandControlState objectManipulationInWorldState, TaskspaceHandControlState
         objectManipulationInChestState)
   {
      StateTransitionCondition condition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return prepareForLocomotion.getBooleanValue();
         }
      };
      StateTransition<DirectIndividualHandControlState> stateTransition = new StateTransition<DirectIndividualHandControlState>(objectManipulationInChestState.getStateEnum(), condition);
      objectManipulationInWorldState.addStateTransition(stateTransition);
   }

   private void addSingularityEscapeToTaskspaceTransition(TaskspaceHandControlState objectManipulationInWorldState, MoveJointsInRangeState
         singularityEscapeState)
   {
      StateTransition<DirectIndividualHandControlState> singularityEscapeToTaskspaceTransition =
         createSingularityEscapeToTaskspaceTransition(objectManipulationInWorldState, singularityEscapeState);
      singularityEscapeState.addStateTransition(singularityEscapeToTaskspaceTransition);
   }

   private void addLoadBearingToTaskspaceTransition(RobotSide robotSide, LoadBearingCylindricalHandControlState loadBearingCylindricalState)
   {
      StateTransition<DirectIndividualHandControlState> loadBearingToTaskSpaceTransition = createLoadBearingToTaskSpaceTransition(robotSide, isReadyToBearLoad);
      loadBearingCylindricalState.addStateTransition(loadBearingToTaskSpaceTransition);
   }

   private void addTransitionToSingularityEscape(RobotSide robotSide, DesiredHandPoseProvider handPoseProvider, ChangeableConfigurationProvider
         desiredConfigurationProvider, State<DirectIndividualHandControlState> objectManipulationInWorldState, MoveJointsInRangeState singularityEscapeState)
   {
      StateTransition<DirectIndividualHandControlState> taskspaceToSingularityEscapeTransition = createSingularityEscapeTransition(robotSide, handPoseProvider,
                                                                                              desiredConfigurationProvider, singularityEscapeState);
      objectManipulationInWorldState.addStateTransition(taskspaceToSingularityEscapeTransition);
   }

   private void addTransitionToDefault(final RobotSide robotSide, TaskspaceHandControlState fromState, JointSpaceHandControlControlState defaultState)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            // TODO: hack!
            return handPoseProvider.checkForNewPose(robotSide) &&!handPoseProvider.isRelativeToWorld();
         }
      };
      StateTransitionAction stateTransitionAction = new StateTransitionAction()
      {
         public void doTransitionAction()
         {
            // just consume the hand pose
            handPoseProvider.getDesiredHandPose(robotSide);
         }
      };

      StateTransition<DirectIndividualHandControlState> taskSpaceToDefaultTransition = new StateTransition<DirectIndividualHandControlState>(defaultState.getStateEnum(),
                                                                                    stateTransitionCondition, stateTransitionAction);
      fromState.addStateTransition(taskSpaceToDefaultTransition);
   }

   private void addNewHandPoseAvailableTransition(final RobotSide robotSide, final DesiredHandPoseProvider handPoseProvider,
           final ChangeableConfigurationProvider desiredConfigurationProvider, State<DirectIndividualHandControlState> fromState,
           State<DirectIndividualHandControlState> toState)
   {
      StateTransitionCondition nextPoseAvailableCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return handPoseProvider.checkForNewPose(robotSide) && handPoseProvider.isRelativeToWorld();
         }
      };

      StateTransitionAction setDesiredPoseBasedOnProvider = new StateTransitionAction()
      {
         public void doTransitionAction()
         {
            desiredConfigurationProvider.set(handPoseProvider.getDesiredHandPose(robotSide));
         }
      };

      final StateTransition<DirectIndividualHandControlState> ret = new StateTransition<DirectIndividualHandControlState>(toState.getStateEnum(),
                                                                 nextPoseAvailableCondition, setDesiredPoseBasedOnProvider);

      StateTransition<DirectIndividualHandControlState> defaultToTaskspaceCondition = ret;
      fromState.addStateTransition(defaultToTaskspaceCondition);
   }

   private void addTransitionToCylindricalLoadBearing(State<DirectIndividualHandControlState> fromState)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return isReadyToBearLoad.getBooleanValue() && (handController != null) && handController.isClosed();
         }
      };
      StateTransitionAction stateTransitionAction = new StateTransitionAction()
      {
         public void doTransitionAction()
         {
            isReadyToBearLoad.set(true);
         }
      };

      StateTransition<DirectIndividualHandControlState> defaultToCylindricalLoadBearingTransition =
         new StateTransition<DirectIndividualHandControlState>(DirectIndividualHandControlState.LOAD_BEARING_CYLINDRICAL, stateTransitionCondition, stateTransitionAction);
      fromState.addStateTransition(defaultToCylindricalLoadBearingTransition);
   }

   // private StateTransition<DirectIndividualHandControlState> createTaskSpaceToPlaneLoadBearingTransition(final RobotSide robotSide,
// final BooleanYoVariable isReadyToSwitchToLoadBearing)
// {
//    StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
//    {
//       public boolean checkCondition()
//       {
//  return isReadyToSwitchToLoadBearing.getBooleanValue() && handController != null && handController.isOpen();
//       }
//    };
//    StateTransitionAction stateTransitionAction = new StateTransitionAction()
//    {
//       public void doTransitionAction()
//       {
//  isReadyToSwitchToLoadBearing.set(true);
//       }
//    };
//
//    return new StateTransition<DirectIndividualHandControlState>(DirectIndividualHandControlState.LOAD_BEARING_PLANE, stateTransitionCondition, stateTransitionAction);
// }

   private StateTransition<DirectIndividualHandControlState> createLoadBearingToTaskSpaceTransition(final RobotSide robotSide,
           final BooleanYoVariable isReadyToSwitchToLoadBearing)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return !isReadyToSwitchToLoadBearing.getBooleanValue();    // ||!handPoseProvider.isInContact(robotSide);
         }
      };
      StateTransitionAction stateTransitionAction = new StateTransitionAction()
      {
         public void doTransitionAction()
         {
            isReadyToSwitchToLoadBearing.set(false);
         }
      };

      return new StateTransition<DirectIndividualHandControlState>(DirectIndividualHandControlState.OBJECT_MANIPULATION, stateTransitionCondition, stateTransitionAction);
   }

   private StateTransition<DirectIndividualHandControlState> createSingularityEscapeToTaskspaceTransition(TaskspaceHandControlState taskSpaceState,
           final MoveJointsInRangeState singularityEscapeState)
   {
      StateTransitionCondition condition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return singularityEscapeState.isDone();
         }
      };

      return new StateTransition<DirectIndividualHandControlState>(taskSpaceState.getStateEnum(), condition);
   }

   private StateTransition<DirectIndividualHandControlState> createSingularityEscapeTransition(final RobotSide robotSide,
           final DesiredHandPoseProvider handPoseProvider, final ChangeableConfigurationProvider desiredConfigurationProvider,
           final MoveJointsInRangeState goalState)
   {
      StateTransitionCondition nextPoseAvailableCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            boolean newPoseAvailable = handPoseProvider.checkForNewPose(robotSide) && handPoseProvider.isRelativeToWorld();

            return newPoseAvailable &&!goalState.areJointsInRange();
         }
      };

      StateTransitionAction setDesiredPoseBasedOnProvider = new StateTransitionAction()
      {
         public void doTransitionAction()
         {
            desiredConfigurationProvider.set(handPoseProvider.getDesiredHandPose(robotSide));
         }
      };

      final StateTransition<DirectIndividualHandControlState> ret = new StateTransition<DirectIndividualHandControlState>(goalState.getStateEnum(),
                                                                 nextPoseAvailableCondition, setDesiredPoseBasedOnProvider);

      return ret;
   }

   private TaskspaceHandControlState createTaskspaceWorldHandControlState(DirectIndividualHandControlState stateEnum, RobotSide robotSide,
           FullRobotModel fullRobotModel, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, HandControllerInterface handController,
           MomentumBasedController momentumBasedController, GeometricJacobian jacobian, YoVariableRegistry parentRegistry,
           SE3ConfigurationProvider currentConfigurationProvider, SE3ConfigurationProvider desiredConfigurationProvider)
   {
      ConstantDoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(TASK_SPACE_TRAJECTORY_TIME);

      RigidBody base = fullRobotModel.getElevator();
      ReferenceFrame referenceFrame = base.getBodyFixedFrame();

      String namePrefix = FormattingTools.underscoredToCamelCase(stateEnum.toString(), true);
      StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame,
                                                                               TASK_SPACE_TRAJECTORY_TIME, currentConfigurationProvider,
                                                                               desiredConfigurationProvider, registry, false);

      OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame,
                                                                                      trajectoryTimeProvider, currentConfigurationProvider,
                                                                                      desiredConfigurationProvider, registry, false);

      TaskspaceHandControlState taskspaceHandControlState = new ObjectManipulationState(stateEnum, robotSide, positionTrajectoryGenerator,
                                                               orientationTrajectoryGenerator, handSpatialAccelerationControlModule, momentumBasedController,
                                                               jacobian, base, handController, toolBody, dynamicGraphicObjectsListRegistry, parentRegistry);

      return taskspaceHandControlState;
   }

   private TaskspaceHandControlState createTaskspaceChestHandControlState(DirectIndividualHandControlState stateEnum, RobotSide robotSide,
           FullRobotModel fullRobotModel, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, HandControllerInterface handController,
           MomentumBasedController momentumBasedController, GeometricJacobian jacobian, YoVariableRegistry parentRegistry,
           SE3ConfigurationProvider currentConfigurationProvider)
   {
      RigidBody base = fullRobotModel.getChest();
      ReferenceFrame referenceFrame = base.getBodyFixedFrame();

      String namePrefix = FormattingTools.underscoredToCamelCase(stateEnum.toString(), true);
      PositionTrajectoryGenerator positionTrajectoryGenerator = new ConstantPositionTrajectoryGenerator(namePrefix, referenceFrame,
                                                                   currentConfigurationProvider, TASK_SPACE_TRAJECTORY_TIME, registry);

      OrientationTrajectoryGenerator orientationTrajectoryGenerator = new ConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame,
                                                                         currentConfigurationProvider, TASK_SPACE_TRAJECTORY_TIME, registry);

      TaskspaceHandControlState taskspaceHandControlState = new ObjectManipulationState(stateEnum, robotSide, positionTrajectoryGenerator,
                                                               orientationTrajectoryGenerator, handSpatialAccelerationControlModule, momentumBasedController,
                                                               jacobian, base, handController, toolBody, dynamicGraphicObjectsListRegistry, parentRegistry);

      return taskspaceHandControlState;
   }

   public void initialize()
   {
      if (handPoseProvider.isRelativeToWorld())
      {
         stateMachine.setCurrentState(DirectIndividualHandControlState.OBJECT_MANIPULATION);    // TODO: why?
      }
      else
      {
         stateMachine.setCurrentState(DirectIndividualHandControlState.DEFAULT);
      }
   }

   public void doControl()
   {
      stateMachine.checkTransitionConditionsThoroughly();
      prepareForLocomotion.set(false);
      stateMachine.doAction();

      for (DynamicGraphicReferenceFrame frame : dynamicGraphicReferenceFrames)
      {
         frame.update();
      }
   }

   private void addState(State<DirectIndividualHandControlState> state)
   {
      stateMachine.addState(state);
   }

   public void goToDefaultState()
   {
      DirectIndividualHandControlState defaultState = DirectIndividualHandControlState.DEFAULT;
      if (stateMachine.getCurrentStateEnum() != defaultState)
         stateMachine.setCurrentState(defaultState);
   }

   public void prepareForLocomotion()
   {
      this.prepareForLocomotion.set(true);
   }
}
