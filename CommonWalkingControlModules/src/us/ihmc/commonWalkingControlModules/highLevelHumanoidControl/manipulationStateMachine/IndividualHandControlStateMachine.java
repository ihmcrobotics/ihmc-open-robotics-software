package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.ManipulationControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.IndividualHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.JointSpaceHandControlControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.LoadBearingHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.MoveJointsInRangeState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.ObjectManipulationState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.TaskspaceHandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantConfigurationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SE3ConfigurationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionAction;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantDoubleProvider;

public class IndividualHandControlStateMachine
{
   private final YoVariableRegistry registry;

   private final Collection<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();

   private final StateMachine<IndividualHandControlState> stateMachine;

   private final RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule;
   private final BooleanYoVariable isReadyToBearLoad;

   final DesiredHandPoseProvider handPoseProvider;

   public IndividualHandControlStateMachine(final DoubleYoVariable simulationTime, final RobotSide robotSide, final FullRobotModel fullRobotModel,
           final ManipulationControllerParameters parameters, final TwistCalculator twistCalculator, ReferenceFrame handPositionControlFrame,
           final DesiredHandPoseProvider handPoseProvider, final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
           HandControllerInterface handController, double gravity, final double controlDT, MomentumBasedController momentumBasedController,
           GeometricJacobian jacobian, final YoVariableRegistry parentRegistry)
   {
      RigidBody endEffector = jacobian.getEndEffector();
      Map<OneDoFJoint, Double> defaultJointPositions = parameters.getDefaultArmJointPositions(fullRobotModel, robotSide);
      Map<OneDoFJoint, Double> minTaskSpacePositions = parameters.getMinTaskspaceArmJointPositions(fullRobotModel, robotSide);
      Map<OneDoFJoint, Double> maxTaskSpacePositions = parameters.getMaxTaskspaceArmJointPositions(fullRobotModel, robotSide);

      String name = endEffector.getName() + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);
      this.isReadyToBearLoad = new BooleanYoVariable(name + "isReadyToSwitchToLoadBearing", registry);
      stateMachine = new StateMachine<IndividualHandControlState>(name, name + "SwitchTime", IndividualHandControlState.class, simulationTime, registry);
      this.handPoseProvider = handPoseProvider;

      handSpatialAccelerationControlModule = new RigidBodySpatialAccelerationControlModule(endEffector.getName(), twistCalculator, endEffector,
              handPositionControlFrame, registry);

      handSpatialAccelerationControlModule.setPositionProportionalGains(100.0, 100.0, 100.0);
      handSpatialAccelerationControlModule.setPositionDerivativeGains(20.0, 20.0, 20.0);
      handSpatialAccelerationControlModule.setOrientationProportionalGains(100.0, 100.0, 100.0);
      handSpatialAccelerationControlModule.setOrientationDerivativeGains(20.0, 20.0, 20.0);


      // TODO: pass in state enums
      JointSpaceHandControlControlState defaultState = new JointSpaceHandControlControlState(IndividualHandControlState.DEFAULT, simulationTime, robotSide,
                                                          jacobian, momentumBasedController, registry, 1.0);
      defaultState.setDesiredJointPositions(defaultJointPositions);

      final ConstantConfigurationProvider currentConfigurationProvider = new ConstantConfigurationProvider(new FramePose(handPositionControlFrame));
      final ChangeableConfigurationProvider desiredConfigurationProvider = new ChangeableConfigurationProvider(handPoseProvider.getDesiredHandPose(robotSide));
      final TaskspaceHandControlState taskSpaceState = createTaskspaceHandControlState(IndividualHandControlState.OBJECT_MANIPULATION, robotSide,
                                                          fullRobotModel, dynamicGraphicObjectsListRegistry, handController, gravity, controlDT,
                                                          momentumBasedController, jacobian, parentRegistry, currentConfigurationProvider,
                                                          desiredConfigurationProvider);

      final MoveJointsInRangeState singularityEscapeState = new MoveJointsInRangeState(IndividualHandControlState.SINGULARITY_ESCAPE, minTaskSpacePositions,
                                                               maxTaskSpacePositions, simulationTime, robotSide, jacobian, momentumBasedController, registry,
                                                               0.3);

      final LoadBearingHandControlState loadBearingState = new LoadBearingHandControlState(IndividualHandControlState.LOAD_BEARING, momentumBasedController,
                                                              jacobian, parentRegistry, isReadyToBearLoad, dynamicGraphicObjectsListRegistry, robotSide,
                                                              parameters);



      StateTransition<IndividualHandControlState> defaultToTaskspaceCondition = createNewHandPoseAvailableTransition(robotSide, handPoseProvider,
                                                                                   desiredConfigurationProvider, taskSpaceState);
      defaultState.addStateTransition(defaultToTaskspaceCondition);



      StateTransition<IndividualHandControlState> taskspaceToTaskspaceCondition = createNewHandPoseAvailableTransition(robotSide, handPoseProvider,
                                                                                     desiredConfigurationProvider, taskSpaceState);
      taskSpaceState.addStateTransition(taskspaceToTaskspaceCondition);    // must be added after taskspaceToSingularityEscapeTransition


      StateTransition<IndividualHandControlState> taskSpaceToLoadBearingTransition = createTaskSpaceToLoadBearingTransition(robotSide, isReadyToBearLoad);
      taskSpaceState.addStateTransition(taskSpaceToLoadBearingTransition);


      StateTransition<IndividualHandControlState> taskSpaceToDefaultTransition = createTaskspaceToDefaultTransition(robotSide, defaultState);
      taskSpaceState.addStateTransition(taskSpaceToDefaultTransition);


      StateTransition<IndividualHandControlState> taskspaceToSingularityEscapeTransition = createSingularityEscapeTransition(robotSide, handPoseProvider,
                                                                                              desiredConfigurationProvider, singularityEscapeState);
      taskSpaceState.addStateTransition(taskspaceToSingularityEscapeTransition);


      StateTransition<IndividualHandControlState> singularityEscapeToTaskspaceTransition = createSingularityEscapeToTaskspaceTransition(taskSpaceState,
                                                                                              singularityEscapeState);
      singularityEscapeState.addStateTransition(singularityEscapeToTaskspaceTransition);


      StateTransition<IndividualHandControlState> loadBearingToTaskSpaceTransition = createLoadBearingToTaskSpaceTransition(robotSide, isReadyToBearLoad);
      loadBearingState.addStateTransition(loadBearingToTaskSpaceTransition);


      addState(defaultState);
      addState(taskSpaceState);
      addState(singularityEscapeState);
      addState(loadBearingState);

      parentRegistry.addChild(registry);
   }

   private StateTransition<IndividualHandControlState> createTaskSpaceToLoadBearingTransition(final RobotSide robotSide,
           final BooleanYoVariable isReadyToSwitchToLoadBearing)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return isReadyToSwitchToLoadBearing.getBooleanValue();    // || handPoseProvider.isInContact(robotSide);
         }
      };
      StateTransitionAction stateTransitionAction = new StateTransitionAction()
      {
         public void doTransitionAction()
         {
            isReadyToSwitchToLoadBearing.set(true);
         }
      };

      return new StateTransition<IndividualHandControlState>(IndividualHandControlState.LOAD_BEARING, stateTransitionCondition, stateTransitionAction);
   }

   private StateTransition<IndividualHandControlState> createLoadBearingToTaskSpaceTransition(final RobotSide robotSide,
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

      return new StateTransition<IndividualHandControlState>(IndividualHandControlState.OBJECT_MANIPULATION, stateTransitionCondition, stateTransitionAction);
   }

   private StateTransition<IndividualHandControlState> createTaskspaceToDefaultTransition(final RobotSide robotSide,
           State<IndividualHandControlState> defaultState)
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

      return new StateTransition<IndividualHandControlState>(defaultState.getStateEnum(), stateTransitionCondition, stateTransitionAction);
   }

   private StateTransition<IndividualHandControlState> createSingularityEscapeToTaskspaceTransition(TaskspaceHandControlState taskSpaceState,
           final MoveJointsInRangeState singularityEscapeState)
   {
      StateTransitionCondition condition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return singularityEscapeState.isDone();
         }
      };

      return new StateTransition<IndividualHandControlState>(taskSpaceState.getStateEnum(), condition);
   }

   private StateTransition<IndividualHandControlState> createNewHandPoseAvailableTransition(final RobotSide robotSide,
           final DesiredHandPoseProvider handPoseProvider, final ChangeableConfigurationProvider desiredConfigurationProvider,
           State<IndividualHandControlState> goalState)
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

      final StateTransition<IndividualHandControlState> ret = new StateTransition<IndividualHandControlState>(goalState.getStateEnum(),
                                                                 nextPoseAvailableCondition, setDesiredPoseBasedOnProvider);

      return ret;
   }

   private StateTransition<IndividualHandControlState> createSingularityEscapeTransition(final RobotSide robotSide,
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

      final StateTransition<IndividualHandControlState> ret = new StateTransition<IndividualHandControlState>(goalState.getStateEnum(),
                                                                 nextPoseAvailableCondition, setDesiredPoseBasedOnProvider);

      return ret;
   }

   private TaskspaceHandControlState createTaskspaceHandControlState(IndividualHandControlState stateEnum, RobotSide robotSide, FullRobotModel fullRobotModel,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, HandControllerInterface handController, double gravity, double controlDT,
           MomentumBasedController momentumBasedController, GeometricJacobian jacobian, YoVariableRegistry parentRegistry,
           SE3ConfigurationProvider currentConfigurationProvider, SE3ConfigurationProvider desiredConfigurationProvider)
   {
      ConstantDoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(1.0);

      RigidBody base = jacobian.getBase();    // fullRobotModel.getElevator(); //  TODO: would actually like to have this be elevator, but not currently handled in
      ReferenceFrame referenceFrame = fullRobotModel.getElevator().getBodyFixedFrame();    // jacobian.getBase().getBodyFixedFrame();

      String namePrefix = FormattingTools.underscoredToCamelCase(stateEnum.toString(), true);
      StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame, 1.0,
                                                                               currentConfigurationProvider, desiredConfigurationProvider, registry);

      OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame,
                                                                                      trajectoryTimeProvider, currentConfigurationProvider,
                                                                                      desiredConfigurationProvider, registry);

      TaskspaceHandControlState taskspaceHandControlState = new ObjectManipulationState(stateEnum, robotSide, positionTrajectoryGenerator,
                                                               orientationTrajectoryGenerator, handSpatialAccelerationControlModule, momentumBasedController,
                                                               jacobian, base, handController, fullRobotModel, gravity, controlDT,
                                                               dynamicGraphicObjectsListRegistry, parentRegistry);

      return taskspaceHandControlState;
   }

   public void initialize()
   {
      if (handPoseProvider.isRelativeToWorld())
      {
         stateMachine.setCurrentState(IndividualHandControlState.OBJECT_MANIPULATION);    // TODO: why?
      }
      else
      {
         stateMachine.setCurrentState(IndividualHandControlState.DEFAULT);
      }
   }

   public void doControl()
   {
      stateMachine.checkTransitionConditionsThoroughly();
      stateMachine.doAction();

      for (DynamicGraphicReferenceFrame frame : dynamicGraphicReferenceFrames)
      {
         frame.update();
      }
   }

   private void addState(State<IndividualHandControlState> state)
   {
      stateMachine.addState(state);
   }

   public boolean inToroidManipulationState()
   {
      return stateMachine.getCurrentStateEnum() == IndividualHandControlState.TWO_HANDED_TOROID_MANIPULATION;
   }

   private static class ChangeableConfigurationProvider implements SE3ConfigurationProvider
   {
      private final FramePose configuration;

      public ChangeableConfigurationProvider(FramePose initialConfiguration)
      {
         configuration = new FramePose(initialConfiguration);
      }

      public void get(FramePose framePose)
      {
         framePose.setIncludingFrame(configuration);
      }

      public void get(FramePoint positionToPack)
      {
         configuration.getPosition(positionToPack);
      }

      public void get(FrameOrientation orientationToPack)
      {
         configuration.getOrientation(orientationToPack);
      }

      public void set(FramePose newPose)
      {
         configuration.setIncludingFrame(newPose);
      }

   }
}
