package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import java.util.*;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.JointSpaceHandControlControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.TaskspaceObjectManipulationState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.IndividualManipulationState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SE3ConfigurationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionAction;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantDoubleProvider;

public class IndividualHandControlStateMachine
{
   private enum ManipulationState {MOVE_HAND_TO_POSITION_IN_CHESTFRAME, JOINT_SPACE, MOVE_HAND_TO_POSITION_IN_WORLDFRAME};

   private final YoVariableRegistry registry;

   private final RobotSide robotSide;

   private final Collection<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();

   private final StateMachine<ManipulationState> stateMachine;
   private final EnumMap<ManipulationState, IndividualManipulationState<ManipulationState>> manipulationStateMap =
      new EnumMap<ManipulationState, IndividualManipulationState<ManipulationState>>(ManipulationState.class);

   private final RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule;

   final DesiredHandPoseProvider handPoseProvider;

   public IndividualHandControlStateMachine(final DoubleYoVariable simulationTime, final RobotSide robotSide, final FullRobotModel fullRobotModel,
                                            final TwistCalculator twistCalculator,
                                            WalkingControllerParameters walkingControllerParameters, final DesiredHandPoseProvider handPoseProvider,
                                            final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, HandControllerInterface handController,
                                            double gravity, final double controlDT, MomentumBasedController momentumBasedController, GeometricJacobian jacobian,
                                            Map<OneDoFJoint, Double> defaultJointPositions, final YoVariableRegistry parentRegistry)
   {
      RigidBody endEffector = jacobian.getEndEffector();

      String name = endEffector.getName() + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);
      stateMachine = new StateMachine<ManipulationState>(name, name + "SwitchTime", ManipulationState.class, simulationTime, registry);
      this.robotSide = robotSide;
      this.handPoseProvider = handPoseProvider;

      String frameName = endEffector.getName() + "PositionControlFrame";
      final ReferenceFrame frameAfterJoint = endEffector.getParentJoint().getFrameAfterJoint();
      ReferenceFrame endEffectorFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, frameAfterJoint,
                                           walkingControllerParameters.getHandControlFramesWithRespectToFrameAfterWrist().get(robotSide));

      handSpatialAccelerationControlModule = new RigidBodySpatialAccelerationControlModule(endEffector.getName(), twistCalculator, endEffector,
              endEffectorFrame, registry);

      handSpatialAccelerationControlModule.setPositionProportionalGains(100.0, 100.0, 100.0);
      handSpatialAccelerationControlModule.setPositionDerivativeGains(20.0, 20.0, 20.0);
      handSpatialAccelerationControlModule.setOrientationProportionalGains(100.0, 100.0, 100.0);
      handSpatialAccelerationControlModule.setOrientationDerivativeGains(20.0, 20.0, 20.0);

      final ChangeableConfigurationProvider currentConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(endEffectorFrame));
      final ChangeableConfigurationProvider desiredConfigurationProvider = new ChangeableConfigurationProvider(handPoseProvider.getDesiredHandPose(robotSide));

//    TaskspaceObjectManipulationState<ManipulationState> moveRelativeToChestState = createIndividualHandControlState(1.0,
//          ManipulationState.MOVE_HAND_TO_POSITION_IN_CHESTFRAME,
//          currentConfigurationProvider, desiredConfigurationProvider,
//          momentumBasedController, jacobian, dynamicGraphicObjectsListRegistry);


      JointSpaceHandControlControlState<ManipulationState> moveInJointSpaceState =
         new JointSpaceHandControlControlState<ManipulationState>(ManipulationState.JOINT_SPACE, simulationTime, robotSide, jacobian, momentumBasedController,
            defaultJointPositions, registry);

      ConstantDoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(1.0);

      ReferenceFrame referenceFrame = jacobian.getBase().getBodyFixedFrame();

      String namePrefix = FormattingTools.underscoredToCamelCase(ManipulationState.MOVE_HAND_TO_POSITION_IN_WORLDFRAME.toString(), true);
      StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame,
            1.0, currentConfigurationProvider, desiredConfigurationProvider,
                                                                               registry);

      OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame,
                                                                                      trajectoryTimeProvider, currentConfigurationProvider,
            desiredConfigurationProvider, registry);

      final TaskspaceObjectManipulationState<ManipulationState> ret = new TaskspaceObjectManipulationState<ManipulationState>(ManipulationState.MOVE_HAND_TO_POSITION_IN_WORLDFRAME, this.robotSide,
                                                                   positionTrajectoryGenerator, orientationTrajectoryGenerator,
                                                                   handSpatialAccelerationControlModule, momentumBasedController, jacobian, handController, fullRobotModel, gravity, controlDT,
            dynamicGraphicObjectsListRegistry, registry);

      TaskspaceObjectManipulationState<ManipulationState> moveRelativeToWorldState = ret;

//      StateTransitionCondition toNextChestPosition = new StateTransitionCondition()
//      {
//         public boolean checkCondition()
//         {
//            return handPoseProvider.checkForNewPose(robotSide) &&!handPoseProvider.isRelativeToWorld();
//         }
//      };

      StateTransitionCondition toNextWorldPosition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return handPoseProvider.checkForNewPose(robotSide) && handPoseProvider.isRelativeToWorld();
         }
      };

      StateTransitionAction setCurrentPoseBasedOnPreviousDesired = new StateTransitionAction()
      {
         private final FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame());

         public void doTransitionAction()
         {
            // Set current configuration to desired of previous state
            desiredConfigurationProvider.get(framePose);
            currentConfigurationProvider.set(framePose);
         }
      };

      StateTransitionAction setDesiredPoseBasedOnProvider = new StateTransitionAction()
      {
         public void doTransitionAction()
         {
            desiredConfigurationProvider.set(handPoseProvider.getDesiredHandPose(robotSide));
         }
      };

//      final StateTransition<ManipulationState> toRelativeToChestPositionTransition =
//         new StateTransition<IndividualHandControlStateMachine.ManipulationState>(ManipulationState.MOVE_HAND_TO_POSITION_IN_CHESTFRAME, toNextChestPosition,
//                             Arrays.asList(setCurrentPoseBasedOnPreviousDesired, setDesiredPoseBasedOnProvider));
      final StateTransition<ManipulationState> toRelativeToWorldPositionTransition =
         new StateTransition<IndividualHandControlStateMachine.ManipulationState>(ManipulationState.MOVE_HAND_TO_POSITION_IN_WORLDFRAME, toNextWorldPosition,
                             Arrays.asList(setCurrentPoseBasedOnPreviousDesired, setDesiredPoseBasedOnProvider));

//      moveRelativeToChestState.addStateTransition(toRelativeToChestPositionTransition);
//      moveRelativeToChestState.addStateTransition(toRelativeToWorldPositionTransition);
      moveInJointSpaceState.addStateTransition(toRelativeToWorldPositionTransition);

//      moveRelativeToWorldState.addStateTransition(toRelativeToChestPositionTransition);
      moveRelativeToWorldState.addStateTransition(toRelativeToWorldPositionTransition);

//      addState(moveRelativeToChestState);
      addState(moveInJointSpaceState);
      addState(moveRelativeToWorldState);



//    ControlFlowOutputPort<TaskspaceConstraintData> desiredAccelerationOutputPort = createOutputPort("desiredAccelerationOutputPort");
//    desiredAccelerationOutputPort.setData(taskspaceConstraintData);

      if (dynamicGraphicObjectsListRegistry != null)
      {
         DynamicGraphicObjectsList list = new DynamicGraphicObjectsList(name);

         DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame = new DynamicGraphicReferenceFrame(endEffectorFrame, registry, 0.3);
         dynamicGraphicReferenceFrames.add(dynamicGraphicReferenceFrame);
         list.add(dynamicGraphicReferenceFrame);

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(list);
         list.hideDynamicGraphicObjects();
      }

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      if (handPoseProvider.isRelativeToWorld())
      {
         stateMachine.setCurrentState(ManipulationState.MOVE_HAND_TO_POSITION_IN_WORLDFRAME);
      }
      else
      {
//         stateMachine.setCurrentState(ManipulationState.MOVE_HAND_TO_POSITION_IN_CHESTFRAME);
         stateMachine.setCurrentState(ManipulationState.JOINT_SPACE);
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

   private void addState(IndividualManipulationState<ManipulationState> state)
   {
      stateMachine.addState(state);
      manipulationStateMap.put(state.getStateEnum(), state);
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
