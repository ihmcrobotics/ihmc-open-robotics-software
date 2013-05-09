package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.statemachines.*;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantDoubleProvider;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.IndividualHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.JointSpaceHandControlControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.TaskspaceObjectManipulationState;
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
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Map;

public class IndividualHandControlStateMachine
{
   private final YoVariableRegistry registry;

   private final RobotSide robotSide;

   private final Collection<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();

   private final StateMachine<IndividualHandControlState> stateMachine;

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
      stateMachine = new StateMachine<IndividualHandControlState>(name, name + "SwitchTime", IndividualHandControlState.class, simulationTime, registry);
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

      JointSpaceHandControlControlState moveInJointSpaceState =
         new JointSpaceHandControlControlState(simulationTime, robotSide, jacobian, momentumBasedController,
            defaultJointPositions, registry);

      ConstantDoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(1.0);

      ReferenceFrame referenceFrame = jacobian.getBase().getBodyFixedFrame();

      String namePrefix = FormattingTools.underscoredToCamelCase(IndividualHandControlState.MOVE_HAND_TO_POSITION_IN_WORLDFRAME.toString(), true);
      StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame,
            1.0, currentConfigurationProvider, desiredConfigurationProvider,
                                                                               registry);

      OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame,
                                                                                      trajectoryTimeProvider, currentConfigurationProvider,
            desiredConfigurationProvider, registry);

      final TaskspaceObjectManipulationState moveRelativeToWorldState = new TaskspaceObjectManipulationState(this.robotSide,
                                                                   positionTrajectoryGenerator, orientationTrajectoryGenerator,
                                                                   handSpatialAccelerationControlModule, momentumBasedController, jacobian, handController, fullRobotModel, gravity, controlDT,
            dynamicGraphicObjectsListRegistry, registry);

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

      final StateTransition<IndividualHandControlState> toRelativeToWorldPositionTransition =
         new StateTransition<IndividualHandControlState>(moveRelativeToWorldState.getStateEnum(), toNextWorldPosition,
                             Arrays.asList(setCurrentPoseBasedOnPreviousDesired, setDesiredPoseBasedOnProvider));

      moveInJointSpaceState.addStateTransition(toRelativeToWorldPositionTransition);

      moveRelativeToWorldState.addStateTransition(toRelativeToWorldPositionTransition);

      addState(moveInJointSpaceState);
      addState(moveRelativeToWorldState);

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
         stateMachine.setCurrentState(IndividualHandControlState.MOVE_HAND_TO_POSITION_IN_WORLDFRAME);
      }
      else
      {
         stateMachine.setCurrentState(IndividualHandControlState.JOINT_SPACE);
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
