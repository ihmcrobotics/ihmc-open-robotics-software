package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import java.util.*;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.IndividualHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.IndividualManipulationState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.JointSpaceManipulationControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.sensors.MassMatrixEstimatingToolRigidBody;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SE3ConfigurationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.controlFlow.AbstractControlFlowElement;
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

public class ManipulationStateMachine extends AbstractControlFlowElement
{
   private enum ManipulationState {MOVE_HAND_TO_POSITION_IN_CHESTFRAME, JOINT_SPACE, MOVE_HAND_TO_POSITION_IN_WORLDFRAME}

   ;

   private final YoVariableRegistry registry;

   private final RobotSide robotSide;

   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final Collection<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();

   private final StateMachine<ManipulationState> stateMachine;
   private final EnumMap<ManipulationState, IndividualManipulationState<ManipulationState>> manipulationStateMap =
      new EnumMap<ManipulationState, IndividualManipulationState<ManipulationState>>(ManipulationState.class);

   private final RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule;

   private final HandControllerInterface handController;

   private final MassMatrixEstimatingToolRigidBody toolBody;
   private final Wrench measuredWristWrench = new Wrench();

   final DesiredHandPoseProvider handPoseProvider;

   public ManipulationStateMachine(final DoubleYoVariable simulationTime, final RobotSide robotSide, final FullRobotModel fullRobotModel,
                                   final TwistCalculator twistCalculator, InverseDynamicsCalculator inverseDynamicsCalculator,
                                   WalkingControllerParameters walkingControllerParameters, final DesiredHandPoseProvider handPoseProvider,
                                   final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, HandControllerInterface handController,
                                   double gravity, final double controlDT, MomentumBasedController momentumBasedController, GeometricJacobian jacobian,
                                   Map<OneDoFJoint, Double> desiredPositions, final YoVariableRegistry parentRegistry)
   {
      RigidBody endEffector = jacobian.getEndEffector();

      String name = endEffector.getName() + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);
      stateMachine = new StateMachine<ManipulationState>(name, name + "SwitchTime", ManipulationState.class, simulationTime, registry);
      this.robotSide = robotSide;
      this.inverseDynamicsCalculator = inverseDynamicsCalculator;
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

//    IndividualHandControlState<ManipulationState> moveRelativeToChestState = createIndividualHandControlState(1.0,
//          ManipulationState.MOVE_HAND_TO_POSITION_IN_CHESTFRAME,
//          currentConfigurationProvider, desiredConfigurationProvider,
//          momentumBasedController, jacobian, dynamicGraphicObjectsListRegistry);


      JointSpaceManipulationControlState<ManipulationState> moveInJointSpaceState =
         new JointSpaceManipulationControlState<ManipulationState>(ManipulationState.JOINT_SPACE, simulationTime, robotSide, jacobian, momentumBasedController,
            desiredPositions, registry);

      IndividualHandControlState<ManipulationState> moveRelativeToWorldState = createIndividualHandControlState(1.0,
                                                                                  ManipulationState.MOVE_HAND_TO_POSITION_IN_WORLDFRAME,
                                                                                  currentConfigurationProvider, desiredConfigurationProvider,
                                                                                  momentumBasedController, jacobian, dynamicGraphicObjectsListRegistry);

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
//         new StateTransition<ManipulationStateMachine.ManipulationState>(ManipulationState.MOVE_HAND_TO_POSITION_IN_CHESTFRAME, toNextChestPosition,
//                             Arrays.asList(setCurrentPoseBasedOnPreviousDesired, setDesiredPoseBasedOnProvider));
      final StateTransition<ManipulationState> toRelativeToWorldPositionTransition =
         new StateTransition<ManipulationStateMachine.ManipulationState>(ManipulationState.MOVE_HAND_TO_POSITION_IN_WORLDFRAME, toNextWorldPosition,
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


      if (handController != null)
      {
         this.handController = handController;
         this.toolBody = new MassMatrixEstimatingToolRigidBody(name + "Tool", handController.getWristJoint(), fullRobotModel, gravity, controlDT, registry,
                 dynamicGraphicObjectsListRegistry);
      }
      else
      {
         this.handController = null;
         this.toolBody = null;
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

   public void startComputation()
   {
      estimateObjectWrench();

      stateMachine.checkTransitionConditionsThoroughly();
      stateMachine.doAction();
      IndividualManipulationState<ManipulationState> manipulationState = manipulationStateMap.get(stateMachine.getCurrentStateEnum());

      // set indiv momentum in controller

      for (DynamicGraphicReferenceFrame frame : dynamicGraphicReferenceFrames)
      {
         frame.update();
      }


      if (handController != null)
      {
         if (handController.isClosed())
         {
            Wrench wrench = new Wrench();
            toolBody.control(manipulationState.getDesiredHandAcceleration(), wrench);

//          inverseDynamicsCalculator.setExternalWrench(handController.getWristJoint().getSuccessor(), wrench);
         }
      }
   }

   private void estimateObjectWrench()
   {
      if (handController != null)
      {
         if (handController.isClosed())
         {
            handController.packWristWrench(measuredWristWrench);
            toolBody.update(measuredWristWrench);
         }
         else
         {
            toolBody.reset();
         }
      }
   }

   public void waitUntilComputationIsDone()
   {
      // TODO Auto-generated method stub

   }

   private void addState(IndividualManipulationState<ManipulationState> state)
   {
      stateMachine.addState(state);
      manipulationStateMap.put(state.getStateEnum(), state);
   }

   private IndividualHandControlState<ManipulationState> createIndividualHandControlState(double trajectoryTime, ManipulationState manipulationState,
           SE3ConfigurationProvider initialConfigurationProvider, SE3ConfigurationProvider finalConfigurationProvider,
           MomentumBasedController momentumBasedController, GeometricJacobian jacobian, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      ConstantDoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(trajectoryTime);

      ReferenceFrame referenceFrame = jacobian.getBase().getBodyFixedFrame();

      String namePrefix = FormattingTools.underscoredToCamelCase(manipulationState.toString(), true);
      StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame,
                                                                               trajectoryTime, initialConfigurationProvider, finalConfigurationProvider,
                                                                               registry);

      OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame,
                                                                                      trajectoryTimeProvider, initialConfigurationProvider,
                                                                                      finalConfigurationProvider, registry);

      final IndividualHandControlState<ManipulationState> ret = new IndividualHandControlState<ManipulationState>(manipulationState, robotSide,
                                                                   positionTrajectoryGenerator, orientationTrajectoryGenerator,
                                                                   handSpatialAccelerationControlModule, momentumBasedController, jacobian,
                                                                   dynamicGraphicObjectsListRegistry, registry);

      return ret;
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
