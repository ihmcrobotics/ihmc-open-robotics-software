package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.IndividualHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states.IndividualManipulationState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.sensors.MassMatrixEstimatingToolRigidBody;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SE3ConfigurationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

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
   private enum ManipulationState
   {
      MOVE_HAND_TO_POSITION_IN_CHESTFRAME, MOVE_HAND_TO_POSITION_IN_WORLDFRAME
   };

   private final YoVariableRegistry registry;

   private final RobotSide robotSide;
   
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final Collection<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();

   private final TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
   private final StateMachine<ManipulationState> stateMachine;
   private final EnumMap<ManipulationState, IndividualManipulationState<ManipulationState>> manipulationStateMap = new EnumMap<ManipulationState, IndividualManipulationState<ManipulationState>>(
         ManipulationState.class);

   private final RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule;

   private final HandControllerInterface handController;

   private final MassMatrixEstimatingToolRigidBody toolBody;
   private final Wrench measuredWristWrench = new Wrench();
     


   public ManipulationStateMachine(final DoubleYoVariable simulationTime, final RobotSide robotSide, final FullRobotModel fullRobotModel,
         final TwistCalculator twistCalculator, InverseDynamicsCalculator inverseDynamicsCalculator, WalkingControllerParameters walkingControllerParameters, final DesiredHandPoseProvider handPoseProvider,
         final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, HandControllerInterface handController, double gravity, final double controlDT,
         final YoVariableRegistry parentRegistry)
   {
      String name = robotSide.getCamelCaseNameForStartOfExpression() + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);
      stateMachine = new StateMachine<ManipulationState>(name, name + "SwitchTime", ManipulationState.class, simulationTime, registry);
      this.robotSide = robotSide;
      this.inverseDynamicsCalculator = inverseDynamicsCalculator;

      String frameName = robotSide.getCamelCaseNameForStartOfExpression() + "HandPositionControlFrame";
      final ReferenceFrame frameAfterJoint = fullRobotModel.getHand(robotSide).getParentJoint().getFrameAfterJoint();
      ReferenceFrame endEffectorFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, frameAfterJoint,
            walkingControllerParameters.getHandControlFramesWithRespectToFrameAfterWrist().get(robotSide));

      RigidBody hand = fullRobotModel.getHand(robotSide);
      handSpatialAccelerationControlModule = new RigidBodySpatialAccelerationControlModule(hand.getName(), twistCalculator, hand, endEffectorFrame, registry);

      handSpatialAccelerationControlModule.setPositionProportionalGains(100.0, 100.0, 100.0);
      handSpatialAccelerationControlModule.setPositionDerivativeGains(20.0, 20.0, 20.0);
      handSpatialAccelerationControlModule.setOrientationProportionalGains(100.0, 100.0, 100.0);
      handSpatialAccelerationControlModule.setOrientationDerivativeGains(20.0, 20.0, 20.0);

      final ChangeableConfigurationProvider currentConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(endEffectorFrame));
      final ChangeableConfigurationProvider desiredConfigurationProvider = new ChangeableConfigurationProvider(handPoseProvider.getDesiredHandPose(robotSide));

      IndividualHandControlState<ManipulationState> moveRelativeToChestState = createIndividualHandControlState(1.0,
            ManipulationState.MOVE_HAND_TO_POSITION_IN_CHESTFRAME, fullRobotModel.getChest().getBodyFixedFrame(), fullRobotModel.getChest(),
            currentConfigurationProvider, desiredConfigurationProvider, dynamicGraphicObjectsListRegistry);

      IndividualHandControlState<ManipulationState> moveRelativeToWorldState = createIndividualHandControlState(1.0,
            ManipulationState.MOVE_HAND_TO_POSITION_IN_WORLDFRAME, ReferenceFrame.getWorldFrame(), fullRobotModel.getChest(), currentConfigurationProvider,
            desiredConfigurationProvider, dynamicGraphicObjectsListRegistry);

      StateTransitionCondition toNextChestPosition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return handPoseProvider.checkForNewPose(robotSide) && !handPoseProvider.isRelativeToWorld();
         }
      };

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

      final StateTransition<ManipulationState> toRelativeToChestPositionTransition = new StateTransition<ManipulationStateMachine.ManipulationState>(
            ManipulationState.MOVE_HAND_TO_POSITION_IN_CHESTFRAME, toNextChestPosition, Arrays.asList(setCurrentPoseBasedOnPreviousDesired,
                  setDesiredPoseBasedOnProvider));
      final StateTransition<ManipulationState> toRelativeToWorldPositionTransition = new StateTransition<ManipulationStateMachine.ManipulationState>(
            ManipulationState.MOVE_HAND_TO_POSITION_IN_WORLDFRAME, toNextWorldPosition, Arrays.asList(setCurrentPoseBasedOnPreviousDesired,
                  setDesiredPoseBasedOnProvider));

      moveRelativeToChestState.addStateTransition(toRelativeToChestPositionTransition);
      moveRelativeToChestState.addStateTransition(toRelativeToWorldPositionTransition);

      moveRelativeToWorldState.addStateTransition(toRelativeToChestPositionTransition);
      moveRelativeToWorldState.addStateTransition(toRelativeToWorldPositionTransition);

      addState(moveRelativeToChestState);
      addState(moveRelativeToWorldState);

      if (handPoseProvider.isRelativeToWorld())
      {
         stateMachine.setCurrentState(ManipulationState.MOVE_HAND_TO_POSITION_IN_WORLDFRAME);
      }
      else
      {
         stateMachine.setCurrentState(ManipulationState.MOVE_HAND_TO_POSITION_IN_CHESTFRAME);
      }

      ControlFlowOutputPort<TaskspaceConstraintData> desiredAccelerationOutputPort = createOutputPort("desiredAccelerationOutputPort");
      desiredAccelerationOutputPort.setData(taskspaceConstraintData);

      if (dynamicGraphicObjectsListRegistry != null)
      {
         DynamicGraphicObjectsList list = new DynamicGraphicObjectsList(name);

         DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame = new DynamicGraphicReferenceFrame(endEffectorFrame, registry, 0.3);
         dynamicGraphicReferenceFrames.add(dynamicGraphicReferenceFrame);
         list.add(dynamicGraphicReferenceFrame);

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(list);
         list.hideDynamicGraphicObjects();
      }

      
      if(handController != null)
      {
         this.handController = handController;
         this.toolBody = new MassMatrixEstimatingToolRigidBody(name + "Tool", handController.getWristJoint(), fullRobotModel, gravity, controlDT, registry, dynamicGraphicObjectsListRegistry);
      }
      else
      {
         this.handController = null;
         this.toolBody = null;
      }

      parentRegistry.addChild(registry);
   }

   public void startComputation()
   {
      estimateObjectWrench();
      
      stateMachine.checkTransitionConditionsThoroughly();
      stateMachine.doAction();
      IndividualManipulationState<ManipulationState> manipulationState = manipulationStateMap.get(stateMachine.getCurrentStateEnum());

      taskspaceConstraintData.set(manipulationState.getDesiredHandAcceleration());

      for (DynamicGraphicReferenceFrame frame : dynamicGraphicReferenceFrames)
      {
         frame.update();
      }
      

      if (handController != null)
      {
         if(handController.isClosed())
         {
            Wrench wrench = new Wrench();
            toolBody.control(manipulationState.getDesiredHandAcceleration(), wrench);
//            inverseDynamicsCalculator.setExternalWrench(handController.getWristJoint().getSuccessor(), wrench);
         }
         handController.doControl();
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
         ReferenceFrame referenceFrame, RigidBody base, SE3ConfigurationProvider initialConfigurationProvider,
         SE3ConfigurationProvider finalConfigurationProvider, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      ConstantDoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(trajectoryTime);

      String namePrefix = FormattingTools.underscoredToCamelCase(manipulationState.toString(), true);
      StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame,
            trajectoryTime, initialConfigurationProvider, finalConfigurationProvider, registry);

      OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame,
            trajectoryTimeProvider, initialConfigurationProvider, finalConfigurationProvider, registry);

      final IndividualHandControlState<ManipulationState> ret = new IndividualHandControlState<ManipulationState>(manipulationState, robotSide, base,
            positionTrajectoryGenerator, orientationTrajectoryGenerator, handSpatialAccelerationControlModule, dynamicGraphicObjectsListRegistry, registry);

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

   public TaskspaceConstraintData getTaskspaceConstraintData()
   {
      return taskspaceConstraintData;
   }

   public void doAdditionalTorqueControl()
   {
      if (handController != null)
      {
         handController.doTorqueControl();
      }
   }
}
