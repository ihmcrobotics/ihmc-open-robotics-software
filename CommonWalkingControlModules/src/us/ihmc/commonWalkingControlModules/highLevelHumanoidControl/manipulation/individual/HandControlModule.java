package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual;

import static com.yobotics.simulationconstructionset.util.statemachines.StateMachineTools.addRequestedStateTransition;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.AbstractJointSpaceHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.InverseKinematicsTaskspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.JointSpaceHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.LoadBearingPlaneHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.LowLevelInverseKinematicsTaskspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.LowLevelJointSpaceHandControlControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TaskspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.controller.YoPIDGains;
import com.yobotics.simulationconstructionset.util.controller.YoSE3PIDGains;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionAction;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantPoseTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.FinalApproachPoseTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.OneDoFJointQuinticTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.PoseTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.StraightLinePoseTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.provider.YoVariableDoubleProvider;

public class HandControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final StateMachine<HandControlState> stateMachine;
   private final RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule;

   private final Map<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator> quinticPolynomialTrajectoryGenerators;

   private final ConstantPoseTrajectoryGenerator holdPoseTrajectoryGenerator;
   private final StraightLinePoseTrajectoryGenerator straightLinePoseTrajectoryGenerator;
   private final FinalApproachPoseTrajectoryGenerator finalApproachPoseTrajectoryGenerator;

   private final BooleanYoVariable isExecutingHandStep;

   private final YoVariableDoubleProvider trajectoryTimeProvider;

   private final Map<OneDoFJoint, Double> jointCurrentPositionMap;

   private final TaskspaceHandPositionControlState taskSpacePositionControlState;
   private final AbstractJointSpaceHandControlState jointSpaceHandControlState;
   private final LoadBearingPlaneHandControlState loadBearingControlState;

   private final EnumYoVariable<HandControlState> requestedState;
   private final OneDoFJoint[] oneDoFJoints;
   private final String name;
   private final RobotSide robotSide;
   private final TwistCalculator twistCalculator;
   private final RigidBody chest, hand;

   private final FullRobotModel fullRobotModel;

   private final double controlDT;

   public HandControlModule(RobotSide robotSide, MomentumBasedController momentumBasedController, ArmControllerParameters armControlParameters,
         YoPIDGains jointspaceGains, YoSE3PIDGains taskspaceGains, ControlStatusProducer controlStatusProducer, YoVariableRegistry parentRegistry)
   {
      this.controlDT = momentumBasedController.getControlDT();

      fullRobotModel = momentumBasedController.getFullRobotModel();
      hand = fullRobotModel.getHand(robotSide);
      chest = fullRobotModel.getChest();
      ReferenceFrame handFrame = hand.getBodyFixedFrame();
      int jacobianId = momentumBasedController.getOrCreateGeometricJacobian(chest, hand, handFrame);

      this.robotSide = robotSide;
      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      name = namePrefix + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);
      twistCalculator = momentumBasedController.getTwistCalculator();

      oneDoFJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(chest, hand), OneDoFJoint.class);

      requestedState = new EnumYoVariable<HandControlState>(name + "RequestedState", "", registry, HandControlState.class, true);
      requestedState.set(null);

      trajectoryTimeProvider = new YoVariableDoubleProvider(name + "TrajectoryTime", registry);

      quinticPolynomialTrajectoryGenerators = new LinkedHashMap<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator>();

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         OneDoFJointQuinticTrajectoryGenerator trajectoryGenerator = new OneDoFJointQuinticTrajectoryGenerator(oneDoFJoint.getName() + "Trajectory",
               oneDoFJoint, trajectoryTimeProvider, registry);
         quinticPolynomialTrajectoryGenerators.put(oneDoFJoint, trajectoryGenerator);
      }

      DoubleYoVariable simulationTime = momentumBasedController.getYoTime();
      stateMachine = new StateMachine<HandControlState>(name, name + "SwitchTime", HandControlState.class, simulationTime, registry);

      handSpatialAccelerationControlModule = new RigidBodySpatialAccelerationControlModule(name, twistCalculator, hand,
            fullRobotModel.getHandControlFrame(robotSide), controlDT, taskspaceGains, registry);

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      holdPoseTrajectoryGenerator = new ConstantPoseTrajectoryGenerator(name + "Hold", true, worldFrame, parentRegistry);
      straightLinePoseTrajectoryGenerator = new StraightLinePoseTrajectoryGenerator(name, true, worldFrame, registry);
      finalApproachPoseTrajectoryGenerator = new FinalApproachPoseTrajectoryGenerator(name, true, worldFrame, registry, dynamicGraphicObjectsListRegistry);

      loadBearingControlState = new LoadBearingPlaneHandControlState(namePrefix, HandControlState.LOAD_BEARING, robotSide, momentumBasedController,
            fullRobotModel.getElevator(), hand, jacobianId, registry);

      if (armControlParameters.doLowLevelPositionControl())
      {
         jointSpaceHandControlState = new LowLevelJointSpaceHandControlControlState(namePrefix, HandControlState.JOINT_SPACE, robotSide, oneDoFJoints,
               momentumBasedController, armControlParameters, controlDT, registry);
      }
      else
      {
         jointSpaceHandControlState = new JointSpaceHandControlState(namePrefix, HandControlState.JOINT_SPACE, robotSide, oneDoFJoints,
               momentumBasedController, armControlParameters, jointspaceGains, controlDT, registry);
      }


      if (armControlParameters.useInverseKinematicsTaskspaceControl())
      {
         if (armControlParameters.doLowLevelPositionControl())
         {
            taskSpacePositionControlState = new LowLevelInverseKinematicsTaskspaceHandPositionControlState(namePrefix, HandControlState.TASK_SPACE_POSITION,
                  robotSide, momentumBasedController, jacobianId, chest, hand, dynamicGraphicObjectsListRegistry, armControlParameters, controlStatusProducer,
                  controlDT, registry);
         }
         else
         {
            taskSpacePositionControlState = new InverseKinematicsTaskspaceHandPositionControlState(namePrefix, HandControlState.TASK_SPACE_POSITION, robotSide,
                  momentumBasedController, jacobianId, chest, hand, dynamicGraphicObjectsListRegistry, armControlParameters, controlStatusProducer, controlDT,
                  registry);
         }
      }
      else
      {
         taskSpacePositionControlState = new TaskspaceHandPositionControlState(namePrefix, HandControlState.TASK_SPACE_POSITION, momentumBasedController,
               jacobianId, chest, hand, dynamicGraphicObjectsListRegistry, registry);
      }

      setupStateMachine();

      jointCurrentPositionMap = new LinkedHashMap<OneDoFJoint, Double>();
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         jointCurrentPositionMap.put(oneDoFJoint, oneDoFJoint.getQ());
      }

      isExecutingHandStep = new BooleanYoVariable(namePrefix + "DoingHandstep", registry);

      parentRegistry.addChild(registry);
   }

   @SuppressWarnings("unchecked")
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
            finalApproachPoseTrajectoryGenerator.showVisualization(false);
         }
      };
      StateTransition<HandControlState> swingToSupportTransition = new StateTransition<HandControlState>(HandControlState.LOAD_BEARING, swingToSupportCondition, swingToSupportAction);
      fromState.addStateTransition(swingToSupportTransition);
   }

   public void doControl()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   public boolean isDone()
   {
      return stateMachine.getCurrentState().isDone();
   }

   public void executeTaskSpaceTrajectory(PoseTrajectoryGenerator poseTrajectory)
   {
      taskSpacePositionControlState.setTrajectory(poseTrajectory, poseTrajectory, handSpatialAccelerationControlModule);
      requestedState.set(taskSpacePositionControlState.getStateEnum());
      stateMachine.checkTransitionConditions();
   }

   public void moveInStraightLine(FramePose finalDesiredPose, double time, ReferenceFrame trajectoryFrame)
   {
      FramePose pose = computeDesiredFramePose(trajectoryFrame);

      straightLinePoseTrajectoryGenerator.registerAndSwitchFrame(trajectoryFrame);
      straightLinePoseTrajectoryGenerator.setInitialPose(pose);
      straightLinePoseTrajectoryGenerator.setFinalPose(finalDesiredPose);
      straightLinePoseTrajectoryGenerator.setTrajectoryTime(time);

      executeTaskSpaceTrajectory(straightLinePoseTrajectoryGenerator);
   }

   private final FrameVector finalDirection = new FrameVector();
   public void moveToSurface(FramePose finalDesiredPose, FrameVector surfaceNormal, double safetyDistance, double time, ReferenceFrame trajectoryFrame, boolean goToSupportWhenDone)
   {
      FramePose pose = computeDesiredFramePose(trajectoryFrame);

      finalApproachPoseTrajectoryGenerator.showVisualization(true);
      finalApproachPoseTrajectoryGenerator.registerAndSwitchFrame(trajectoryFrame);
      finalApproachPoseTrajectoryGenerator.setInitialPose(pose);
      finalApproachPoseTrajectoryGenerator.setFinalPose(finalDesiredPose);
      finalApproachPoseTrajectoryGenerator.setTrajectoryTime(time);
      finalDirection.setIncludingFrame(surfaceNormal);
      finalDirection.negate();
      finalApproachPoseTrajectoryGenerator.setFinalDirection(finalDirection, safetyDistance);

      isExecutingHandStep.set(goToSupportWhenDone);
      executeTaskSpaceTrajectory(finalApproachPoseTrajectoryGenerator);
   }

   private FramePose computeDesiredFramePose(ReferenceFrame trajectoryFrame)
   {
      FramePose pose;
      if (stateMachine.getCurrentState() instanceof TaskspaceHandPositionControlState)
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
         quinticPolynomialTrajectoryGenerators.get(oneDoFJoint).setFinalPosition(desiredJointPositions.get(oneDoFJoint));
      }

      jointSpaceHandControlState.setTrajectories(quinticPolynomialTrajectoryGenerators);
      requestedState.set(jointSpaceHandControlState.getStateEnum());
      stateMachine.checkTransitionConditions();
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

      double epsilon = 1e-2;
      moveUsingQuinticSplines(jointCurrentPositionMap, epsilon);
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
