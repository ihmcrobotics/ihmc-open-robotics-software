package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual;

import static com.yobotics.simulationconstructionset.util.statemachines.StateMachineTools.addRequestedStateTransition;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.SE3PDGains;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.AbstractJointSpaceHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.InverseKinematicsTaskspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.JointSpaceHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.LoadBearingPlaneHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.LowLevelInverseKinematicsTaskspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.LowLevelJointSpaceHandControlControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.ObjectManipulationState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TaskspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.individual.states.PointPositionHandControlState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantPoseTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.OneDoFJointQuinticTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.OrientationTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.StraightLinePoseTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.provider.YoVariableDoubleProvider;

public class HandControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final StateMachine<HandControlState> stateMachine;
   private final Map<ReferenceFrame, RigidBodySpatialAccelerationControlModule> handSpatialAccelerationControlModules;

   private final Map<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator> quinticPolynomialTrajectoryGenerators;

   private final ConstantPoseTrajectoryGenerator holdPoseTrajectoryGenerator;
   private final StraightLinePoseTrajectoryGenerator straightLinePoseTrajectoryGenerator;
   private final YoVariableDoubleProvider trajectoryTimeProvider;

   private final Map<OneDoFJoint, Double> jointCurrentPositionMap;

   private final TaskspaceHandPositionControlState taskSpacePositionControlState;
   private final AbstractJointSpaceHandControlState jointSpaceHandControlState;
   private final ObjectManipulationState objectManipulationState;
   private final LoadBearingPlaneHandControlState loadBearingControlState;
   private final List<TaskspaceHandPositionControlState> taskSpacePositionControlStates = new ArrayList<TaskspaceHandPositionControlState>();
   private final PointPositionHandControlState pointPositionControlState;

   private final EnumYoVariable<HandControlState> requestedState;
   private final OneDoFJoint[] oneDoFJoints;
   private final String name;
   private final RobotSide robotSide;
   private final TwistCalculator twistCalculator;
   private final SE3PDGains taskspaceControlGains;
   private final RigidBody chest, hand;

   private final double controlDT;

   private final DoubleYoVariable maxAccelerationArmTaskspace, maxJerkArmTaskspace;

   public HandControlModule(RobotSide robotSide, SE3PDGains taskspaceControlGains, MomentumBasedController momentumBasedController,
         ArmControllerParameters armControlParameters, ControlStatusProducer controlStatusProducer, YoVariableRegistry parentRegistry)
   {
      this.controlDT = momentumBasedController.getControlDT();

      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      hand = fullRobotModel.getHand(robotSide);
      chest = fullRobotModel.getChest();
      ReferenceFrame handFrame = hand.getBodyFixedFrame();
      int jacobianId = momentumBasedController.getOrCreateGeometricJacobian(chest, hand, handFrame);

      this.robotSide = robotSide;
      String namePrefix = hand.getName();
      name = namePrefix + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);
      this.twistCalculator = momentumBasedController.getTwistCalculator();

      this.taskspaceControlGains = taskspaceControlGains;

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

      handSpatialAccelerationControlModules = new LinkedHashMap<ReferenceFrame, RigidBodySpatialAccelerationControlModule>();

      straightLinePoseTrajectoryGenerator = new StraightLinePoseTrajectoryGenerator(name, true, worldFrame, registry);
      holdPoseTrajectoryGenerator = new ConstantPoseTrajectoryGenerator(name + "Hold", true, worldFrame, parentRegistry);

      loadBearingControlState = new LoadBearingPlaneHandControlState(namePrefix, HandControlState.LOAD_BEARING, robotSide, momentumBasedController,
            fullRobotModel.getElevator(), hand, jacobianId, registry);

      if (armControlParameters.doLowLevelPositionControl())
      {
         jointSpaceHandControlState = new LowLevelJointSpaceHandControlControlState(namePrefix, HandControlState.JOINT_SPACE, robotSide,
               oneDoFJoints, momentumBasedController, armControlParameters, controlDT, registry);
      }
      else
      {
         jointSpaceHandControlState = new JointSpaceHandControlState(namePrefix, HandControlState.JOINT_SPACE, robotSide,
               oneDoFJoints, momentumBasedController, armControlParameters, controlDT, registry);
      }

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      objectManipulationState = new ObjectManipulationState(namePrefix, HandControlState.OBJECT_MANIPULATION, robotSide, momentumBasedController,
            jacobianId, null, chest, hand, dynamicGraphicObjectsListRegistry, parentRegistry);

      if (armControlParameters.useInverseKinematicsTaskspaceControl())
      {
         if (armControlParameters.doLowLevelPositionControl())
         {
            taskSpacePositionControlState = new LowLevelInverseKinematicsTaskspaceHandPositionControlState(namePrefix,
                  HandControlState.TASK_SPACE_POSITION, robotSide, momentumBasedController, jacobianId, chest, hand,
                  dynamicGraphicObjectsListRegistry, armControlParameters, controlStatusProducer, controlDT, registry);
         }
         else
         {
            taskSpacePositionControlState = new InverseKinematicsTaskspaceHandPositionControlState(namePrefix, HandControlState.TASK_SPACE_POSITION,
                  robotSide, momentumBasedController, jacobianId, chest, hand, dynamicGraphicObjectsListRegistry, armControlParameters,
                  controlStatusProducer, controlDT, registry);
         }
      }
      else
      {
         taskSpacePositionControlState = new TaskspaceHandPositionControlState(namePrefix, HandControlState.TASK_SPACE_POSITION, robotSide,
               momentumBasedController, jacobianId, chest, hand, dynamicGraphicObjectsListRegistry, registry);
      }
      pointPositionControlState = new PointPositionHandControlState(momentumBasedController, robotSide, dynamicGraphicObjectsListRegistry, registry);

      setupStateMachine(simulationTime);

      taskSpacePositionControlStates.add(taskSpacePositionControlState);
      taskSpacePositionControlStates.add(objectManipulationState);

      maxAccelerationArmTaskspace = new DoubleYoVariable("maxAccelerationArmTaskspace", registry);
      maxAccelerationArmTaskspace.set(armControlParameters.getArmTaskspaceMaxAcceleration());
      maxJerkArmTaskspace = new DoubleYoVariable("maxJerkArmTaskspace", registry);
      maxJerkArmTaskspace.set(armControlParameters.getArmTaskspaceMaxJerk());

      //Pre-create the RigidBodySpatialAccelerationControlModules
      getOrCreateRigidBodySpatialAccelerationControlModule(worldFrame);
      getOrCreateRigidBodySpatialAccelerationControlModule(fullRobotModel.getChest().getBodyFixedFrame());
      getOrCreateRigidBodySpatialAccelerationControlModule(fullRobotModel.getHandControlFrame(robotSide));

      jointCurrentPositionMap = new LinkedHashMap<OneDoFJoint, Double>();
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         jointCurrentPositionMap.put(oneDoFJoint, oneDoFJoint.getQ());
      }

      parentRegistry.addChild(registry);
   }

   @SuppressWarnings("unchecked")
   private void setupStateMachine(DoubleYoVariable simulationTime)
   {
      addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, taskSpacePositionControlState);
      addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, objectManipulationState);
      addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, jointSpaceHandControlState);
      addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, pointPositionControlState);

      addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, objectManipulationState);
      addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, jointSpaceHandControlState);
      addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, taskSpacePositionControlState);
      addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, pointPositionControlState);

      addRequestedStateTransition(requestedState, false, objectManipulationState, jointSpaceHandControlState);
      addRequestedStateTransition(requestedState, false, objectManipulationState, taskSpacePositionControlState);
      addRequestedStateTransition(requestedState, false, objectManipulationState, objectManipulationState);
      addRequestedStateTransition(requestedState, false, objectManipulationState, pointPositionControlState);

      addRequestedStateTransition(requestedState, false, pointPositionControlState, jointSpaceHandControlState);
      addRequestedStateTransition(requestedState, false, pointPositionControlState, taskSpacePositionControlState);
      addRequestedStateTransition(requestedState, false, pointPositionControlState, objectManipulationState);
      addRequestedStateTransition(requestedState, false, pointPositionControlState, pointPositionControlState);

      addRequestedStateTransition(requestedState, false, taskSpacePositionControlState, loadBearingControlState);
      addRequestedStateTransition(requestedState, true, loadBearingControlState, taskSpacePositionControlState);
      addRequestedStateTransition(requestedState, false, jointSpaceHandControlState, loadBearingControlState);
      addRequestedStateTransition(requestedState, false, loadBearingControlState, jointSpaceHandControlState);

      stateMachine.addState(jointSpaceHandControlState);
      stateMachine.addState(taskSpacePositionControlState);
      stateMachine.addState(objectManipulationState);
      stateMachine.addState(loadBearingControlState);
      stateMachine.addState(pointPositionControlState);
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

   public void executeTaskSpaceTrajectory(PositionTrajectoryGenerator positionTrajectory, OrientationTrajectoryGenerator orientationTrajectory,
         ReferenceFrame frameToControlPoseOf, RigidBody base, SE3PDGains gains)
   {
      RigidBodySpatialAccelerationControlModule rigidBodySpatialAccelerationControlModule = getOrCreateRigidBodySpatialAccelerationControlModule(frameToControlPoseOf);
      rigidBodySpatialAccelerationControlModule.setGains(gains);
      taskSpacePositionControlState.setTrajectory(positionTrajectory, orientationTrajectory, base, rigidBodySpatialAccelerationControlModule, frameToControlPoseOf);
      requestedState.set(taskSpacePositionControlState.getStateEnum());
      stateMachine.checkTransitionConditions();
   }

   public void moveInStraightLine(FramePose finalDesiredPose, double time, RigidBody base, ReferenceFrame frameToControlPoseOf, ReferenceFrame trajectoryFrame,
         SE3PDGains gains)
   {
      FramePose pose = computeDesiredFramePose(frameToControlPoseOf, trajectoryFrame);

      straightLinePoseTrajectoryGenerator.registerAndSwitchFrame(trajectoryFrame);
      straightLinePoseTrajectoryGenerator.setInitialPose(pose);
      straightLinePoseTrajectoryGenerator.setFinalPose(finalDesiredPose);
      straightLinePoseTrajectoryGenerator.setTrajectoryTime(time);
      
      executeTaskSpaceTrajectory(straightLinePoseTrajectoryGenerator, straightLinePoseTrajectoryGenerator, frameToControlPoseOf, base, gains);
   }

   private FramePose computeDesiredFramePose(ReferenceFrame frameToControlPoseOf, ReferenceFrame trajectoryFrame)
   {
      FramePose pose;
      if (stateMachine.getCurrentState() instanceof TaskspaceHandPositionControlState)
      {
         // start at current desired
         pose = getCurrentDesiredPose((TaskspaceHandPositionControlState) stateMachine.getCurrentState(), frameToControlPoseOf, trajectoryFrame);
      }
      else if (stateMachine.getCurrentState() instanceof PointPositionHandControlState)
      {
         pose = getCurrentDesiredPose((PointPositionHandControlState) stateMachine.getCurrentState(), frameToControlPoseOf, trajectoryFrame);
      }
      else
      {
         // FIXME: make this be based on desired joint angles
         pose = new FramePose(frameToControlPoseOf);
         pose.changeFrame(trajectoryFrame);
      }

      return pose;
   }

   private FramePose getCurrentDesiredPose(TaskspaceHandPositionControlState taskspaceHandPositionControlState, ReferenceFrame frameToControlPoseOf,
         ReferenceFrame trajectoryFrame)
   {
      FramePose pose = taskspaceHandPositionControlState.getDesiredPose();
      pose.changeFrame(trajectoryFrame);

      Transform3D oldTrackingFrameTransform = new Transform3D();
      pose.getPose(oldTrackingFrameTransform);
      Transform3D transformFromNewTrackingFrameToOldTrackingFrame = frameToControlPoseOf.getTransformToDesiredFrame(taskspaceHandPositionControlState
            .getFrameToControlPoseOf());

      Transform3D newTrackingFrameTransform = new Transform3D();
      newTrackingFrameTransform.mul(oldTrackingFrameTransform, transformFromNewTrackingFrameToOldTrackingFrame);
      pose.setPoseIncludingFrame(trajectoryFrame, newTrackingFrameTransform);

      return pose;
   }

   private FramePose getCurrentDesiredPose(PointPositionHandControlState pointPositionHandControlState, ReferenceFrame frameToControlPoseOf,
         ReferenceFrame trajectoryFrame)
   {
      // desired position, actual orientation
      FramePoint position = pointPositionHandControlState.getDesiredPosition();
      position.changeFrame(trajectoryFrame);

      FramePose pose = new FramePose(frameToControlPoseOf);
      pose.changeFrame(trajectoryFrame);
      pose.setPosition(position);

      Transform3D oldTrackingFrameTransform = new Transform3D();
      pose.getPose(oldTrackingFrameTransform);
      Transform3D transformFromNewTrackingFrameToOldTrackingFrame = frameToControlPoseOf.getTransformToDesiredFrame(pointPositionHandControlState
            .getFrameToControlPoseOf());

      Transform3D newTrackingFrameTransform = new Transform3D();
      newTrackingFrameTransform.mul(oldTrackingFrameTransform, transformFromNewTrackingFrameToOldTrackingFrame);
      pose.setPoseIncludingFrame(trajectoryFrame, newTrackingFrameTransform);

      return pose;
   }

   public void requestLoadBearing()
   {
      requestedState.set(loadBearingControlState.getStateEnum());
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
      stateMachine.checkTransitionConditions();
   }

   public boolean isLoadBearing()
   {
      return stateMachine.getCurrentStateEnum() == HandControlState.LOAD_BEARING;
   }

   public void holdPositionInBase()
   {
      holdPositionInFrame(chest.getBodyFixedFrame(), chest, taskspaceControlGains);
   }

   public void holdPositionInFrame(ReferenceFrame frame, RigidBody base, SE3PDGains gains)
   {
      ReferenceFrame handFrame = hand.getBodyFixedFrame();
      FramePose pose = new FramePose(handFrame);
      pose.changeFrame(frame);
      
      holdPoseTrajectoryGenerator.registerAndSwitchFrame(frame);
      holdPoseTrajectoryGenerator.setConstantPose(pose);

      executeTaskSpaceTrajectory(holdPoseTrajectoryGenerator, holdPoseTrajectoryGenerator, handFrame, base, gains);
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

   private RigidBodySpatialAccelerationControlModule getOrCreateRigidBodySpatialAccelerationControlModule(ReferenceFrame handPositionControlFrame)
   {
      RigidBodySpatialAccelerationControlModule ret = handSpatialAccelerationControlModules.get(handPositionControlFrame);
      if (ret == null)
      {
         ret = new RigidBodySpatialAccelerationControlModule(name + handPositionControlFrame.getName(), twistCalculator, hand, handPositionControlFrame,
               controlDT, registry);

         handSpatialAccelerationControlModules.put(handPositionControlFrame, ret);
      }

      ret.setOrientationMaxAccelerationAndJerk(maxAccelerationArmTaskspace.getDoubleValue(), maxJerkArmTaskspace.getDoubleValue());
      ret.setPositionMaxAccelerationAndJerk(maxAccelerationArmTaskspace.getDoubleValue(), maxJerkArmTaskspace.getDoubleValue());

      return ret;
   }

   public boolean isControllingPoseInWorld()
   {
      State<HandControlState> currentState = stateMachine.getCurrentState();

      for (TaskspaceHandPositionControlState taskSpacePositionControlState : taskSpacePositionControlStates)
      {
         if (currentState == taskSpacePositionControlState)
            return taskSpacePositionControlState.getReferenceFrame() == worldFrame;
      }

      return false;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }
}
