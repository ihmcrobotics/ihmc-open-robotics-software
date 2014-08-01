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
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.PointPositionHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TaskspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.ChangeableConfigurationProvider;
import us.ihmc.utilities.math.trajectories.providers.SE3ConfigurationProvider;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
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
import com.yobotics.simulationconstructionset.util.trajectory.ConstantOrientationTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantPositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.OneDoFJointQuinticTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.OrientationInterpolationTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.OrientationTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.StraightLinePositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.provider.YoSE3ConfigurationProvider;
import com.yobotics.simulationconstructionset.util.trajectory.provider.YoVariableDoubleProvider;

public class IndividualHandControlModule
{
   private final YoVariableRegistry registry;

   private final StateMachine<IndividualHandControlState> stateMachine;
   private final Map<ReferenceFrame, RigidBodySpatialAccelerationControlModule> handSpatialAccelerationControlModules;

   private final ChangeableConfigurationProvider initialConfigurationProvider;
   private final ChangeableConfigurationProvider finalConfigurationProvider;

   private final Map<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator> quinticPolynomialTrajectoryGenerators;

   private final Map<ReferenceFrame, StraightLinePositionTrajectoryGenerator> straightLinePositionWorldTrajectoryGenerators;
   private final Map<ReferenceFrame, OrientationInterpolationTrajectoryGenerator> orientationInterpolationWorldTrajectoryGenerators;
   private final YoVariableDoubleProvider trajectoryTimeProvider;

   private final Map<ReferenceFrame, ConstantPositionTrajectoryGenerator> holdPositionTrajectoryGenerators;
   private final Map<ReferenceFrame, ConstantOrientationTrajectoryGenerator> holdOrientationTrajectoryGenerators;

   private final Map<OneDoFJoint, Double> jointCurrentPositionMap;

   private final TaskspaceHandPositionControlState taskSpacePositionControlState;
   private final AbstractJointSpaceHandControlState jointSpaceHandControlState;
   private final ObjectManipulationState objectManipulationState;
   private final LoadBearingPlaneHandControlState loadBearingControlState;
   private final List<TaskspaceHandPositionControlState> taskSpacePositionControlStates = new ArrayList<TaskspaceHandPositionControlState>();
   private final PointPositionHandControlState pointPositionControlState;

   private final EnumYoVariable<IndividualHandControlState> requestedState;
   private final OneDoFJoint[] oneDoFJoints;
   private final String name;
   private final RobotSide robotSide;
   private final TwistCalculator twistCalculator;
   private final SE3PDGains taskspaceControlGains;
   private final Map<ReferenceFrame, YoSE3ConfigurationProvider> currentDesiredConfigurationProviders = new LinkedHashMap<ReferenceFrame, YoSE3ConfigurationProvider>();
   private final RigidBody chest, hand;

   private final double controlDT;

   private final DoubleYoVariable maxAccelerationArmTaskspace, maxJerkArmTaskspace;

   public IndividualHandControlModule(RobotSide robotSide, SE3PDGains taskspaceControlGains, MomentumBasedController momentumBasedController,
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

      DoubleYoVariable simulationTime = momentumBasedController.getYoTime();
      stateMachine = new StateMachine<IndividualHandControlState>(name, name + "SwitchTime", IndividualHandControlState.class, simulationTime, registry);

      handSpatialAccelerationControlModules = new LinkedHashMap<ReferenceFrame, RigidBodySpatialAccelerationControlModule>();

      initialConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(handFrame));
      finalConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(handFrame)); // FIXME: make Yo, but is difficult because frame can change

      straightLinePositionWorldTrajectoryGenerators = new LinkedHashMap<ReferenceFrame, StraightLinePositionTrajectoryGenerator>();
      orientationInterpolationWorldTrajectoryGenerators = new LinkedHashMap<ReferenceFrame, OrientationInterpolationTrajectoryGenerator>();

      holdPositionTrajectoryGenerators = new LinkedHashMap<ReferenceFrame, ConstantPositionTrajectoryGenerator>();
      holdOrientationTrajectoryGenerators = new LinkedHashMap<ReferenceFrame, ConstantOrientationTrajectoryGenerator>();

      InverseDynamicsJoint[] controlledJointsInJointSpaceState = ScrewTools.createJointPath(chest, hand);

      loadBearingControlState = new LoadBearingPlaneHandControlState(namePrefix, IndividualHandControlState.LOAD_BEARING, robotSide, momentumBasedController,
            fullRobotModel.getElevator(), hand, jacobianId, registry);

      if (armControlParameters.doLowLevelPositionControl())
      {
         jointSpaceHandControlState = new LowLevelJointSpaceHandControlControlState(namePrefix, IndividualHandControlState.JOINT_SPACE, robotSide,
               controlledJointsInJointSpaceState, momentumBasedController, armControlParameters, controlDT, registry);
      }
      else
      {
         jointSpaceHandControlState = new JointSpaceHandControlState(namePrefix, IndividualHandControlState.JOINT_SPACE, robotSide,
               controlledJointsInJointSpaceState, momentumBasedController, armControlParameters, controlDT, registry);
      }

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      objectManipulationState = new ObjectManipulationState(namePrefix, IndividualHandControlState.OBJECT_MANIPULATION, robotSide, momentumBasedController,
            jacobianId, null, chest, hand, dynamicGraphicObjectsListRegistry, parentRegistry);

      if (armControlParameters.useInverseKinematicsTaskspaceControl())
      {
         if (armControlParameters.doLowLevelPositionControl())
         {
            taskSpacePositionControlState = new LowLevelInverseKinematicsTaskspaceHandPositionControlState(namePrefix,
                  IndividualHandControlState.TASK_SPACE_POSITION, robotSide, momentumBasedController, jacobianId, chest, hand,
                  dynamicGraphicObjectsListRegistry, armControlParameters, controlStatusProducer, controlDT, registry);
         }
         else
         {
            taskSpacePositionControlState = new InverseKinematicsTaskspaceHandPositionControlState(namePrefix, IndividualHandControlState.TASK_SPACE_POSITION,
                  robotSide, momentumBasedController, jacobianId, chest, hand, dynamicGraphicObjectsListRegistry, armControlParameters,
                  controlStatusProducer, controlDT, registry);
         }
      }
      else
      {
         taskSpacePositionControlState = new TaskspaceHandPositionControlState(namePrefix, IndividualHandControlState.TASK_SPACE_POSITION, robotSide,
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
      getOrCreateRigidBodySpatialAccelerationControlModule(ReferenceFrame.getWorldFrame());
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
      updateCurrentDesiredConfiguration();
   }

   private void updateCurrentDesiredConfiguration()
   {
      for (ReferenceFrame frameToControlPositionOf : currentDesiredConfigurationProviders.keySet())
      {
         FramePose pose = computeDesiredFramePose(frameToControlPositionOf, ReferenceFrame.getWorldFrame());
         currentDesiredConfigurationProviders.get(frameToControlPositionOf).setPose(pose);
      }
   }

   public boolean isDone()
   {
      return stateMachine.getCurrentState().isDone();
   }

   public void executeTaskSpaceTrajectory(PositionTrajectoryGenerator positionTrajectory, OrientationTrajectoryGenerator orientationTrajectory,
         ReferenceFrame frameToControlPoseOf, RigidBody base, boolean estimateMassProperties, SE3PDGains gains)
   {
      TaskspaceHandPositionControlState state = estimateMassProperties ? objectManipulationState : taskSpacePositionControlState;
      RigidBodySpatialAccelerationControlModule rigidBodySpatialAccelerationControlModule = getOrCreateRigidBodySpatialAccelerationControlModule(frameToControlPoseOf);
      rigidBodySpatialAccelerationControlModule.setGains(gains);
      state.setTrajectory(positionTrajectory, orientationTrajectory, base, rigidBodySpatialAccelerationControlModule, frameToControlPoseOf);
      requestedState.set(state.getStateEnum());
      stateMachine.checkTransitionConditions();
   }

   public void moveInStraightLine(FramePose finalDesiredPose, double time, RigidBody base, ReferenceFrame frameToControlPoseOf, ReferenceFrame trajectoryFrame,
         boolean holdObject, SE3PDGains gains)
   {
      FramePose pose = computeDesiredFramePose(frameToControlPoseOf, trajectoryFrame);

      initialConfigurationProvider.set(pose);
      finalConfigurationProvider.set(finalDesiredPose);
      trajectoryTimeProvider.set(time);
      executeTaskSpaceTrajectory(getOrCreateStraightLinePositionTrajectoryGenerator(trajectoryFrame),
            getOrCreateOrientationInterpolationTrajectoryGenerator(trajectoryFrame), frameToControlPoseOf, base, holdObject, gains);
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

   public boolean isHoldingObject()
   {
      return stateMachine.getCurrentStateEnum() == IndividualHandControlState.OBJECT_MANIPULATION;
   }

   public boolean isLoadBearing()
   {
      return stateMachine.getCurrentStateEnum() == IndividualHandControlState.LOAD_BEARING;
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
      initialConfigurationProvider.set(pose);
      PositionTrajectoryGenerator positionTrajectory = getOrCreateConstantPositionTrajectoryGenerator(frame);
      OrientationTrajectoryGenerator orientationTrajectory = getOrCreateConstantOrientationTrajectoryGenerator(frame);

      executeTaskSpaceTrajectory(positionTrajectory, orientationTrajectory, handFrame, base, isHoldingObject(), gains);
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

   private StraightLinePositionTrajectoryGenerator getOrCreateStraightLinePositionTrajectoryGenerator(ReferenceFrame referenceFrame)
   {
      StraightLinePositionTrajectoryGenerator ret = straightLinePositionWorldTrajectoryGenerators.get(referenceFrame);
      if (ret == null)
      {
         ret = new StraightLinePositionTrajectoryGenerator(name + referenceFrame.getName(), referenceFrame, trajectoryTimeProvider,
               initialConfigurationProvider, finalConfigurationProvider, registry);
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
               initialConfigurationProvider, finalConfigurationProvider, registry);
         orientationInterpolationWorldTrajectoryGenerators.put(referenceFrame, ret);
      }

      return ret;
   }

   private ConstantPositionTrajectoryGenerator getOrCreateConstantPositionTrajectoryGenerator(ReferenceFrame referenceFrame)
   {
      ConstantPositionTrajectoryGenerator ret = holdPositionTrajectoryGenerators.get(referenceFrame);
      if (ret == null)
      {
         ret = new ConstantPositionTrajectoryGenerator(name + "Constant" + referenceFrame.getName(), referenceFrame, initialConfigurationProvider, 0.0,
               registry);
         holdPositionTrajectoryGenerators.put(referenceFrame, ret);
      }

      return ret;
   }

   private ConstantOrientationTrajectoryGenerator getOrCreateConstantOrientationTrajectoryGenerator(ReferenceFrame referenceFrame)
   {
      ConstantOrientationTrajectoryGenerator ret = holdOrientationTrajectoryGenerators.get(referenceFrame);
      if (ret == null)
      {
         ret = new ConstantOrientationTrajectoryGenerator(name + "Constant" + referenceFrame.getName(), chest.getBodyFixedFrame(), initialConfigurationProvider,
               0.0, registry);
         holdOrientationTrajectoryGenerators.put(referenceFrame, ret);
      }

      return ret;
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
      State<IndividualHandControlState> currentState = stateMachine.getCurrentState();

      for (TaskspaceHandPositionControlState taskSpacePositionControlState : taskSpacePositionControlStates)
      {
         if (currentState == taskSpacePositionControlState)
            return taskSpacePositionControlState.getReferenceFrame() == ReferenceFrame.getWorldFrame();
      }

      return false;
   }

   public SE3ConfigurationProvider getCurrentDesiredConfigurationProvider(ReferenceFrame frameToControlPositionOf)
   {
      YoSE3ConfigurationProvider ret = currentDesiredConfigurationProviders.get(frameToControlPositionOf);
      if (ret == null)
      {
         ret = new YoSE3ConfigurationProvider("currentDesired" + frameToControlPositionOf.getName() + "Configuration", ReferenceFrame.getWorldFrame(), registry);
         currentDesiredConfigurationProviders.put(frameToControlPositionOf, ret);
         updateCurrentDesiredConfiguration();
      }

      return ret;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }
}
