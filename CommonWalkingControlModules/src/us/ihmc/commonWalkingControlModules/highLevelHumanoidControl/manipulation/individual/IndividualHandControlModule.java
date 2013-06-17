package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PositionController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.*;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.SE3PDGains;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.sensors.MassMatrixEstimatingToolRigidBody;
import us.ihmc.commonWalkingControlModules.trajectories.*;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import javax.media.j3d.Transform3D;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static com.yobotics.simulationconstructionset.util.statemachines.StateMachineTools.addRequestedStateTransition;

public class IndividualHandControlModule
{
   private final YoVariableRegistry registry;

   private final StateMachine<IndividualHandControlState> stateMachine;
   private final Map<ReferenceFrame, RigidBodySpatialAccelerationControlModule> handSpatialAccelerationControlModules;
   private final MassMatrixEstimatingToolRigidBody toolBody;

   private final ChangeableConfigurationProvider initialConfigurationProvider;
   private final ChangeableConfigurationProvider finalConfigurationProvider;

   private final Map<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator> quinticPolynomialTrajectoryGenerators;

   private final Map<ReferenceFrame, StraightLinePositionTrajectoryGenerator> straightLinePositionWorldTrajectoryGenerators;
   private final Map<ReferenceFrame, OrientationInterpolationTrajectoryGenerator> orientationInterpolationWorldTrajectoryGenerators;
   private final YoVariableDoubleProvider trajectoryTimeProvider;

   private final Map<ReferenceFrame, ConstantPositionTrajectoryGenerator> holdPositionTrajectoryGenerators;
   private final Map<ReferenceFrame, ConstantOrientationTrajectoryGenerator> holdOrientationTrajectoryGenerators;

   private final TaskspaceHandPositionControlState taskSpacePositionControlState;
   private final JointSpaceHandControlControlState jointSpaceHandControlState;
   private final LoadBearingCylindricalHandControlState loadBearingCylindricalState;
   private final ProgressiveUnloadingHandControlState progressiveUnloadingHandControlState;
   private final ObjectManipulationState objectManipulationState;
   private final LoadBearingPlaneHandControlState loadBearingPlaneFingersBentBackState;
   private final List<TaskspaceHandPositionControlState> taskSpacePositionControlStates = new ArrayList<TaskspaceHandPositionControlState>();
   private final PointPositionHandControlState pointPositionControlState;

   private final EnumYoVariable<IndividualHandControlState> requestedState;
   private final HandControllerInterface handController;
   private final OneDoFJoint[] oneDoFJoints;
   private final GeometricJacobian jacobian;
   private final String name;
   private final RobotSide robotSide;
   private final TwistCalculator twistCalculator;
   private final SE3PDGains defaultGains = new SE3PDGains();
   private final Map<ReferenceFrame, YoSE3ConfigurationProvider> currentDesiredConfigurationProviders = new LinkedHashMap<ReferenceFrame,
                                                                                                           YoSE3ConfigurationProvider>();

   public IndividualHandControlModule(final DoubleYoVariable simulationTime, final RobotSide robotSide, FullRobotModel fullRobotModel,
                                      final TwistCalculator twistCalculator, final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                      HandControllerInterface handController, double gravity, final double controlDT,
                                      MomentumBasedController momentumBasedController, GeometricJacobian jacobian, final YoVariableRegistry parentRegistry)
   {
      RigidBody endEffector = jacobian.getEndEffector();

      this.robotSide = robotSide;
      name = endEffector.getName() + getClass().getSimpleName();
      registry = new YoVariableRegistry(name);
      this.twistCalculator = twistCalculator;
      this.handController = handController;

      defaultGains.set(100.0, 1.0, 100.0, 1.0);

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

      if (handController != null)
         this.toolBody = new MassMatrixEstimatingToolRigidBody(name + "Tool", handController.getWristJoint(), fullRobotModel, gravity, controlDT, registry,
                 dynamicGraphicObjectsListRegistry);
      else
         this.toolBody = null;

      this.jacobian = jacobian;

      stateMachine = new StateMachine<IndividualHandControlState>(name, name + "SwitchTime", IndividualHandControlState.class, simulationTime, registry);

      handSpatialAccelerationControlModules = new LinkedHashMap<ReferenceFrame, RigidBodySpatialAccelerationControlModule>();

      ReferenceFrame endEffectorFrame = jacobian.getEndEffectorFrame();
      initialConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(endEffectorFrame));
      finalConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(endEffectorFrame));    // FIXME: make Yo, but is difficult because frame can change

      straightLinePositionWorldTrajectoryGenerators = new LinkedHashMap<ReferenceFrame, StraightLinePositionTrajectoryGenerator>();
      orientationInterpolationWorldTrajectoryGenerators = new LinkedHashMap<ReferenceFrame, OrientationInterpolationTrajectoryGenerator>();

      holdPositionTrajectoryGenerators = new LinkedHashMap<ReferenceFrame, ConstantPositionTrajectoryGenerator>();
      holdOrientationTrajectoryGenerators = new LinkedHashMap<ReferenceFrame, ConstantOrientationTrajectoryGenerator>();

      loadBearingCylindricalState = new LoadBearingCylindricalHandControlState(IndividualHandControlState.LOAD_BEARING_CYLINDRICAL, momentumBasedController,
              jacobian, fullRobotModel.getElevator(), parentRegistry, robotSide);


      progressiveUnloadingHandControlState = new ProgressiveUnloadingHandControlState(IndividualHandControlState.UNLOADING_STATE, momentumBasedController,
              jacobian, fullRobotModel.getElevator(), handController, parentRegistry, robotSide);

      loadBearingPlaneFingersBentBackState = new LoadBearingPlaneHandControlState(IndividualHandControlState.LOAD_BEARING_PLANE_FINGERS_BENT_BACK, robotSide,
              momentumBasedController, fullRobotModel.getElevator(), jacobian, handController, registry);

      jointSpaceHandControlState = new JointSpaceHandControlControlState(IndividualHandControlState.JOINT_SPACE, robotSide, jacobian, momentumBasedController,
              registry, 1.0);

      objectManipulationState = new ObjectManipulationState(IndividualHandControlState.OBJECT_MANIPULATION, robotSide, momentumBasedController, jacobian,
              handController, toolBody, dynamicGraphicObjectsListRegistry, parentRegistry);

      taskSpacePositionControlState = new TaskspaceHandPositionControlState(IndividualHandControlState.TASK_SPACE_POSITION, robotSide, momentumBasedController,
              jacobian, dynamicGraphicObjectsListRegistry, registry);

      pointPositionControlState = new PointPositionHandControlState(momentumBasedController, robotSide, dynamicGraphicObjectsListRegistry, registry);

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

      addTransitionToCylindricalLoadBearing(requestedState, handController, jointSpaceHandControlState, loadBearingCylindricalState, simulationTime);
      addTransitionToCylindricalLoadBearing(requestedState, handController, taskSpacePositionControlState, loadBearingCylindricalState, simulationTime);
//      addTransitionToLeaveCylindricalLoadBearing(handController, loadBearingCylindricalState, taskSpacePositionControlState);
      addRequestedStateTransition(requestedState, true, loadBearingCylindricalState, progressiveUnloadingHandControlState, taskSpacePositionControlState);

      addTransitionToPlaneLoadBearingFingersBentBack(requestedState, handController, taskSpacePositionControlState, loadBearingPlaneFingersBentBackState);
      addRequestedStateTransition(requestedState, true, loadBearingPlaneFingersBentBackState, taskSpacePositionControlState);

      stateMachine.addState(jointSpaceHandControlState);
      stateMachine.addState(taskSpacePositionControlState);
      stateMachine.addState(objectManipulationState);
      stateMachine.addState(loadBearingCylindricalState);
      stateMachine.addState(progressiveUnloadingHandControlState);
      stateMachine.addState(loadBearingPlaneFingersBentBackState);
      stateMachine.addState(pointPositionControlState);

      taskSpacePositionControlStates.add(taskSpacePositionControlState);
      taskSpacePositionControlStates.add(objectManipulationState);

      parentRegistry.addChild(registry);
   }

   public void setInitialState(IndividualHandControlState state)
   {
      stateMachine.setCurrentState(state);
   }

   private static void addTransitionToCylindricalLoadBearing(final EnumYoVariable<IndividualHandControlState> requestedState,
           final HandControllerInterface handControllerInterface, State<IndividualHandControlState> fromState, final State<IndividualHandControlState> toState,
           final DoubleYoVariable time)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            boolean transitionRequested = requestedState.getEnumValue() == toState.getStateEnum();
            boolean ableToBearLoad = handControllerInterface.isAbleToBearLoad();
            boolean initializedClosedHack = time.getDoubleValue() < .01;    // FIXME: get rid of this. Currently necessary for getting into car

            return transitionRequested && (ableToBearLoad || initializedClosedHack);
         }
      };
      StateTransition<IndividualHandControlState> stateTransition = new StateTransition<IndividualHandControlState>(toState.getStateEnum(),
                                                                       stateTransitionCondition);
      fromState.addStateTransition(stateTransition);
   }

   private static void addTransitionToLeaveCylindricalLoadBearing(final EnumYoVariable<IndividualHandControlState> requestedState,
           final HandControllerInterface handControllerInterface, State<IndividualHandControlState> fromState, final State<IndividualHandControlState> toState)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            boolean transitionRequested = requestedState.getEnumValue() == toState.getStateEnum();

            return transitionRequested;
         }
      };
      StateTransitionAction stateTransitionAction = new StateTransitionAction()
      {
         public void doTransitionAction()
         {
            handControllerInterface.openFingers();
         }
      };
      StateTransition<IndividualHandControlState> stateTransition = new StateTransition<IndividualHandControlState>(toState.getStateEnum(),
                                                                       stateTransitionCondition, stateTransitionAction);
      fromState.addStateTransition(stateTransition);
   }

   private static void addTransitionToPlaneLoadBearingFingersBentBack(final EnumYoVariable<IndividualHandControlState> requestedState,
           final HandControllerInterface handControllerInterface, State<IndividualHandControlState> fromState, final State<IndividualHandControlState> toState)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            boolean transitionRequested = requestedState.getEnumValue() == toState.getStateEnum();
            boolean ableToBearLoad = handControllerInterface.areFingersBentBack();

            return transitionRequested && ableToBearLoad;
         }
      };
      StateTransition<IndividualHandControlState> stateTransition = new StateTransition<IndividualHandControlState>(toState.getStateEnum(),
                                                                       stateTransitionCondition);
      fromState.addStateTransition(stateTransition);
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
      RigidBodySpatialAccelerationControlModule rigidBodySpatialAccelerationControlModule =
         getOrCreateRigidBodySpatialAccelerationControlModule(frameToControlPoseOf);
      rigidBodySpatialAccelerationControlModule.setGains(gains);
      state.setTrajectory(positionTrajectory, orientationTrajectory, base, rigidBodySpatialAccelerationControlModule);
      requestedState.set(state.getStateEnum());
      stateMachine.checkTransitionConditions();
   }

   public void executePointPositionTrajectory(PositionTrajectoryGenerator positionTrajectoryGenerator, PositionController positionController,
           FramePoint pointToControlPositionOf, GeometricJacobian jacobian)
   {
      pointPositionControlState.setTrajectory(positionTrajectoryGenerator, positionController, pointToControlPositionOf, jacobian);
      requestedState.set(pointPositionControlState.getStateEnum());
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
      pose.getTransformFromPoseToFrame(oldTrackingFrameTransform);
      Transform3D transformFromNewTrackingFrameToOldTrackingFrame =
         frameToControlPoseOf.getTransformToDesiredFrame(taskspaceHandPositionControlState.getFrameToControlPoseOf());

      Transform3D newTrackingFrameTransform = new Transform3D();
      newTrackingFrameTransform.mul(oldTrackingFrameTransform, transformFromNewTrackingFrameToOldTrackingFrame);
      pose.set(trajectoryFrame, newTrackingFrameTransform);

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
      pose.getTransformFromPoseToFrame(oldTrackingFrameTransform);
      Transform3D transformFromNewTrackingFrameToOldTrackingFrame =
            frameToControlPoseOf.getTransformToDesiredFrame(pointPositionHandControlState.getFrameToControlPoseOf());

      Transform3D newTrackingFrameTransform = new Transform3D();
      newTrackingFrameTransform.mul(oldTrackingFrameTransform, transformFromNewTrackingFrameToOldTrackingFrame);
      pose.set(trajectoryFrame, newTrackingFrameTransform);

      return pose;
   }

   public boolean isInCylindricalLoadBearingState()
   {
      return stateMachine.isCurrentState(IndividualHandControlState.LOAD_BEARING_CYLINDRICAL);
   }

   public void requestLoadBearing()
   {
      if (handController.isClosing())
         requestedState.set(loadBearingCylindricalState.getStateEnum());
      else if (handController.areFingersBendingBack())
         requestedState.set(loadBearingPlaneFingersBentBackState.getStateEnum());
   }

   public void executeJointSpaceTrajectory(Map<OneDoFJoint, ? extends DoubleTrajectoryGenerator> trajectories)
   {
      jointSpaceHandControlState.setTrajectories(trajectories);
      requestedState.set(jointSpaceHandControlState.getStateEnum());
      stateMachine.checkTransitionConditions();
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

      executeJointSpaceTrajectory(quinticPolynomialTrajectoryGenerators);
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
      holdPositionInFrame(jacobian.getBaseFrame(), jacobian.getBase(), defaultGains);
   }

   public void holdPositionInFrame(ReferenceFrame frame, RigidBody base, SE3PDGains gains)
   {
      ReferenceFrame endEffectorFrame = jacobian.getEndEffectorFrame();
      FramePose pose = new FramePose(endEffectorFrame);
      pose.changeFrame(frame);
      initialConfigurationProvider.set(pose);
      PositionTrajectoryGenerator positionTrajectory = getOrCreateConstantPositionTrajectoryGenerator(frame);
      OrientationTrajectoryGenerator orientationTrajectory = getOrCreateConstantOrientationTrajectoryGenerator(frame);

      executeTaskSpaceTrajectory(positionTrajectory, orientationTrajectory, endEffectorFrame, base, isHoldingObject(), gains);
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
         ret = new ConstantPositionTrajectoryGenerator(name + "Constant" + referenceFrame.getName(), referenceFrame,
            initialConfigurationProvider, 0.0, registry);
         holdPositionTrajectoryGenerators.put(referenceFrame, ret);
      }

      return ret;
   }

   private ConstantOrientationTrajectoryGenerator getOrCreateConstantOrientationTrajectoryGenerator(ReferenceFrame referenceFrame)
   {
      ConstantOrientationTrajectoryGenerator ret = holdOrientationTrajectoryGenerators.get(referenceFrame);
      if (ret == null)
      {
         ret = new ConstantOrientationTrajectoryGenerator(name + "Constant" + referenceFrame.getName(), jacobian.getBaseFrame(),
               initialConfigurationProvider, 0.0, registry);
         holdOrientationTrajectoryGenerators.put(referenceFrame, ret);
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

         handSpatialAccelerationControlModules.put(handPositionControlFrame, ret);
      }

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
         ret = new YoSE3ConfigurationProvider("currentDesired" + frameToControlPositionOf.getName() + "Configuration", ReferenceFrame.getWorldFrame(),
                 registry);
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
