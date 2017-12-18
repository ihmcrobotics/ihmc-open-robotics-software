package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.robotics.trajectories.providers.SettablePositionProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class MoveViaWaypointsState extends AbstractFootControlState
{
   private final YoBoolean isPerformingTouchdown;
   private final SettableDoubleProvider touchdownInitialTimeProvider = new SettableDoubleProvider(0.0);
   private final SettablePositionProvider currentDesiredFootPosition = new SettablePositionProvider();
   private final SoftTouchdownPositionTrajectoryGenerator positionTrajectoryForDisturbanceRecovery;

   private final RigidBodyTaskspaceControlState taskspaceControlState;
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();

   private final YoFrameVector angularWeight;
   private final YoFrameVector linearWeight;
   private final Vector3D tempAngularWeightVector = new Vector3D();
   private final Vector3D tempLinearWeightVector = new Vector3D();

   private final FramePose initialPose = new FramePose();

   private final RigidBodyTransform controlFrameTransform = new RigidBodyTransform();
   private ReferenceFrame controlFrame;
   private final ReferenceFrame ankleFrame;
   private final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;

   public MoveViaWaypointsState(FootControlHelper footControlHelper, VectorProvider touchdownVelocityProvider, VectorProvider touchdownAccelerationProvider,
         YoPIDSE3Gains gains, YoVariableRegistry registry)
   {
      super(ConstraintType.MOVE_VIA_WAYPOINTS, footControlHelper);

      RigidBody foot = controllerToolbox.getFullRobotModel().getFoot(robotSide);
      String namePrefix = foot.getName() + "MoveViaWaypoints";

      isPerformingTouchdown = new YoBoolean(namePrefix + "IsPerformingTouchdown", registry);
      positionTrajectoryForDisturbanceRecovery = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", worldFrame, currentDesiredFootPosition,
            touchdownVelocityProvider, touchdownAccelerationProvider, touchdownInitialTimeProvider, registry);

      angularWeight = new YoFrameVector(namePrefix + "AngularWeight", null, registry);
      linearWeight = new YoFrameVector(namePrefix + "LinearWeight", null, registry);

      YoDouble yoTime = controllerToolbox.getYoTime();
      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();
      ReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();
      ankleFrame = foot.getParentJoint().getFrameAfterJoint();
      controlFrame = ankleFrame;

      taskspaceControlState = new RigidBodyTaskspaceControlState("", foot, pelvis, rootBody, trajectoryFrames, controlFrame, pelvisFrame, yoTime,
            null, graphicsListRegistry, registry);
      taskspaceControlState.setGains(gains.getOrientationGains(), gains.getPositionGains());

      spatialFeedbackControlCommand.set(rootBody, foot);
      spatialFeedbackControlCommand.setPrimaryBase(pelvis);
      spatialFeedbackControlCommand.setGains(gains);

      legSingularityAndKneeCollapseAvoidanceControlModule = footControlHelper.getLegSingularityAndKneeCollapseAvoidanceControlModule();
   }

   public void setWeights(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      this.angularWeight.set(angularWeight);
      this.linearWeight.set(linearWeight);

      taskspaceControlState.setWeights(angularWeight, linearWeight);
   }

   public void holdCurrentPosition()
   {
      taskspaceControlState.holdCurrent();
   }

   public void handleFootTrajectoryCommand(FootTrajectoryCommand command)
   {
      if (command.useCustomControlFrame())
      {
         command.getControlFramePose(controlFrameTransform);
         taskspaceControlState.setControlFramePose(controlFrameTransform);
      }
      else
      {
         taskspaceControlState.setDefaultControlFrame();
      }

      controlFrame = taskspaceControlState.getControlFrame();
      initialPose.setToZero(controlFrame);

      if (!taskspaceControlState.handlePoseTrajectoryCommand(command, initialPose))
      {
         taskspaceControlState.holdCurrent();
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      taskspaceControlState.doTransitionIntoAction();
      isPerformingTouchdown.set(false);

      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.setCheckVelocityForSwingSingularityAvoidance(false);
      }
   }

   @Override
   public void doSpecificAction()
   {
      if (isPerformingTouchdown.getBooleanValue())
      {
         positionTrajectoryForDisturbanceRecovery.compute(getTimeInCurrentState());
         positionTrajectoryForDisturbanceRecovery.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
         desiredAngularVelocity.setToZero();
         desiredAngularAcceleration.setToZero();

         packCommandForTouchdown();
      }
      else
      {
         taskspaceControlState.doAction();
         spatialFeedbackControlCommand.set(taskspaceControlState.getSpatialFeedbackControlCommand());

         if (taskspaceControlState.abortState())
            requestTouchdownForDisturbanceRecovery();
      }

      doSingularityAvoidance(spatialFeedbackControlCommand);
   }

   private void packCommandForTouchdown()
   {
      spatialFeedbackControlCommand.set(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      spatialFeedbackControlCommand.set(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      angularWeight.get(tempAngularWeightVector);
      linearWeight.get(tempLinearWeightVector);
      spatialFeedbackControlCommand.setWeightsForSolver(tempAngularWeightVector, tempLinearWeightVector);
   }

   public void requestTouchdownForDisturbanceRecovery()
   {
      if (isPerformingTouchdown.getBooleanValue())
         return;

      desiredPosition.setToZero(controlFrame);
      desiredOrientation.setToZero(controlFrame);
      desiredPosition.changeFrame(worldFrame);
      desiredOrientation.changeFrame(worldFrame);

      currentDesiredFootPosition.set(desiredPosition);
      touchdownInitialTimeProvider.setValue(getTimeInCurrentState());
      positionTrajectoryForDisturbanceRecovery.initialize();

      isPerformingTouchdown.set(true);
   }

   private final FramePoint3D desiredAnklePosition = new FramePoint3D();
   private final FramePose desiredPose = new FramePose();

   private void doSingularityAvoidance(SpatialFeedbackControlCommand spatialFeedbackControlCommand)
   {
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         spatialFeedbackControlCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
         spatialFeedbackControlCommand.getIncludingFrame(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

         desiredPose.setPoseIncludingFrame(desiredPosition, desiredOrientation);
         changeDesiredPoseBodyFrame(controlFrame, ankleFrame, desiredPose);
         desiredPose.getPositionIncludingFrame(desiredAnklePosition);

         legSingularityAndKneeCollapseAvoidanceControlModule.correctSwingFootTrajectory(desiredAnklePosition, desiredLinearVelocity, desiredLinearAcceleration);

         desiredPose.setPosition(desiredAnklePosition);
         changeDesiredPoseBodyFrame(ankleFrame, controlFrame, desiredPose);
         desiredPose.getPositionIncludingFrame(desiredPosition);
      }

      spatialFeedbackControlCommand.set(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      spatialFeedbackControlCommand.set(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
   }

   private final RigidBodyTransform oldBodyFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform newBodyFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformFromNewBodyFrameToOldBodyFrame = new RigidBodyTransform();

   private void changeDesiredPoseBodyFrame(ReferenceFrame oldBodyFrame, ReferenceFrame newBodyFrame, FramePose framePoseToModify)
   {
      if (oldBodyFrame == newBodyFrame)
         return;

      framePoseToModify.getPose(oldBodyFrameDesiredTransform);
      newBodyFrame.getTransformToDesiredFrame(transformFromNewBodyFrameToOldBodyFrame, oldBodyFrame);
      newBodyFrameDesiredTransform.set(oldBodyFrameDesiredTransform);
      newBodyFrameDesiredTransform.multiply(transformFromNewBodyFrameToOldBodyFrame);
      framePoseToModify.setPose(newBodyFrameDesiredTransform);
   }

   public void requestStopTrajectory()
   {
      taskspaceControlState.holdCurrent();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }

   @Override
   public void doTransitionOutOfAction()
   {
      taskspaceControlState.doTransitionOutOfAction();
   }
}