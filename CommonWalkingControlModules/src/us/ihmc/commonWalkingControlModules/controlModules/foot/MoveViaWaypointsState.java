package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.robotics.trajectories.providers.SettablePositionProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;

public class MoveViaWaypointsState extends AbstractUnconstrainedState
{
   private final ReferenceFrame footFrame;

   private final BooleanYoVariable isPerformingTouchdown;
   private final SettableDoubleProvider touchdownInitialTimeProvider = new SettableDoubleProvider(0.0);
   private final SettablePositionProvider currentDesiredFootPosition = new SettablePositionProvider();
   private final SoftTouchdownPositionTrajectoryGenerator positionTrajectoryForDisturbanceRecovery;

   private final RigidBodyTaskspaceControlState taskspaceControlState;
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommandForTouchdown = new SpatialFeedbackControlCommand();

   private final Vector3D tempAngularWeightVector = new Vector3D();
   private final Vector3D tempLinearWeightVector = new Vector3D();

   public MoveViaWaypointsState(FootControlHelper footControlHelper, VectorProvider touchdownVelocityProvider, VectorProvider touchdownAccelerationProvider,
         YoSE3PIDGainsInterface gains, YoVariableRegistry registry)
   {
      super(ConstraintType.MOVE_VIA_WAYPOINTS, footControlHelper, gains, registry);

      RigidBody foot = controllerToolbox.getFullRobotModel().getFoot(robotSide);
      String namePrefix = foot.getName() + "MoveViaWaypoints";

      isPerformingTouchdown = new BooleanYoVariable(namePrefix + "IsPerformingTouchdown", registry);
      positionTrajectoryForDisturbanceRecovery = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", worldFrame, currentDesiredFootPosition,
            touchdownVelocityProvider, touchdownAccelerationProvider, touchdownInitialTimeProvider, registry);

      footFrame = foot.getBodyFixedFrame();
      DoubleYoVariable yoTime = controllerToolbox.getYoTime();
      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();
      ReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();
      ReferenceFrame controlFrame = foot.getParentJoint().getFrameAfterJoint();

      taskspaceControlState = new RigidBodyTaskspaceControlState(foot, pelvis, rootBody, trajectoryFrames, controlFrame, pelvisFrame, yoTime,
            graphicsListRegistry, registry);
      taskspaceControlState.setGains(gains.getOrientationGains(), gains.getPositionGains());

      spatialFeedbackControlCommandForTouchdown.set(rootBody, foot);
      spatialFeedbackControlCommandForTouchdown.setPrimaryBase(pelvis);
      spatialFeedbackControlCommandForTouchdown.setGains(gains);
   }

   @Override
   public void setWeight(double weight)
   {
      super.setWeight(weight);
      taskspaceControlState.setWeight(weight);
   }

   @Override
   public void setWeights(Vector3D angularWeight, Vector3D linearWeight)
   {
      super.setWeights(angularWeight, linearWeight);
      taskspaceControlState.setWeights(angularWeight, linearWeight);
   }

   public void holdCurrentPosition()
   {
      taskspaceControlState.holdPose();
   }

   public void handleFootTrajectoryCommand(FootTrajectoryCommand command)
   {
      if (!taskspaceControlState.handlePoseTrajectoryCommand(command))
      {
         taskspaceControlState.holdPose();
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      taskspaceControlState.doTransitionIntoAction();
      isPerformingTouchdown.set(false);
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

         if (taskspaceControlState.abortState())
            requestTouchdownForDisturbanceRecovery();
      }
   }

   private void packCommandForTouchdown()
   {
      spatialFeedbackControlCommandForTouchdown.set(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      spatialFeedbackControlCommandForTouchdown.set(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      angularWeight.get(tempAngularWeightVector);
      linearWeight.get(tempLinearWeightVector);
      spatialFeedbackControlCommandForTouchdown.setWeightsForSolver(tempAngularWeightVector, tempLinearWeightVector);
   }

   public void requestTouchdownForDisturbanceRecovery()
   {
      if (isPerformingTouchdown.getBooleanValue())
         return;

      desiredPosition.setToZero(footFrame);
      desiredOrientation.setToZero(footFrame);
      desiredPosition.changeFrame(worldFrame);
      desiredOrientation.changeFrame(worldFrame);

      currentDesiredFootPosition.set(desiredPosition);
      touchdownInitialTimeProvider.setValue(getTimeInCurrentState());
      positionTrajectoryForDisturbanceRecovery.initialize();

      isPerformingTouchdown.set(true);
   }

   public void requestStopTrajectory()
   {
      taskspaceControlState.holdPose();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      if (isPerformingTouchdown.getBooleanValue())
         return spatialFeedbackControlCommandForTouchdown;

      return taskspaceControlState.getFeedbackControlCommand();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      taskspaceControlState.doTransitionOutOfAction();
   }

   @Override
   protected void computeAndPackTrajectory()
   {

   }

   @Override
   protected void initializeTrajectory()
   {

   }
}