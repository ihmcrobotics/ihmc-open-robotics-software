package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class QuadrupedSupportState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int dofs = 3;

   private final RobotQuadrant robotQuadrant;
   private final YoPlaneContactState contactState;

   private final RigidBodyBasics rootBody;
   private final RigidBodyBasics footBody;
   private final ReferenceFrame soleFrame;
   private final PoseReferenceFrame desiredSoleFrame;

   private final FootSwitchInterface footSwitch;

   private final FrameVector3D footNormalContactVector = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);
   private boolean footIsVerifiedAsLoaded = false;

   private final DoubleParameter minimumTimeInSupportState;

   private final YoFramePoint3D groundPlanePosition;
   private final YoFramePoint3D upcomingGroundPlanePosition;

   private final SpatialAcceleration footAcceleration = new SpatialAcceleration();

   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();


   private final SelectionMatrix6D accelerationSelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix6D feedbackSelectionMatrix = new SelectionMatrix6D();

   private final QuadrupedFootControlModuleParameters parameters;

   private final YoBoolean footBarelyLoaded;
   private final DoubleParameter footLoadThreshold;
   private final boolean[] isDirectionFeedbackControlled = new boolean[dofs];

   private final FramePose3D bodyFixedControlledPose = new FramePose3D();

   private final FramePoint3D footPosition = new FramePoint3D();
   private final FrameQuaternion footOrientation = new FrameQuaternion();

   private final FramePoint3D desiredCoPPosition = new FramePoint3D(worldFrame);
   private final FramePoint3D desiredPosition = new FramePoint3D(worldFrame);
   private final FrameVector3D desiredLinearVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3D desiredLinearAcceleration = new FrameVector3D(worldFrame);

   public QuadrupedSupportState(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.groundPlanePosition = controllerToolbox.getGroundPlanePositions().get(robotQuadrant);
      this.upcomingGroundPlanePosition = controllerToolbox.getUpcomingGroundPlanePositions().get(robotQuadrant);
      this.contactState = controllerToolbox.getFootContactState(robotQuadrant);
      this.soleFrame = controllerToolbox.getSoleReferenceFrame(robotQuadrant);
      this.footBody = contactState.getRigidBody();
      this.parameters = controllerToolbox.getFootControlModuleParameters();

      desiredSoleFrame = new PoseReferenceFrame(robotQuadrant.getShortName() + "DesiredSoleFrame", worldFrame);

      rootBody = controllerToolbox.getFullRobotModel().getElevator();

      desiredLinearVelocity.setToZero(worldFrame);
      desiredLinearAcceleration.setToZero(worldFrame);

      spatialAccelerationCommand.setWeight(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialAccelerationCommand.set(rootBody, footBody);
      spatialAccelerationCommand.setPrimaryBase(controllerToolbox.getFullRobotModel().getBody());
      spatialAccelerationCommand.setSelectionMatrixForLinearControl();

      spatialFeedbackControlCommand.setWeightForSolver(SolverWeightLevels.FOOT_SUPPORT_WEIGHT);
      spatialFeedbackControlCommand.set(rootBody, footBody);
      spatialFeedbackControlCommand.setPrimaryBase(controllerToolbox.getFullRobotModel().getBody());

      minimumTimeInSupportState = new DoubleParameter(robotQuadrant.getShortName() + "TimeInSupportState", registry, 0.05);

      footBarelyLoaded = new YoBoolean(robotQuadrant.getShortName() + "_BarelyLoaded", registry);
      footLoadThreshold = new DoubleParameter(robotQuadrant.getShortName() + "_FootLoadThreshold", registry, 0.15);

      footSwitch = controllerToolbox.getRuntimeEnvironment().getFootSwitches().get(robotQuadrant);
   }

   @Override
   public void onEntry()
   {
      contactState.setFullyConstrained();
      contactState.setContactNormalVector(footNormalContactVector);

      if (waypointCallback != null)
         waypointCallback.isDoneMoving(robotQuadrant, true);

      footIsVerifiedAsLoaded = false;

      for (int i = 0; i < dofs; i++)
         isDirectionFeedbackControlled[i] = false;

      footBarelyLoaded.set(false);
   }


   private final FramePoint3D tempPoint = new FramePoint3D();
   @Override
   public void doAction(double timeInState)
   {
      // determine foot state
      footBarelyLoaded.set(footSwitch.computeFootLoadPercentage() < footLoadThreshold.getValue());

      updateHoldPositionSetpoints();


      // assemble acceleration command
      ReferenceFrame bodyFixedFrame = contactState.getRigidBody().getBodyFixedFrame();
      footAcceleration.setToZero(bodyFixedFrame, rootBody.getBodyFixedFrame(), soleFrame);
      footAcceleration.setBodyFrame(bodyFixedFrame);
      spatialAccelerationCommand.setSpatialAcceleration(soleFrame, footAcceleration);
      spatialAccelerationCommand.setLinearWeights(parameters.getSupportFootWeights());

      if (footSwitch.hasFootHitGround())
      {
         if (!footIsVerifiedAsLoaded && timeInState > minimumTimeInSupportState.getValue())
         {
            footIsVerifiedAsLoaded = true;

            tempPoint.setToZero(soleFrame);
            groundPlanePosition.setMatchingFrame(tempPoint);
            upcomingGroundPlanePosition.setMatchingFrame(tempPoint);
         }
      }

      // assemble feedback command
      bodyFixedControlledPose.setToZero(soleFrame);
      bodyFixedControlledPose.changeFrame(footBody.getBodyFixedFrame());
      desiredCoPPosition.setToZero(desiredSoleFrame);
      desiredCoPPosition.changeFrame(worldFrame);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyFixedControlledPose);
      spatialFeedbackControlCommand.set(desiredCoPPosition, desiredLinearVelocity);
      spatialFeedbackControlCommand.setFeedForwardLinearAction(desiredLinearAcceleration);
      spatialFeedbackControlCommand.setLinearWeightsForSolver(parameters.getSupportFootWeights());
      spatialFeedbackControlCommand.setPositionGains(parameters.getHoldPositionGains());

      // set selection matrices
      accelerationSelectionMatrix.setToLinearSelectionOnly();
      feedbackSelectionMatrix.setToLinearSelectionOnly();

      for (int i = 0; i < dofs; i++)
         isDirectionFeedbackControlled[i] = false;

      if (footBarelyLoaded.getBooleanValue())
      {
         isDirectionFeedbackControlled[0] = true; // control x position
         isDirectionFeedbackControlled[1] = true; // control y position
      }

      for (int i = 0; i < dofs; i++)
      {
         if (isDirectionFeedbackControlled[i])
            accelerationSelectionMatrix.getLinearPart().selectAxis(i, false);
         else
            feedbackSelectionMatrix.getLinearPart().selectAxis(i, false);
      }

      spatialAccelerationCommand.setSelectionMatrix(accelerationSelectionMatrix);
      spatialFeedbackControlCommand.setSelectionMatrix(feedbackSelectionMatrix);
   }

   private void updateHoldPositionSetpoints()
   {
      footPosition.setToZero(soleFrame);
      footOrientation.setToZero(soleFrame);
      footPosition.changeFrame(worldFrame);
      footOrientation.changeFrame(worldFrame);

      desiredPosition.checkReferenceFrameMatch(footPosition);

      // The z component is always updated as it is never held in place
      if (footBarelyLoaded.getBooleanValue()) // => Holding X-Y-Components (cuz barely loaded)
      { // Update pitch and roll for when the CoP will get on the edge, and z as always
         desiredPosition.setZ(footPosition.getZ());
      }
      else // Not holding anything
      { // Update the full pose.
         desiredPosition.set(footPosition);
      }

      desiredSoleFrame.setPoseAndUpdate(desiredPosition, footOrientation);
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      return null;
   }

   @Override
   public void onExit()
   {
      footIsVerifiedAsLoaded = false;
      footBarelyLoaded.set(false);
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return null;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }
}
