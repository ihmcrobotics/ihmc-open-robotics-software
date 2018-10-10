package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.CommandConversionTools;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class RigidBodyTaskspaceControlState extends RigidBodyControlState
{
   public static final double timeEpsilonForInitialPoint = 0.05;
   public static final int maxPoints = 10000;
   public static final int maxPointsInGenerator = 5;

   private final SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   private final BooleanParameter useBaseFrameForControl = new BooleanParameter("UseBaseFrameForControl", registry, false);
   private final YoBoolean usingWeightFromMessage;

   private final YoInteger numberOfPointsInQueue;
   private final YoInteger numberOfPointsInGenerator;
   private final YoInteger numberOfPoints;

   private final SO3TrajectoryControllerCommand so3Command = new SO3TrajectoryControllerCommand();
   private final EuclideanTrajectoryControllerCommand euclideanCommand = new EuclideanTrajectoryControllerCommand();

   private final RigidBodyPositionControlHelper positionControlHelper;
   private final RigidBodyOrientationControlHelper orientationControlHelper;

   private final YoBoolean hybridModeActive;
   private final RigidBodyJointControlHelper jointControlHelper;

   public RigidBodyTaskspaceControlState(String postfix, RigidBody bodyToControl, RigidBody baseBody, RigidBody elevator,
                                         Collection<ReferenceFrame> trajectoryFrames, ReferenceFrame controlFrame, ReferenceFrame baseFrame,
                                         boolean enablePositionTracking, boolean enableOrientationTracking, YoDouble yoTime,
                                         RigidBodyJointControlHelper jointControlHelper, YoGraphicsListRegistry graphicsListRegistry,
                                         YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, bodyToControl.getName() + postfix, yoTime, parentRegistry);
      String bodyName = bodyToControl.getName() + postfix;
      String prefix = bodyName + "Taskspace";

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);
      usingWeightFromMessage = new YoBoolean(prefix + "UsingWeightFromMessage", registry);

      positionControlHelper = new RigidBodyPositionControlHelper(postfix, warningPrefix, bodyToControl, baseBody, elevator, trajectoryFrames, controlFrame,
                                                                 baseFrame, useBaseFrameForControl, usingWeightFromMessage, registry, graphicsListRegistry);
      orientationControlHelper = new RigidBodyOrientationControlHelper(postfix, warningPrefix, bodyToControl, baseBody, elevator, trajectoryFrames, baseFrame,
                                                                       useBaseFrameForControl, usingWeightFromMessage, registry);

      this.jointControlHelper = jointControlHelper;
      hybridModeActive = new YoBoolean(prefix + "HybridModeActive", registry);

      feedbackControlCommand.set(elevator, bodyToControl);
      feedbackControlCommand.setPrimaryBase(baseBody);
      feedbackControlCommand.setSelectionMatrixToIdentity();
   }

   public void setWeights(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      positionControlHelper.setWeights(angularWeight);
      orientationControlHelper.setWeights(angularWeight);
   }

   public void setGains(PID3DGainsReadOnly orientationGains, PID3DGainsReadOnly positionGains)
   {
      positionControlHelper.setGains(positionGains);
      orientationControlHelper.setGains(orientationGains);
   }

   @Override
   public void doAction(double timeInState)
   {
      double timeInTrajectory = getTimeInTrajectory();
      boolean positionDone = positionControlHelper.doAction(timeInTrajectory);
      boolean orientationDone = orientationControlHelper.doAction(timeInTrajectory);
      trajectoryDone.set(positionDone && orientationDone);

      updateCommand();

      numberOfPointsInQueue.set(positionControlHelper.getNumberOfPointsInQueue());
      numberOfPointsInGenerator.set(positionControlHelper.getNumberOfPointsInGenerator());
      numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());

      if (hybridModeActive.getBooleanValue())
      {
         jointControlHelper.doAction(timeInTrajectory);
      }

      updateGraphics();
   }

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredLinearVelocity = new FrameVector3D();
   private final FrameVector3D desiredLinearAcceleration = new FrameVector3D();
   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();
   private final FrameVector3D desiredAngularAcceleration = new FrameVector3D();
   private final FramePose3D controlFramePose = new FramePose3D();

   private void updateCommand()
   {
      PointFeedbackControlCommand positionCommand = positionControlHelper.getFeedbackControlCommand();
      OrientationFeedbackControlCommand orientationCommand = orientationControlHelper.getFeedbackControlCommand();

      // Set weight and selection matrices
      SpatialAccelerationCommand accelerationCommand = feedbackControlCommand.getSpatialAccelerationCommand();
      accelerationCommand.getWeightMatrix().setLinearPart(positionCommand.getSpatialAccelerationCommand().getWeightMatrix().getLinearPart());
      accelerationCommand.getWeightMatrix().setAngularPart(orientationCommand.getSpatialAccelerationCommand().getWeightMatrix().getAngularPart());
      accelerationCommand.getSelectionMatrix().setLinearPart(positionCommand.getSpatialAccelerationCommand().getSelectionMatrix().getLinearPart());
      accelerationCommand.getSelectionMatrix().setAngularPart(orientationCommand.getSpatialAccelerationCommand().getSelectionMatrix().getAngularPart());

      // Set feedback gains
      feedbackControlCommand.getGains().setPositionGains(positionCommand.getGains());
      feedbackControlCommand.getGains().setOrientationGains(orientationCommand.getGains());
      feedbackControlCommand.setGainsFrames(orientationCommand.getAngularGainsFrame(), positionCommand.getLinearGainsFrame());

      // Copy over desired values
      positionCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity);
      positionCommand.getFeedForwardActionIncludingFrame(desiredLinearAcceleration);
      orientationCommand.getIncludingFrame(desiredOrientation, desiredAngularVelocity);
      orientationCommand.getFeedForwardActionIncludingFrame(desiredAngularAcceleration);
      feedbackControlCommand.set(desiredPosition, desiredLinearVelocity);
      feedbackControlCommand.setFeedForwardLinearAction(desiredLinearAcceleration);
      feedbackControlCommand.set(desiredOrientation, desiredAngularVelocity);
      feedbackControlCommand.setFeedForwardAngularAction(desiredAngularAcceleration);

      // Copy from the position command since the orientation does not have a control frame.
      positionCommand.getSpatialAccelerationCommand().getControlFramePoseIncludingFrame(controlFramePose);
      feedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePose);

      feedbackControlCommand.setControlBaseFrame(positionCommand.getControlBaseFrame());
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void onExit()
   {
      positionControlHelper.onExit();
      orientationControlHelper.onExit();
      hideGraphics();
      clear();
   }

   public void holdCurrent()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionControlHelper.holdCurrent();
      orientationControlHelper.holdCurrent();
      trajectoryDone.set(false);
   }

   public void holdCurrentDesired()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionControlHelper.holdCurrentDesired();
      orientationControlHelper.holdCurrentDesired();
      trajectoryDone.set(false);
   }

   public void goToPoseFromCurrent(FramePose3DReadOnly pose, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionControlHelper.goToPositionFromCurrent(pose.getPosition(), trajectoryTime);
      orientationControlHelper.goToOrientationFromCurrent(pose.getOrientation(), trajectoryTime);
      trajectoryDone.set(false);
   }

   public void goToPose(FramePose3D pose, FramePose3D initialPose, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionControlHelper.goToPosition(pose.getPosition(), initialPose.getPosition(), trajectoryTime);
      orientationControlHelper.goToOrientation(pose.getOrientation(), initialPose.getOrientation(), trajectoryTime);
      trajectoryDone.set(false);
   }

   public void setControlFramePosition(FramePoint3DReadOnly controlFramePosition)
   {
      positionControlHelper.setControlFramePosition(controlFramePosition);
   }

   public void setDefaultControlFrame()
   {
      positionControlHelper.setDefaultControlFrame();
   }

   public FramePoint3DReadOnly getControlFramePosition()
   {
      return positionControlHelper.getControlFramePosition();
   }

   public boolean handleOrientationTrajectoryCommand(SO3TrajectoryControllerCommand command, FramePose3D initialPose)
   {
      if (!handleCommandInternal(command))
      {
         return false;
      }

      return orientationControlHelper.handleSO3TrajectoryCommand(command, initialPose.getOrientation());
   }

   public boolean handleEuclideanTrajectoryCommand(EuclideanTrajectoryControllerCommand command, FramePose3D initialPose)
   {
      if (!handleCommandInternal(command))
      {
         return false;
      }

      return positionControlHelper.handleEuclideanTrajectoryCommand(command, initialPose.getPosition());
   }

   public boolean handlePoseTrajectoryCommand(SE3TrajectoryControllerCommand command, FramePose3D initialPose)
   {
      if (!handleCommandInternal(command))
      {
         return false;
      }

      CommandConversionTools.convertToEuclidean(command, euclideanCommand);
      CommandConversionTools.convertToSO3(command, so3Command);

      boolean positionHandled = positionControlHelper.handleEuclideanTrajectoryCommand(euclideanCommand, initialPose.getPosition());
      boolean orientationHandled = orientationControlHelper.handleSO3TrajectoryCommand(so3Command, initialPose.getOrientation());
      return positionHandled && orientationHandled;
   }

   public boolean handleHybridPoseTrajectoryCommand(SE3TrajectoryControllerCommand command, FramePose3D initialPose,
                                                    JointspaceTrajectoryCommand jointspaceCommand, double[] initialJointPositions)
   {
      if (!handleCommandInternal(jointspaceCommand))
      {
         return false;
      }

      if (jointControlHelper == null)
      {
         PrintTools.warn(warningPrefix + "Can not use hybrid mode. Was not created with a jointspace helper.");
         return false;
      }

      if (!jointControlHelper.handleTrajectoryCommand(jointspaceCommand, initialJointPositions))
      {
         return false;
      }

      if (!handlePoseTrajectoryCommand(command, initialPose))
      {
         return false;
      }

      hybridModeActive.set(true);
      return true;
   }

   public void getDesiredPose(FramePose3DBasics desiredPoseToPack)
   {
      getDesiredPosition(desiredPoseToPack.getPosition());
      getDesiredOrientation(desiredPoseToPack.getOrientation());
   }

   public void getDesiredPosition(FixedFramePoint3DBasics desiredPositionToPack)
   {
      positionControlHelper.getDesiredPosition(desiredPositionToPack);
   }

   public void getDesiredOrientation(FixedFrameQuaternionBasics desiredOrientationToPack)
   {
      orientationControlHelper.getDesiredOrientation(desiredOrientationToPack);
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      feedbackControlCommandList.clear();
      feedbackControlCommandList.addCommand(feedbackControlCommand);
      if (hybridModeActive.getBooleanValue())
      {
         feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
      }
      return feedbackControlCommandList;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      feedbackControlCommandList.clear();
      feedbackControlCommandList.addCommand(feedbackControlCommand);
      if (jointControlHelper != null)
      {
         feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
      }
      return feedbackControlCommandList;
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      return positionControlHelper.getLastTrajectoryPointTime();
   }

   @Override
   public boolean isEmpty()
   {
      return positionControlHelper.isEmpty();
   }

   public void clear()
   {
      positionControlHelper.clear();
      orientationControlHelper.clear();
      numberOfPointsInQueue.set(0);
      numberOfPointsInGenerator.set(0);
      numberOfPoints.set(0);
      hybridModeActive.set(false);
      usingWeightFromMessage.set(false);
      resetLastCommandId();
   }
}
