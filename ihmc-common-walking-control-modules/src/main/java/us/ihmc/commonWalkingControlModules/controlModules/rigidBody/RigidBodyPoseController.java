package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controlModules.TaskspaceTrajectoryStatusMessageHelper;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.CommandConversionTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * A rigid body control mode for position and orientation. It combines the functionalities from
 * the {@link RigidBodyPositionControlHelper position} and {@link RigidBodyOrientationControlHelper orientation}
 * control helpers to provide combined spatial feedback control commands for submission to the whole body
 * controller core.
 * <p>
 * This class also features a hybrid control mode, where the jointspace control commands from
 * {@link RigidBodyJointControlHelper} are also included. This is useful for more control over, for example,
 * the arms of the robot by specifying desireds in both taskspace and jointspace, which can help avoid
 * singularities and instabilities.
 * </p>
 */
public class RigidBodyPoseController extends RigidBodyTaskspaceControlState
{
   private final SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   private final YoBoolean usingWeightFromMessage;

   private final YoInteger numberOfPointsInQueue;
   private final YoInteger numberOfPointsInGenerator;
   private final YoInteger numberOfPoints;

   private final SO3TrajectoryControllerCommand so3Command = new SO3TrajectoryControllerCommand();
   private final EuclideanTrajectoryControllerCommand euclideanCommand = new EuclideanTrajectoryControllerCommand();

   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final RigidBodyPositionControlHelper positionHelper;
   private final RigidBodyOrientationControlHelper orientationHelper;

   private final YoBoolean hybridModeActive;
   private final RigidBodyJointControlHelper jointControlHelper;

   private final TaskspaceTrajectoryStatusMessageHelper statusHelper;

   public RigidBodyPoseController(RigidBodyBasics bodyToControl,
                                  RigidBodyBasics baseBody,
                                  RigidBodyBasics elevator,
                                  ReferenceFrame controlFrame,
                                  ReferenceFrame baseFrame,
                                  YoDouble yoTime,
                                  RigidBodyJointControlHelper jointControlHelper,
                                  boolean enableFunctionGenerators,
                                  YoGraphicsListRegistry graphicsListRegistry,
                                  YoRegistry parentRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, bodyToControl.getName(), yoTime, parentRegistry);
      String bodyName = bodyToControl.getName();
      String prefix = bodyName + "Taskspace";

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);

      usingWeightFromMessage = new YoBoolean(prefix + "UsingWeightFromMessage", registry);
      BooleanParameter useBaseFrameForControl = new BooleanParameter(prefix + "UseBaseFrameForControl", registry, false);
      positionHelper = new RigidBodyPositionControlHelper(warningPrefix,
                                                          bodyToControl,
                                                          baseBody,
                                                          elevator,
                                                          controlFrame,
                                                          baseFrame,
                                                          useBaseFrameForControl,
                                                          usingWeightFromMessage,
                                                          enableFunctionGenerators,
                                                          yoTime,
                                                          registry,
                                                          graphicsListRegistry);
      orientationHelper = new RigidBodyOrientationControlHelper(warningPrefix,
                                                                bodyToControl,
                                                                baseBody,
                                                                elevator,
                                                                controlFrame,
                                                                baseFrame,
                                                                useBaseFrameForControl,
                                                                usingWeightFromMessage,
                                                                enableFunctionGenerators,
                                                                yoTime,
                                                                registry);

      this.jointControlHelper = jointControlHelper;
      hybridModeActive = new YoBoolean(prefix + "HybridModeActive", registry);

      statusHelper = new TaskspaceTrajectoryStatusMessageHelper(bodyToControl);

      feedbackControlCommand.set(elevator, bodyToControl);
      feedbackControlCommand.setPrimaryBase(baseBody);
      feedbackControlCommand.setSelectionMatrixToIdentity();
   }

   public void setWeights(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      positionHelper.setWeights(linearWeight);
      orientationHelper.setWeights(angularWeight);
   }

   public Vector3DReadOnly getAngularDefaultWeight()
   {
      return orientationHelper.getDefaultWeight();
   }

   public Vector3DReadOnly getLinearDefaultWeight()
   {
      return positionHelper.getDefaultWeight();
   }

   public void setGains(PID3DGainsReadOnly orientationGains, PID3DGainsReadOnly positionGains)
   {
      positionHelper.setGains(positionGains);
      orientationHelper.setGains(orientationGains);
   }

   public PID3DGainsReadOnly getOrientationGains()
   {
      return orientationHelper.getGains();
   }

   public PID3DGainsReadOnly getPositionGains()
   {
      return positionHelper.getGains();
   }

   @Override
   public void doAction(double timeInState)
   {
      double timeInTrajectory = getTimeInTrajectory();
      boolean positionDone = positionHelper.doAction(timeInTrajectory);
      boolean orientationDone = orientationHelper.doAction(timeInTrajectory);
      trajectoryDone.set(positionDone && orientationDone);

      updateCommand();

      numberOfPointsInQueue.set(positionHelper.getNumberOfPointsInQueue());
      numberOfPointsInGenerator.set(positionHelper.getNumberOfPointsInGenerator());
      numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());

      if (hybridModeActive.getBooleanValue())
      {
         jointControlHelper.doAction(timeInTrajectory);
      }

      statusHelper.updateWithTimeInTrajectory(timeInTrajectory);

      updateGraphics();
   }

   private void updateCommand()
   {
      PointFeedbackControlCommand positionCommand = positionHelper.getFeedbackControlCommand();
      OrientationFeedbackControlCommand orientationCommand = orientationHelper.getFeedbackControlCommand();

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
      feedbackControlCommand.setInverseDynamics(positionCommand.getReferencePosition(),
                                                positionCommand.getReferenceLinearVelocity(),
                                                positionCommand.getReferenceLinearAcceleration());
      feedbackControlCommand.setInverseDynamics(orientationCommand.getReferenceOrientation(),
                                                orientationCommand.getReferenceAngularVelocity(),
                                                orientationCommand.getReferenceAngularAcceleration());

      // Copy from the position command since the orientation does not have a control frame.
      feedbackControlCommand.setControlFrameFixedInEndEffector(positionCommand.getBodyFixedPointToControl(),
                                                               orientationCommand.getBodyFixedOrientationToControl());

      feedbackControlCommand.setControlBaseFrame(positionCommand.getControlBaseFrame());
   }

   @Override
   public void onEntry()
   {
      positionHelper.resetFunctionGenerators();
      orientationHelper.resetFunctionGenerator();
   }

   @Override
   public void onExit(double timeInState)
   {
      positionHelper.onExit();
      orientationHelper.onExit();
      hybridModeActive.set(false);
      hideGraphics();
      clear();
   }

   @Override
   public void holdCurrent()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionHelper.holdCurrent();
      orientationHelper.holdCurrent();
      hybridModeActive.set(false);
   }

   @Override
   public void holdCurrentDesired()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();

      orientationHelper.getDesiredOrientation(desiredOrientation);
      positionHelper.holdCurrentDesired(desiredOrientation);
      orientationHelper.holdCurrentDesired();
      hybridModeActive.set(false);
   }

   @Override
   public void goToPoseFromCurrent(FramePose3DReadOnly pose, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionHelper.goToPositionFromCurrent(pose.getPosition(), trajectoryTime);
      orientationHelper.goToOrientationFromCurrent(pose.getOrientation(), trajectoryTime);
      hybridModeActive.set(false);
   }

   @Override
   public void goToPose(FramePose3DReadOnly pose, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();

      orientationHelper.getDesiredOrientation(desiredOrientation);
      positionHelper.goToPosition(pose.getPosition(), desiredOrientation, trajectoryTime);
      orientationHelper.goToOrientation(pose.getOrientation(), trajectoryTime);
      hybridModeActive.set(false);
   }

   @Override
   public boolean handleTrajectoryCommand(SO3TrajectoryControllerCommand command)
   {
      positionHelper.disable();

      if (handleCommandInternal(command) && orientationHelper.handleTrajectoryCommand(command))
      {
         usingWeightFromMessage.set(orientationHelper.isMessageWeightValid());
         hybridModeActive.set(false);
         if (command.getExecutionMode() != ExecutionMode.STREAM)
            statusHelper.registerNewTrajectory(command);
         return true;
      }

      clear();
      positionHelper.clear();
      orientationHelper.clear();
      return false;
   }

   @Override
   public boolean handleTrajectoryCommand(SE3TrajectoryControllerCommand command)
   {
      if (!handleCommandInternal(command))
      {
         clear();
         positionHelper.clear();
         orientationHelper.clear();
         return false;
      }

      CommandConversionTools.convertToEuclidean(command, euclideanCommand);
      CommandConversionTools.convertToSO3(command, so3Command);

      orientationHelper.getDesiredOrientation(desiredOrientation);

      if (positionHelper.handleTrajectoryCommand(euclideanCommand, desiredOrientation) && orientationHelper.handleTrajectoryCommand(so3Command))
      {
         usingWeightFromMessage.set(positionHelper.isMessageWeightValid() && orientationHelper.isMessageWeightValid());
         hybridModeActive.set(false);
         if (command.getExecutionMode() != ExecutionMode.STREAM)
            statusHelper.registerNewTrajectory(command);
         return true;
      }

      clear();
      positionHelper.clear();
      orientationHelper.clear();
      return false;
   }

   @Override
   public boolean handleHybridTrajectoryCommand(SE3TrajectoryControllerCommand command,
                                                JointspaceTrajectoryCommand jointspaceCommand,
                                                double[] initialJointPositions)
   {
      if (handleTrajectoryCommand(command) && jointControlHelper.handleTrajectoryCommand(jointspaceCommand, initialJointPositions))
      {
         hybridModeActive.set(true);
         statusHelper.registerNewTrajectory(command);
         return true;
      }

      clear();
      positionHelper.clear();
      orientationHelper.clear();
      return false;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      if (hybridModeActive.getBooleanValue())
      {
         feedbackControlCommandList.clear();
         feedbackControlCommandList.addCommand(feedbackControlCommand);
         feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
         return feedbackControlCommandList;
      }

      return feedbackControlCommand;
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
      return positionHelper.getLastTrajectoryPointTime();
   }

   @Override
   public boolean isEmpty()
   {
      return positionHelper.isEmpty();
   }

   @Override
   public boolean isHybridModeActive()
   {
      return hybridModeActive.getValue();
   }

   private void clear()
   {
      numberOfPointsInQueue.set(0);
      numberOfPointsInGenerator.set(0);
      numberOfPoints.set(0);
      hybridModeActive.set(false);
      usingWeightFromMessage.set(false);
      trajectoryDone.set(true);
      resetLastCommandId();
   }

   @Override
   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return statusHelper.pollStatusMessage(feedbackControlCommand);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(positionHelper.getSCS2YoGraphics());
      return group;
   }

   @Override
   public RigidBodyOrientationControlHelper getOrientationControlHelper()
   {
      return orientationHelper;
   }
}
