package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controlModules.TaskspaceTrajectoryStatusMessageHelper;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * A rigid body control mode for orientation only. See {@link RigidBodyOrientationControlHelper}.
 * <p>
 * This class also features a hybrid control mode, where the jointspace control commands from
 * {@link RigidBodyJointControlHelper} are also included. This is useful for more control over, for example,
 * the arms of the robot by specifying desireds in both taskspace and jointspace, which can help avoid
 * singularities and instabilities.
 * </p>
 */
public class RigidBodyOrientationController extends RigidBodyTaskspaceControlState
{
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   private final YoBoolean usingWeightFromMessage;

   private final YoInteger numberOfPointsInQueue;
   private final YoInteger numberOfPointsInGenerator;
   private final YoInteger numberOfPoints;

   private final RigidBodyOrientationControlHelper orientationHelper;

   private final YoBoolean hybridModeActive;
   private final RigidBodyJointControlHelper jointControlHelper;

   private final TaskspaceTrajectoryStatusMessageHelper statusHelper;

   public RigidBodyOrientationController(RigidBodyBasics bodyToControl,
                                         RigidBodyBasics baseBody,
                                         RigidBodyBasics elevator,
                                         ReferenceFrame baseFrame,
                                         YoDouble yoTime,
                                         RigidBodyJointControlHelper jointControlHelper,
                                         boolean enableFunctionGenerators,
                                         YoRegistry parentRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, bodyToControl.getName(), yoTime, parentRegistry);

      String bodyName = bodyToControl.getName();
      String prefix = bodyName + "OrientationTaskspace";

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);

      usingWeightFromMessage = new YoBoolean(prefix + "UsingWeightFromMessage", registry);
      BooleanParameter useBaseFrameForControl = new BooleanParameter(prefix + "UseBaseFrameForControl", registry, false);
      // Must be the body frame until the controller core allows custom control frame rotations for orientation commands:
      MovingReferenceFrame controlFrame = bodyToControl.getBodyFixedFrame();
      orientationHelper = new RigidBodyOrientationControlHelper(prefix,
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
   }

   public void setGains(PID3DGainsReadOnly gains)
   {
      orientationHelper.setGains(gains);
   }

   public void setWeights(Vector3DReadOnly weights)
   {
      orientationHelper.setWeights(weights);
   }

   @Override
   public void holdCurrent()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      orientationHelper.holdCurrent();
   }

   @Override
   public void holdCurrentDesired()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      orientationHelper.holdCurrentDesired();
   }

   @Override
   public void goToPoseFromCurrent(FramePose3DReadOnly pose, double trajectoryTime)
   {
      goToOrientationFromCurrent(pose.getOrientation(), trajectoryTime);
   }

   @Override
   public void goToPose(FramePose3DReadOnly pose, double trajectoryTime)
   {
      goToOrientation(pose.getOrientation(), trajectoryTime);
   }

   public void goToOrientationFromCurrent(FrameQuaternionReadOnly orientation, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      orientationHelper.goToOrientationFromCurrent(orientation, trajectoryTime);
   }

   public void goToOrientation(FrameQuaternionReadOnly orientation, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      orientationHelper.goToOrientation(orientation, trajectoryTime);
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
      double timeInTrajectory = getTimeInTrajectory();
      trajectoryDone.set(orientationHelper.doAction(timeInTrajectory));

      numberOfPointsInQueue.set(orientationHelper.getNumberOfPointsInQueue());
      numberOfPointsInGenerator.set(orientationHelper.getNumberOfPointsInGenerator());
      numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());

      if (hybridModeActive.getBooleanValue())
      {
         jointControlHelper.doAction(timeInTrajectory);
      }

      statusHelper.updateWithTimeInTrajectory(timeInTrajectory);

      updateGraphics();
   }

   @Override
   public boolean handleTrajectoryCommand(SO3TrajectoryControllerCommand command)
   {
      if (handleCommandInternal(command) && orientationHelper.handleTrajectoryCommand(command))
      {
         usingWeightFromMessage.set(orientationHelper.isMessageWeightValid());
         if (command.getExecutionMode() != ExecutionMode.STREAM)
            statusHelper.registerNewTrajectory(command);
         return true;
      }

      clear();
      orientationHelper.clear();
      return false;
   }

   @Override
   public boolean handleHybridTrajectoryCommand(SO3TrajectoryControllerCommand command,
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
      orientationHelper.clear();
      return false;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      // TODO: this can be removed once the controller core can handle control frame orientations with orientation commands.
      if (Math.abs(orientationHelper.getFeedbackControlCommand().getBodyFixedOrientationToControl().getS()) < 1.0 - 1.0e-5)
      {
         throw new RuntimeException("Control frame orientations for orientation control only are not supported!");
      }

      if (hybridModeActive.getBooleanValue())
      {
         feedbackControlCommandList.clear();
         feedbackControlCommandList.addCommand(orientationHelper.getFeedbackControlCommand());
         feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
         return feedbackControlCommandList;
      }

      return orientationHelper.getFeedbackControlCommand();
   }

   public FrameQuaternionReadOnly getDesiredOrientation()
   {
      return orientationHelper.getFeedbackControlCommand().getReferenceOrientation();
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      feedbackControlCommandList.clear();
      feedbackControlCommandList.addCommand(orientationHelper.getFeedbackControlCommand());
      if (jointControlHelper != null)
      {
         feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
      }
      return feedbackControlCommandList;
   }

   @Override
   public void onExit(double timeInState)
   {
      orientationHelper.onExit();
      hideGraphics();
      clear();
   }

   @Override
   public boolean isEmpty()
   {
      return orientationHelper.isEmpty();
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      return orientationHelper.getLastTrajectoryPointTime();
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
      usingWeightFromMessage.set(false);
      trajectoryDone.set(true);
      resetLastCommandId();
   }

   @Override
   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return statusHelper.pollStatusMessage(orientationHelper.getFeedbackControlCommand());
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}
