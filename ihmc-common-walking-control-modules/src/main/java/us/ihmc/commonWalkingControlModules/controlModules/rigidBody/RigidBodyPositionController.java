package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controlModules.TaskspaceTrajectoryStatusMessageHelper;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class RigidBodyPositionController extends RigidBodyTaskspaceControlState
{
   private final YoBoolean usingWeightFromMessage;

   private final YoInteger numberOfPointsInQueue;
   private final YoInteger numberOfPointsInGenerator;
   private final YoInteger numberOfPoints;

   private final ReferenceFrame bodyFrame;
   private final FrameQuaternion currentOrientation = new FrameQuaternion();
   private final RigidBodyPositionControlHelper positionHelper;

   private final TaskspaceTrajectoryStatusMessageHelper statusHelper;

   public RigidBodyPositionController(RigidBodyBasics bodyToControl, RigidBodyBasics baseBody, RigidBodyBasics elevator, ReferenceFrame controlFrame,
                                      ReferenceFrame baseFrame, YoDouble yoTime, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, bodyToControl.getName(), yoTime, parentRegistry);

      String bodyName = bodyToControl.getName();
      String prefix = bodyName + "PositionTaskspace";

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);

      // This needs to be identity to allow for consistent conversions of the control frame point if a message with a control frame is recieved.
      bodyFrame = bodyToControl.getBodyFixedFrame();
      if (!controlFrame.getTransformToDesiredFrame(bodyFrame).getRotationMatrix().isIdentity())
      {
         throw new RuntimeException("The control frame orientation of a position controlled body must be identity!");
      }

      usingWeightFromMessage = new YoBoolean(prefix + "UsingWeightFromMessage", registry);
      BooleanParameter useBaseFrameForControl = new BooleanParameter(prefix + "UseBaseFrameForControl", registry, false);
      positionHelper = new RigidBodyPositionControlHelper(prefix, bodyToControl, baseBody, elevator, controlFrame, baseFrame, useBaseFrameForControl,
                                                          usingWeightFromMessage, registry, graphicsListRegistry);

      graphics.addAll(positionHelper.getGraphics());
      hideGraphics();

      statusHelper = new TaskspaceTrajectoryStatusMessageHelper(bodyToControl);
   }

   public void setGains(PID3DGainsReadOnly gains)
   {
      positionHelper.setGains(gains);
   }

   public void setWeights(Vector3DReadOnly weights)
   {
      positionHelper.setWeights(weights);
   }

   @Override
   public void holdCurrent()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionHelper.holdCurrent();
   }

   @Override
   public void holdCurrentDesired()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      currentOrientation.setToZero(bodyFrame);
      positionHelper.holdCurrentDesired(currentOrientation);
   }

   @Override
   public void goToPoseFromCurrent(FramePose3DReadOnly pose, double trajectoryTime)
   {
      goToPositionFromCurrent(pose.getPosition(), trajectoryTime);
   }

   @Override
   public void goToPose(FramePose3DReadOnly pose, double trajectoryTime)
   {
      goToPosition(pose.getPosition(), trajectoryTime);
   }

   public void goToPositionFromCurrent(FramePoint3DReadOnly position, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionHelper.goToPositionFromCurrent(position, trajectoryTime);
   }

   public void goToPosition(FramePoint3DReadOnly position, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      currentOrientation.setToZero(bodyFrame);
      positionHelper.goToPosition(position, currentOrientation, trajectoryTime);
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
      double timeInTrajectory = getTimeInTrajectory();
      trajectoryDone.set(positionHelper.doAction(timeInTrajectory));

      numberOfPointsInQueue.set(positionHelper.getNumberOfPointsInQueue());
      numberOfPointsInGenerator.set(positionHelper.getNumberOfPointsInGenerator());
      numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());

      statusHelper.updateWithTimeInTrajectory(timeInTrajectory);

      updateGraphics();
   }

   @Override
   public boolean handleTrajectoryCommand(EuclideanTrajectoryControllerCommand command)
   {
      // For a position controlled body the control frame orientation must remain identity so the current orientation can be used to
      // transform the desired from the old control frame position to the new.
      if (command.useCustomControlFrame() && !command.getControlFramePose().getRotationMatrix().isIdentity())
      {
         LogTools.warn("Specifying a control frame orientation for a body position controller is not supported!");
         clear();
         positionHelper.clear();
         return false;
      }

      currentOrientation.setToZero(bodyFrame);
      if (handleCommandInternal(command) && positionHelper.handleTrajectoryCommand(command, currentOrientation))
      {
         usingWeightFromMessage.set(positionHelper.isMessageWeightValid());
         statusHelper.registerNewTrajectory(command);
         return true;
      }

      clear();
      positionHelper.clear();
      return false;
   }

   @Override
   public PointFeedbackControlCommand getFeedbackControlCommand()
   {
      return positionHelper.getFeedbackControlCommand();
   }

   @Override
   public void onExit()
   {
      positionHelper.onExit();
      hideGraphics();
      clear();
   }

   @Override
   public boolean isEmpty()
   {
      return positionHelper.isEmpty();
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      return positionHelper.getLastTrajectoryPointTime();
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
      return statusHelper.pollStatusMessage(positionHelper.getFeedbackControlCommand());
   }
}
