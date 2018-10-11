package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;
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

   private final RigidBodyPositionControlHelper positionHelper;

   public RigidBodyPositionController(RigidBody bodyToControl, RigidBody baseBody, RigidBody elevator, Collection<ReferenceFrame> trajectoryFrames,
                                      ReferenceFrame controlFrame, ReferenceFrame baseFrame, YoDouble yoTime, YoVariableRegistry parentRegistry,
                                      YoGraphicsListRegistry graphicsListRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, bodyToControl.getName(), yoTime, parentRegistry);

      String bodyName = bodyToControl.getName();
      String prefix = bodyName + "TaskspacePosition";

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);

      usingWeightFromMessage = new YoBoolean(prefix + "UsingWeightFromMessage", registry);
      BooleanParameter useBaseFrameForControl = new BooleanParameter(prefix + "UseBaseFrameForControl", registry, false);
      positionHelper = new RigidBodyPositionControlHelper(prefix, bodyToControl, baseBody, elevator, trajectoryFrames, controlFrame, baseFrame,
                                                          useBaseFrameForControl, usingWeightFromMessage, registry, graphicsListRegistry);

      graphics.addAll(positionHelper.getGraphics());
      hideGraphics();
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
      positionHelper.holdCurrentDesired();
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
      positionHelper.goToPosition(position, trajectoryTime);
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

      updateGraphics();
   }

   @Override
   public boolean handleTrajectoryCommand(EuclideanTrajectoryControllerCommand command)
   {
      if (handleCommandInternal(command) && positionHelper.handleTrajectoryCommand(command))
      {
         usingWeightFromMessage.set(positionHelper.isMessageWeightValid());
         return true;
      }

      clear();
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
      clear();
      hideGraphics();
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

   public void clear()
   {
      positionHelper.clear();
      numberOfPointsInQueue.set(0);
      numberOfPointsInGenerator.set(0);
      numberOfPoints.set(0);
      usingWeightFromMessage.set(false);
      trajectoryDone.set(true);
      resetLastCommandId();
   }
}
