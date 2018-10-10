package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class RigidBodyPositionController extends RigidBodyControlState
{
   private final YoInteger numberOfPointsInQueue;
   private final YoInteger numberOfPointsInGenerator;
   private final YoInteger numberOfPoints;

   private final YoBoolean usingWeightFromMessage;

   private final RigidBodyPositionControlHelper positionControlHelper;

   public RigidBodyPositionController(String postfix, RigidBody bodyToControl, RigidBody baseBody, RigidBody elevator,
                                      Collection<ReferenceFrame> trajectoryFrames, ReferenceFrame controlFrame, ReferenceFrame baseFrame, YoDouble yoTime,
                                      BooleanProvider useBaseFrameForControl, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, bodyToControl.getName() + postfix, yoTime, parentRegistry);

      String bodyName = bodyToControl.getName() + postfix;
      String prefix = bodyName + "TaskspacePosition";

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);

      usingWeightFromMessage = new YoBoolean(prefix + "UsingWeightFromMessage", registry);
      positionControlHelper = new RigidBodyPositionControlHelper(postfix, prefix, bodyToControl, baseBody, elevator, trajectoryFrames, controlFrame, baseFrame,
                                                                 useBaseFrameForControl, usingWeightFromMessage, registry, graphicsListRegistry);

      graphics.addAll(positionControlHelper.getGraphics());
      hideGraphics();
   }

   public void setGains(PID3DGainsReadOnly gains)
   {
      positionControlHelper.setGains(gains);
   }

   public void setWeights(Vector3DReadOnly weights)
   {
      positionControlHelper.setWeights(weights);
   }

   public void setDefaultControlFrame()
   {
      positionControlHelper.setDefaultControlFrame();
   }

   public void setControlFramePosition(Tuple3DReadOnly positionInBodyFrame)
   {
      positionControlHelper.setControlFramePosition(positionInBodyFrame);
   }

   public void holdCurrent()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionControlHelper.holdCurrent();
      trajectoryDone.set(false);
   }

   public void holdCurrentDesired()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionControlHelper.holdCurrentDesired();
      trajectoryDone.set(false);
   }

   public void goToPositionFromCurrent(FramePoint3DReadOnly position, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionControlHelper.goToPositionFromCurrent(position, trajectoryTime);
      trajectoryDone.set(false);
   }

   public void goToPosition(FramePoint3DReadOnly position, FramePoint3DReadOnly initialPosition, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      positionControlHelper.goToPosition(position, initialPosition, trajectoryTime);
      trajectoryDone.set(false);
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
      double timeInTrajectory = getTimeInTrajectory();
      trajectoryDone.set(positionControlHelper.doAction(timeInTrajectory));

      numberOfPointsInQueue.set(positionControlHelper.getNumberOfPointsInQueue());
      numberOfPointsInGenerator.set(positionControlHelper.getNumberOfPointsInGenerator());
      numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());

      updateGraphics();
   }

   public boolean handleEuclideanTrajectoryCommand(EuclideanTrajectoryControllerCommand command, FramePoint3D initialPosition)
   {
      if (!handleCommandInternal(command))
      {
         return false;
      }

      return positionControlHelper.handleEuclideanTrajectoryCommand(command, initialPosition);
   }

   @Override
   public PointFeedbackControlCommand getFeedbackControlCommand()
   {
      return positionControlHelper.getFeedbackControlCommand();
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
      return positionControlHelper.isEmpty();
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      return positionControlHelper.getLastTrajectoryPointTime();
   }

   public void clear()
   {
      positionControlHelper.clear();
      numberOfPointsInQueue.set(0);
      numberOfPointsInGenerator.set(0);
      numberOfPoints.set(0);
      usingWeightFromMessage.set(false);
      resetLastCommandId();
   }
}
