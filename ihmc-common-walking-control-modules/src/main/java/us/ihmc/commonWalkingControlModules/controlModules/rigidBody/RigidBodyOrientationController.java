package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class RigidBodyOrientationController extends RigidBodyControlState
{
   private final YoInteger numberOfPointsInQueue;
   private final YoInteger numberOfPointsInGenerator;
   private final YoInteger numberOfPoints;

   private final YoBoolean usingWeightFromMessage;

   private final RigidBodyOrientationControlHelper orientationControlHelper;

   public RigidBodyOrientationController(String postfix, RigidBody bodyToControl, RigidBody baseBody, RigidBody elevator,
                                         Collection<ReferenceFrame> trajectoryFrames, ReferenceFrame controlFrame, ReferenceFrame baseFrame, YoDouble yoTime,
                                         BooleanProvider useBaseFrameForControl, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, bodyToControl.getName() + postfix, yoTime, parentRegistry);

      String bodyName = bodyToControl.getName() + postfix;
      String prefix = bodyName + "TaskspaceOrientation";

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);

      usingWeightFromMessage = new YoBoolean(prefix + "UsingWeightFromMessage", registry);
      orientationControlHelper = new RigidBodyOrientationControlHelper(postfix, prefix, bodyToControl, baseBody, elevator, trajectoryFrames, controlFrame,
                                                                       baseFrame, useBaseFrameForControl, usingWeightFromMessage, registry);
   }

   public void setGains(PID3DGainsReadOnly gains)
   {
      orientationControlHelper.setGains(gains);
   }

   public void setWeights(Vector3DReadOnly weights)
   {
      orientationControlHelper.setWeights(weights);
   }

   public void holdCurrent()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      orientationControlHelper.holdCurrent();
      trajectoryDone.set(false);
   }

   public void holdCurrentDesired()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      orientationControlHelper.holdCurrentDesired();
      trajectoryDone.set(false);
   }

   public void goToOrientationFromCurrent(FrameQuaternionReadOnly orientation, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      orientationControlHelper.goToOrientationFromCurrent(orientation, trajectoryTime);
      trajectoryDone.set(false);
   }

   public void goToOrientation(FrameQuaternionReadOnly orientation, FrameQuaternionReadOnly initialOrientation, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      orientationControlHelper.goToOrientation(orientation, initialOrientation, trajectoryTime);
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
      trajectoryDone.set(orientationControlHelper.doAction(timeInTrajectory));

      numberOfPointsInQueue.set(orientationControlHelper.getNumberOfPointsInQueue());
      numberOfPointsInGenerator.set(orientationControlHelper.getNumberOfPointsInGenerator());
      numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());

      updateGraphics();
   }

   public boolean handleSO3TrajectoryCommand(SO3TrajectoryControllerCommand command, FrameQuaternionReadOnly initialOrientation)
   {
      if (!handleCommandInternal(command))
      {
         return false;
      }

      return orientationControlHelper.handleSO3TrajectoryCommand(command, initialOrientation);
   }

   @Override
   public OrientationFeedbackControlCommand getFeedbackControlCommand()
   {
      return orientationControlHelper.getFeedbackControlCommand();
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
      return orientationControlHelper.isEmpty();
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      return orientationControlHelper.getLastTrajectoryPointTime();
   }

   public void clear()
   {
      orientationControlHelper.clear();
      numberOfPointsInQueue.set(0);
      numberOfPointsInGenerator.set(0);
      numberOfPoints.set(0);
      usingWeightFromMessage.set(false);
      resetLastCommandId();
   }
}
