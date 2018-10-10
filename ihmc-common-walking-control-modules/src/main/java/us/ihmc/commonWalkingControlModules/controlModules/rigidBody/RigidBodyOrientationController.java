package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class RigidBodyOrientationController extends RigidBodyControlState
{
   private final YoBoolean usingWeightFromMessage;

   private final YoInteger numberOfPointsInQueue;
   private final YoInteger numberOfPointsInGenerator;
   private final YoInteger numberOfPoints;

   private final RigidBodyOrientationControlHelper orientationHelper;

   public RigidBodyOrientationController(String postfix, RigidBody bodyToControl, RigidBody baseBody, RigidBody elevator,
                                         Collection<ReferenceFrame> trajectoryFrames, ReferenceFrame baseFrame, YoDouble yoTime,
                                         YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, bodyToControl.getName() + postfix, yoTime, parentRegistry);

      String bodyName = bodyToControl.getName() + postfix;
      String prefix = bodyName + "TaskspaceOrientation";

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);

      usingWeightFromMessage = new YoBoolean(prefix + "UsingWeightFromMessage", registry);
      BooleanParameter useBaseFrameForControl = new BooleanParameter(prefix + "UseBaseFrameForControl", registry, false);
      orientationHelper = new RigidBodyOrientationControlHelper(postfix, prefix, bodyToControl, baseBody, elevator, trajectoryFrames, baseFrame,
                                                                       useBaseFrameForControl, usingWeightFromMessage, registry);
   }

   public void setGains(PID3DGainsReadOnly gains)
   {
      orientationHelper.setGains(gains);
   }

   public void setWeights(Vector3DReadOnly weights)
   {
      orientationHelper.setWeights(weights);
   }

   public void holdCurrent()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      orientationHelper.holdCurrent();
   }

   public void holdCurrentDesired()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      orientationHelper.holdCurrentDesired();
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

      updateGraphics();
   }

   public boolean handleTrajectoryCommand(SO3TrajectoryControllerCommand command)
   {
      if (handleCommandInternal(command) && orientationHelper.handleTrajectoryCommand(command))
      {
         usingWeightFromMessage.set(orientationHelper.isMessageWeightValid());
         return true;
      }

      clear();
      return false;
   }

   @Override
   public OrientationFeedbackControlCommand getFeedbackControlCommand()
   {
      return orientationHelper.getFeedbackControlCommand();
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
      return orientationHelper.isEmpty();
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      return orientationHelper.getLastTrajectoryPointTime();
   }

   public void clear()
   {
      orientationHelper.clear();
      numberOfPointsInQueue.set(0);
      numberOfPointsInGenerator.set(0);
      numberOfPoints.set(0);
      usingWeightFromMessage.set(false);
      trajectoryDone.set(true);
      resetLastCommandId();
   }
}
