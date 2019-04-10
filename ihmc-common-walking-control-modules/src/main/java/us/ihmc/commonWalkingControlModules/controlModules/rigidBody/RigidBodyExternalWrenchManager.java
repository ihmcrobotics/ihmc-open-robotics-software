package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WrenchTrajectoryControllerCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.math.trajectories.LinearSpatialVectorTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.LinearSpatialVectorTrajectoryGenerator.SpatialWaypoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class RigidBodyExternalWrenchManager extends RigidBodyControlState
{
   private final ExternalWrenchCommand externalWrenchCommand = new ExternalWrenchCommand();

   private final YoInteger numberOfPointsInQueue;
   private final YoInteger numberOfPointsInGenerator;
   private final YoInteger numberOfPoints;

   private final SpatialVector spatialVector = new SpatialVector();
   private final Wrench desiredWrench = new Wrench();

   private final SpatialWaypoint lastPointAdded = new SpatialWaypoint();
   private final RecyclingArrayDeque<SpatialWaypoint> pointQueue = new RecyclingArrayDeque<>(RigidBodyTaskspaceControlState.maxPoints, SpatialWaypoint.class,
                                                                                             SpatialWaypoint::set);
   private final LinearSpatialVectorTrajectoryGenerator trajectoryGenerator;

   private final RigidBodyBasics bodyToControl;

   private final MovingReferenceFrame bodyFrame;
   private final MovingReferenceFrame baseFrame;

   private final FramePose3DReadOnly defaultControlFramePose;
   private final PoseReferenceFrame activeControlFrame;

   public RigidBodyExternalWrenchManager(RigidBodyBasics bodyToControl, RigidBodyBasics baseBody, ReferenceFrame controlFrame, YoDouble yoTime,
                                         YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(null, bodyToControl.getName() + "Wrench", yoTime, parentRegistry);

      this.bodyToControl = bodyToControl;

      bodyFrame = bodyToControl.getBodyFixedFrame();
      baseFrame = baseBody.getBodyFixedFrame();
      defaultControlFramePose = new FramePose3D(bodyFrame, controlFrame.getTransformToDesiredFrame(bodyFrame));
      activeControlFrame = new PoseReferenceFrame("activeControlFrame", bodyToControl.getBodyFixedFrame());

      String prefix = bodyToControl.getName() + "Wrench";

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);

      trajectoryGenerator = new LinearSpatialVectorTrajectoryGenerator(prefix, RigidBodyTaskspaceControlState.maxPointsInGenerator,
                                                                       ReferenceFrame.getWorldFrame(), registry);
      trajectoryGenerator.clear(baseFrame);
   }

   private void setDefaultControlFrame()
   {
      activeControlFrame.setPoseAndUpdate(defaultControlFramePose);
   }

   private void setControlFramePose(RigidBodyTransform controlFramePoseInBody)
   {
      activeControlFrame.setPoseAndUpdate(controlFramePoseInBody);
   }

   public void getDesiredWrench(WrenchBasics wrenchToPack)
   {
      if (trajectoryGenerator.isEmpty())
      {
         wrenchToPack.setToZero(bodyFrame, bodyFrame);
      }
      else
      {
         spatialVector.setIncludingFrame(trajectoryGenerator.getCurrentValue());
         spatialVector.changeFrame(activeControlFrame);
         wrenchToPack.setIncludingFrame(bodyFrame, spatialVector);
         wrenchToPack.changeFrame(bodyFrame);
      }
   }

   @Override
   public void onEntry()
   {
      clear();
   }

   @Override
   public void onExit()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
      double timeInTrajectory = getTimeInTrajectory();

      boolean done = false;
      if (trajectoryGenerator.isDone() || trajectoryGenerator.getLastWaypointTime() <= timeInTrajectory)
         done = fillAndReinitializeTrajectories();

      if (!done)
      {
         trajectoryGenerator.compute(timeInTrajectory);
         getDesiredWrench(desiredWrench);
         desiredWrench.negate(); // Changing from desired wrench to external wrench to compensate for.
         externalWrenchCommand.set(bodyToControl, desiredWrench);
      }

      trajectoryDone.set(done);
      numberOfPointsInQueue.set(getNumberOfPointsInQueue());
      numberOfPointsInGenerator.set(getNumberOfPointsInGenerator());
      numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());

      updateGraphics();
   }

   private boolean fillAndReinitializeTrajectories()
   {
      if (pointQueue.isEmpty())
      {
         return true;
      }

      if (!trajectoryGenerator.isEmpty())
      {
         trajectoryGenerator.clear();
         lastPointAdded.changeFrame(trajectoryGenerator.getCurrentTrajectoryFrame());
         trajectoryGenerator.appendWaypoint(lastPointAdded);
      }

      int currentNumberOfWaypoints = trajectoryGenerator.getCurrentNumberOfWaypoints();
      int pointsToAdd = RigidBodyTaskspaceControlState.maxPointsInGenerator - currentNumberOfWaypoints;
      for (int pointIdx = 0; pointIdx < pointsToAdd; pointIdx++)
      {
         if (pointQueue.isEmpty())
            break;

         SpatialWaypoint pointToAdd = pointQueue.pollFirst();
         lastPointAdded.setIncludingFrame(pointToAdd);
         trajectoryGenerator.appendWaypoint(pointToAdd);
      }

      trajectoryGenerator.initialize();
      return false;
   }

   public boolean handleWrenchTrajectoryCommand(WrenchTrajectoryControllerCommand command)
   {
      if (!handleCommandInternal(command) || command.getNumberOfTrajectoryPoints() == 0)
      {
         clear();
         return false;
      }

      if (command.getExecutionMode() == ExecutionMode.OVERRIDE || isEmpty())
      {
         // Record the current desired wrench.
         getDesiredWrench(desiredWrench);

         clear();

         // Set the new control frame and move the desired position to be for that frame.
         if (command.useCustomControlFrame())
         {
            setControlFramePose(command.getControlFramePose());
         }

         trajectoryGenerator.changeFrame(command.getTrajectoryFrame());

         if (command.getTrajectoryPointTime(0) > RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint)
         {
            desiredWrench.changeFrame(activeControlFrame);
            queueInitialPoint(desiredWrench);
         }
      }
      else if (command.getTrajectoryFrame() != trajectoryGenerator.getCurrentTrajectoryFrame())
      {
         LogTools.warn(warningPrefix + "Was executing in " + trajectoryGenerator.getCurrentTrajectoryFrame() + " can not switch to "
               + command.getTrajectoryFrame() + " without override.");
         return false;
      }

      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         double trajectoryPointTime = command.getTrajectoryPointTime(i);
         if (!checkTime(trajectoryPointTime))
            return false;
         SpatialVector trajectoryPoint = command.getTrajectoryPoint(i);
         trajectoryPoint.changeFrame(trajectoryGenerator.getCurrentTrajectoryFrame());
         if (!queuePoint(trajectoryPointTime, trajectoryPoint))
            return false;
      }

      return true;
   }

   public int getNumberOfPointsInQueue()
   {
      return pointQueue.size();
   }

   public int getNumberOfPointsInGenerator()
   {
      return trajectoryGenerator.getCurrentNumberOfWaypoints();
   }

   private void queueInitialPoint(WrenchReadOnly initialWrench)
   {
      SpatialWaypoint initialPoint = pointQueue.addLast();
      initialPoint.setIncludingFrame(initialWrench);
      initialPoint.changeFrame(trajectoryGenerator.getCurrentTrajectoryFrame());
      initialPoint.setTime(0.0);
   }

   private boolean queuePoint(double time, SpatialVectorReadOnly trajectoryPoint)
   {
      if (pointQueue.size() >= RigidBodyTaskspaceControlState.maxPoints)
      {
         LogTools.warn(warningPrefix + "Reached maximum capacity of " + RigidBodyTaskspaceControlState.maxPoints + " can not execute trajectory.");
         return false;
      }

      pointQueue.addLast().setIncludingFrame(time, trajectoryPoint);
      return true;
   }

   private boolean checkTime(double time)
   {
      if (time <= getLastTrajectoryPointTime())
      {
         LogTools.warn(warningPrefix + "Time in trajectory must be strictly increasing.");
         return false;
      }
      return true;
   }

   @Override
   public boolean isEmpty()
   {
      if (!pointQueue.isEmpty())
      {
         return false;
      }
      return trajectoryGenerator.isDone();
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      if (isEmpty())
      {
         return Double.NEGATIVE_INFINITY;
      }
      else if (pointQueue.isEmpty())
      {
         return trajectoryGenerator.getLastWaypointTime();
      }
      else
      {
         return pointQueue.peekLast().getTime();
      }
   }

   public void clear()
   {
      numberOfPointsInQueue.set(0);
      numberOfPointsInGenerator.set(0);
      numberOfPoints.set(0);
      trajectoryDone.set(true);
      resetLastCommandId();

      trajectoryGenerator.clear(baseFrame);
      setDefaultControlFrame();
      pointQueue.clear();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      if (trajectoryDone.getValue())
         return null;
      else
         return externalWrenchCommand;
   }
}
