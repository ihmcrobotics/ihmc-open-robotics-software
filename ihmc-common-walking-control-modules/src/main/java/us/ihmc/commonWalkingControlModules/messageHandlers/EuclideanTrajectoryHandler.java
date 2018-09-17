package us.ihmc.commonWalkingControlModules.messageHandlers;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleEuclideanTrajectoryPoint;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Helper class to handle the queuing and interpolating of {@link EuclideanTrajectoryControllerCommand} trajectories.
 *
 * @author Georg Wiedebach
 */
public class EuclideanTrajectoryHandler
{
   private static final int defaultMaxNumberOfPoints = 1000;

   private final YoVariableRegistry registry;

   private final RecyclingArrayList<SimpleEuclideanTrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(defaultMaxNumberOfPoints,
                                                                                                                SimpleEuclideanTrajectoryPoint.class);

   private final DoubleProvider yoTime;
   private final YoPolynomial polynomial;
   private final YoFramePoint3D position;
   private final YoFrameVector3D velocity;
   private final YoFrameVector3D acceleration;
   private final YoInteger numberOfPoints;

   private long lastMessageID = -1L;

   /**
    * Create a new {@link EuclideanTrajectoryHandler}.
    *
    * @param name is a prefix for any {@link YoVariable} created in this class.
    * @param yoTime provider of the current time.
    * @param parentRegistry registry that the registry of this class will be attached to.
    */
   public EuclideanTrajectoryHandler(String name, DoubleProvider yoTime, YoVariableRegistry parentRegistry)
   {
      this.yoTime = yoTime;

      registry = new YoVariableRegistry(name + "TrajectoryHandler");
      polynomial = new YoPolynomial(name + "CubicPolynomial", 4, registry);
      position = new YoFramePoint3D(name + "Position", ReferenceFrame.getWorldFrame(), registry);
      velocity = new YoFrameVector3D(name + "Velocity", ReferenceFrame.getWorldFrame(), registry);
      acceleration = new YoFrameVector3D(name + "Acceleration", ReferenceFrame.getWorldFrame(), registry);
      numberOfPoints = new YoInteger(name + "NumberOfPoints", registry);

      trajectoryPoints.clear();

      parentRegistry.addChild(registry);
   }

   protected FramePoint3DReadOnly getPosition()
   {
      return position;
   }

   protected FrameVector3DReadOnly getVelocity()
   {
      return velocity;
   }

   protected FrameVector3DReadOnly getAcceleration()
   {
      return acceleration;
   }

   protected double getCurrentTime()
   {
      return yoTime.getValue();
   }

   protected void handleTrajectory(EuclideanTrajectoryControllerCommand command)
   {
      switch (command.getExecutionMode())
      {
      case OVERRIDE:
         command.addTimeOffset(yoTime.getValue());
         trajectoryPoints.clear();
         break;
      case QUEUE:
         if (trajectoryPoints.isEmpty())
         {
            PrintTools.warn("Can not queue without points");
            return;
         }
         if (command.getTrajectoryPoint(0).getTime() <= 0.0)
         {
            PrintTools.warn("Can not queue trajectory with initial time 0.0");
            return;
         }
         if (command.getPreviousCommandId() != lastMessageID)
         {
            PrintTools.warn("Invalid message ID.");
            return;
         }
         double lastTime = trajectoryPoints.getLast().getTime();
         command.addTimeOffset(lastTime);
         break;
      default:
         throw new RuntimeException("Unhadled execution mode.");
      }

      lastMessageID = command.getCommandId();

      for (int idx = 0; idx < command.getNumberOfTrajectoryPoints(); idx++)
      {
         trajectoryPoints.add().set(command.getTrajectoryPoint(idx));
      }

      numberOfPoints.set(trajectoryPoints.size());
   }

   /**
    * Checks whether the provided time is within the trajectory maintained in this class. If this returns {@code true} it
    * is safe to compute the trajectory at the provided time.
    *
    * @param time of interest
    * @return whether {@code time} is contained in the time interval of this trajectory
    */
   public boolean isWithinInterval(double time)
   {
      if (trajectoryPoints.size() < 2)
      {
         return false;
      }
      if (time < trajectoryPoints.get(0).getTime())
      {
         return false;
      }
      if (time > trajectoryPoints.getLast().getTime())
      {
         return false;
      }
      return true;
   }

   /**
    * Will clear all trajectory points from this handler that are in the past. Call this regularly to avoid trajectory
    * point build up. However, when you are computing the trajectory for non-increasing values of time avoid calling this
    * method.
    */
   public void clearPointsInPast()
   {
      double currentTime = yoTime.getValue();
      if (trajectoryPoints.size() != 0 && trajectoryPoints.getLast().getTime() < currentTime)
      {
         trajectoryPoints.clear();
         numberOfPoints.set(0);
         return;
      }

      while (trajectoryPoints.size() > 1 && trajectoryPoints.get(1).getTime() < currentTime)
      {
         trajectoryPoints.remove(0);
      }
      numberOfPoints.set(trajectoryPoints.size());
   }

   protected void packDesiredsAtTime(double time)
   {
      int endIndex = 0;
      while (trajectoryPoints.get(endIndex).getTime() < time)
      {
         endIndex++;
      }

      if (endIndex == 0)
      {
         endIndex++;
      }

      SimpleEuclideanTrajectoryPoint startPoint = trajectoryPoints.get(endIndex - 1);
      SimpleEuclideanTrajectoryPoint endPoint = trajectoryPoints.get(endIndex);

      double t0 = startPoint.getTime();
      double t1 = endPoint.getTime();

      for (int i = 0; i < 3; i++)
      {
         double p0 = startPoint.getEuclideanWaypoint().getPosition().getElement(i);
         double v0 = startPoint.getEuclideanWaypoint().getLinearVelocity().getElement(i);
         double p1 = endPoint.getEuclideanWaypoint().getPosition().getElement(i);
         double v1 = endPoint.getEuclideanWaypoint().getLinearVelocity().getElement(i);

         polynomial.setCubic(t0, t1, p0, v0, p1, v1);
         polynomial.compute(time);

         position.setElement(i, polynomial.getPosition());
         velocity.setElement(i, polynomial.getVelocity());
         acceleration.setElement(i, polynomial.getAcceleration());
      }
   }
}
