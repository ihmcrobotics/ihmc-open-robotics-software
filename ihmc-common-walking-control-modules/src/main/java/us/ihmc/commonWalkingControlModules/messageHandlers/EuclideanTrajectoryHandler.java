package us.ihmc.commonWalkingControlModules.messageHandlers;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingIterator;
import us.ihmc.commons.lists.RecyclingLinkedList;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.math.trajectories.trajectorypoints.EuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Helper class to handle the queuing and interpolating of {@link EuclideanTrajectoryControllerCommand} trajectories.
 *
 * @author Georg Wiedebach
 */
public class EuclideanTrajectoryHandler
{
   private static final int defaultMaxNumberOfPoints = 1000;

   private final YoRegistry registry;

   private final EuclideanTrajectoryPointBasics lastPoint = new EuclideanTrajectoryPoint();
   private final EuclideanTrajectoryPointBasics firstPoint = new EuclideanTrajectoryPoint();
   private final EuclideanTrajectoryPointBasics secondPoint = new EuclideanTrajectoryPoint();
   private final EuclideanTrajectoryPointBasics tempPoint = new EuclideanTrajectoryPoint();
   private final Point3D integratedPosition = new Point3D();

   private final RecyclingLinkedList<EuclideanTrajectoryPointBasics> trajectoryPoints = new RecyclingLinkedList<>(defaultMaxNumberOfPoints,
                                                                                                                     EuclideanTrajectoryPoint::new,
                                                                                                                     EuclideanTrajectoryPointBasics::set);
   private final RecyclingIterator<EuclideanTrajectoryPointBasics> trajectoryIterator = trajectoryPoints.createForwardIterator();

   private final DoubleProvider yoTime;
   private final YoPolynomial polynomial;
   private final YoFramePoint3D position;
   private final YoFrameVector3D velocity;
   private final YoFrameVector3D acceleration;
   private final YoInteger numberOfPoints;
   /**
    * Used when streaming to account for time variations occurring during the transport of the message
    * over the network.
    */
   private final YoDouble streamTimestampOffset;
   private final YoDouble streamTimestampSource;

   private long lastMessageID = -1L;

   /**
    * Create a new {@link EuclideanTrajectoryHandler}.
    *
    * @param name is a prefix for any {@link YoVariable} created in this class.
    * @param yoTime provider of the current time.
    * @param parentRegistry registry that the registry of this class will be attached to.
    */
   public EuclideanTrajectoryHandler(String name, DoubleProvider yoTime, YoRegistry parentRegistry)
   {
      this.yoTime = yoTime;

      registry = new YoRegistry(name + "TrajectoryHandler");
      polynomial = new YoPolynomial(name + "CubicPolynomial", 4, registry);
      position = new YoFramePoint3D(name + "Position", ReferenceFrame.getWorldFrame(), registry);
      velocity = new YoFrameVector3D(name + "Velocity", ReferenceFrame.getWorldFrame(), registry);
      acceleration = new YoFrameVector3D(name + "Acceleration", ReferenceFrame.getWorldFrame(), registry);
      numberOfPoints = new YoInteger(name + "NumberOfPoints", registry);

      streamTimestampOffset = new YoDouble(name + "StreamTimestampOffset", registry);
      streamTimestampOffset.setToNaN();
      streamTimestampSource = new YoDouble(name + "StreamTimestampSource", registry);
      streamTimestampSource.setToNaN();

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
      double streamTimeOffset = 0.0;
      double streamTimestampOffset = this.streamTimestampOffset.getValue();
      double streamTimestampSource = this.streamTimestampSource.getValue();

      if (command.getExecutionMode() == ExecutionMode.STREAM)
      { // Need to do time checks before moving on.
         if (command.getTimestamp() <= 0 || !isWithinInterval(yoTime.getValue()))
         {
            streamTimestampOffset = Double.NaN;
            streamTimestampSource = Double.NaN;
         }
         else
         {
            double senderTime = Conversions.nanosecondsToSeconds(command.getTimestamp());

            if (!Double.isNaN(streamTimestampSource) && senderTime < streamTimestampSource)
            {
               // Messages are out of order which is fine, we just don't want to handle the new message.
               return;
            }

            streamTimestampSource = senderTime;

            streamTimeOffset = yoTime.getValue() - senderTime;

            if (Double.isNaN(streamTimestampOffset))
            {
               streamTimestampOffset = streamTimeOffset;
            }
            else
            {
               /*
                * Update to the smallest time offset, which is closer to the true offset between the sender CPU and
                * control CPU. If the change in offset is too large though, we always set the streamTimestampOffset
                * for safety.
                */
               if (Math.abs(streamTimeOffset - streamTimestampOffset) > 0.5)
                  streamTimestampOffset = streamTimeOffset;
               else
                  streamTimestampOffset = Math.min(streamTimeOffset, streamTimestampOffset);
            }
         }
      }

      switch (command.getExecutionMode())
      {
      case OVERRIDE:
         command.addTimeOffset(yoTime.getValue());
         while (!trajectoryPoints.isEmpty())
         {
            trajectoryPoints.removeFirst();
         }
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
         trajectoryPoints.peekLast(lastPoint);
         double lastTime = lastPoint.getTime();
         command.addTimeOffset(lastTime);
         break;
      case STREAM:
         {
            this.streamTimestampOffset.set(streamTimestampOffset);
            this.streamTimestampSource.set(streamTimestampSource);

            int numberOfTrajectoryPoints = command.getNumberOfTrajectoryPoints();
            if (command.getNumberOfTrajectoryPoints() == 0 || numberOfTrajectoryPoints > 2)
            {
               LogTools.warn("When streaming, trajectories should contain either 1 or 2 trajectory point(s), was: " + command.getNumberOfTrajectoryPoints());
               return;
            }

            FrameEuclideanTrajectoryPointList streamingTrajectoryPoints = command.getTrajectoryPointList();

            FrameEuclideanTrajectoryPoint streamingFirstPoint = streamingTrajectoryPoints.getTrajectoryPoint(0);

            if (streamingFirstPoint.getTime() != 0.0)
            {
               LogTools.warn("When streaming, the trajectory point should have a time of zero, was: " + streamingFirstPoint.getTime());
               return;
            }

            this.firstPoint.set(streamingFirstPoint);
            firstPoint.setTime(yoTime.getValue());

            if (!Double.isNaN(streamTimestampOffset))
               this.firstPoint.addTimeOffset(streamTimestampOffset - streamTimeOffset);

            if (streamingTrajectoryPoints.getNumberOfTrajectoryPoints() == 1)
            { // No extrapolation provided, so we do it here.
               this.secondPoint.set(firstPoint);
               integratedPosition.scaleAdd(command.getStreamIntegrationDuration(), secondPoint.getLinearVelocity(), secondPoint.getPosition());
               secondPoint.getPosition().set(integratedPosition);
            }
            else
            {
               FrameEuclideanTrajectoryPoint secondPoint = streamingTrajectoryPoints.getTrajectoryPoint(1);

               if (secondPoint.getTime() != command.getStreamIntegrationDuration())
               {
                  LogTools.warn("When streaming, the second trajectory point should have a time equal to the integration duration, was: " + secondPoint.getTime());
                  return;
               }

               this.secondPoint.set(secondPoint);
            }

            this.secondPoint.setTime(command.getStreamIntegrationDuration() + this.firstPoint.getTime());

            while (!trajectoryPoints.isEmpty())
               trajectoryPoints.removeFirst();

            trajectoryPoints.addLast(firstPoint);
            trajectoryPoints.addLast(secondPoint);

            break;
         }
      default:
         throw new RuntimeException("Unhadled execution mode.");
      }

      lastMessageID = command.getCommandId();

      if (command.getExecutionMode() != ExecutionMode.STREAM)
      {
         for (int idx = 0; idx < command.getNumberOfTrajectoryPoints(); idx++)
         {
            trajectoryPoints.addLast(command.getTrajectoryPoint(idx));
         }

         numberOfPoints.set(trajectoryPoints.size());
      }
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
      trajectoryPoints.peekFirst(firstPoint);
      if (time < firstPoint.getTime())
      {
         return false;
      }
      trajectoryPoints.peekLast(lastPoint);
      if (time > lastPoint.getTime())
      {
         return false;
      }
      return true;
   }

   /**
    * @return true if there are no trajectory points currently stored
    */
   public boolean isEmpty()
   {
      return trajectoryPoints.isEmpty();
   }

   /**
    * Will clear all trajectory points from this handler that are in the past. Call this regularly to avoid trajectory
    * point build up. However, when you are computing the trajectory for non-increasing values of time avoid calling this
    * method.
    */
   public void clearPointsInPast()
   {
      // Remove points until we find one that is in the future but always remember the last removed one.
      double currentTime = yoTime.getValue();
      boolean removedPoints = false;
      while (!trajectoryPoints.isEmpty())
      {
         trajectoryPoints.peekFirst(firstPoint);
         if (firstPoint.getTime() < currentTime)
         {
            tempPoint.set(firstPoint);
            trajectoryPoints.removeFirst();
            removedPoints = true;
         }
         else
         {
            break;
         }
      }

      // If we removed points and still have at least one point in the list of points that is in the future we
      // need to add the last removed point back so we can interpolate the current time.
      if (removedPoints && !trajectoryPoints.isEmpty())
      {
         trajectoryPoints.addFirst(tempPoint);
      }

      numberOfPoints.set(trajectoryPoints.size());
   }

   protected void packDesiredsAtTime(double time)
   {
      trajectoryIterator.reset();

      // This should not be called with a time outside the range of the trajectory so no need for safety.
      trajectoryIterator.next(firstPoint);
      while (trajectoryIterator.hasNext())
      {
         trajectoryIterator.next(secondPoint);
         if (secondPoint.getTime() >= time)
         {
            break;
         }
         firstPoint.set(secondPoint);
      }

      double t0 = firstPoint.getTime();
      double t1 = secondPoint.getTime();

      for (int i = 0; i < 3; i++)
      {
         double p0 = firstPoint.getPosition().getElement(i);
         double v0 = firstPoint.getLinearVelocity().getElement(i);
         double p1 = secondPoint.getPosition().getElement(i);
         double v1 = secondPoint.getLinearVelocity().getElement(i);

         polynomial.setCubic(t0, t1, p0, v0, p1, v1);
         polynomial.compute(time);

         position.setElement(i, polynomial.getValue());
         velocity.setElement(i, polynomial.getVelocity());
         acceleration.setElement(i, polynomial.getAcceleration());
      }
   }
}
