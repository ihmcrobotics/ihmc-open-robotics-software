package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.ArrayList;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class FootstepDataCommand implements Command<FootstepDataCommand, FootstepDataMessage>
{
   private RobotSide robotSide;
   private FootstepOrigin origin;
   private TrajectoryType trajectoryType = TrajectoryType.DEFAULT;
   private final RecyclingArrayList<Point3D> trajectoryWaypoints = new RecyclingArrayList<>(2, Point3D.class);
   private double swingHeight = 0.0;
   private final Point3D position = new Point3D();
   private final Quaternion orientation = new Quaternion();
   private final RecyclingArrayList<Point2D> predictedContactPoints = new RecyclingArrayList<>(4, Point2D.class);

   private boolean hasTimings = false;
   private double swingTime = Double.NaN;
   private double transferTime = Double.NaN;

   private boolean hasAbsoluteTime = false;
   private double swingStartTime = Double.NaN;

   public FootstepDataCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      robotSide = null;
      origin = null;
      trajectoryType = TrajectoryType.DEFAULT;
      swingHeight = 0.0;
      position.set(0.0, 0.0, 0.0);
      orientation.set(0.0, 0.0, 0.0, 1.0);
      predictedContactPoints.clear();
      trajectoryWaypoints.clear();

      hasTimings = false;
      swingTime = Double.NaN;
      transferTime = Double.NaN;

      hasAbsoluteTime = false;
      swingStartTime = Double.NaN;
   }

   @Override
   public void set(FootstepDataMessage message)
   {
      robotSide = message.getRobotSide();
      origin = message.getOrigin();
      trajectoryType = message.getTrajectoryType();
      Point3D[] originalWaypointList = message.getTrajectoryWaypoints();
      trajectoryWaypoints.clear();
      if (originalWaypointList != null)
      {
         for (int i = 0; i < originalWaypointList.length; i++)
            trajectoryWaypoints.add().set(originalWaypointList[i]);
      }
      swingHeight = message.getSwingHeight();
      position.set(message.getLocation());
      orientation.set(message.getOrientation());
      ArrayList<Point2D> originalPredictedContactPoints = message.getPredictedContactPoints();
      predictedContactPoints.clear();
      if (originalPredictedContactPoints != null)
      {
         for (int i = 0; i < originalPredictedContactPoints.size(); i++)
            predictedContactPoints.add().set(originalPredictedContactPoints.get(i));
      }

      hasTimings = message.hasTimings;
      swingTime = message.swingDuration;
      transferTime = message.transferDuration;

      hasAbsoluteTime = message.hasAbsoluteTime;
      swingStartTime = message.swingStartTime;
   }

   @Override
   public void set(FootstepDataCommand other)
   {
      robotSide = other.robotSide;
      origin = other.origin;
      trajectoryType = other.trajectoryType;
      RecyclingArrayList<Point3D> otherWaypointList = other.trajectoryWaypoints;
      trajectoryWaypoints.clear();
      for (int i = 0; i < otherWaypointList.size(); i++)
         trajectoryWaypoints.add().set(otherWaypointList.get(i));
      swingHeight = other.swingHeight;
      position.set(other.position);
      orientation.set(other.orientation);
      RecyclingArrayList<Point2D> otherPredictedContactPoints = other.predictedContactPoints;
      predictedContactPoints.clear();
      for (int i = 0; i < otherPredictedContactPoints.size(); i++)
         predictedContactPoints.add().set(otherPredictedContactPoints.get(i));

      hasTimings = other.hasTimings;
      swingTime = other.swingTime;
      transferTime = other.transferTime;

      hasAbsoluteTime = other.hasAbsoluteTime;
      swingStartTime = other.swingStartTime;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setPose(Point3D position, Quaternion orientation)
   {
      this.position.set(position);
      this.orientation.set(orientation);
   }

   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight = swingHeight;
   }

   public void setOrigin(FootstepOrigin origin)
   {
      this.origin = origin;
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType = trajectoryType;
   }

   public void setPredictedContactPoints(RecyclingArrayList<Point2D> predictedContactPoints)
   {
      this.predictedContactPoints.clear();
      for(int i = 0; i < predictedContactPoints.size(); i++)
         this.predictedContactPoints.add().set(predictedContactPoints.get(i));
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public FootstepOrigin getOrigin()
   {
      return origin;
   }

   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }

   public RecyclingArrayList<Point3D> getTrajectoryWaypoints()
   {
      return trajectoryWaypoints;
   }

   public double getSwingHeight()
   {
      return swingHeight;
   }

   public Point3D getPosition()
   {
      return position;
   }

   public Quaternion getOrientation()
   {
      return orientation;
   }

   public RecyclingArrayList<Point2D> getPredictedContactPoints()
   {
      return predictedContactPoints;
   }

   public boolean hasTimings()
   {
      return hasTimings;
   }

   public double getSwingTime()
   {
      return swingTime;
   }

   public double getTransferTime()
   {
      return transferTime;
   }

   public boolean hasAbsoluteTime()
   {
      return hasAbsoluteTime;
   }

   public double getSwingStartTime()
   {
      return swingStartTime;
   }

   @Override
   public Class<FootstepDataMessage> getMessageClass()
   {
      return FootstepDataMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return origin != null && robotSide != null;
   }
}
