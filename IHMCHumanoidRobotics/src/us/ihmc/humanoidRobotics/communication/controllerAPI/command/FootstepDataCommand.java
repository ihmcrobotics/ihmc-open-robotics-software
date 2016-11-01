package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.controllerAPI.command.Command;
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
   private double swingHeight = 0.0;
   private final Point3d position = new Point3d();
   private final Quat4d orientation = new Quat4d();
   private final RecyclingArrayList<Point2d> predictedContactPoints = new RecyclingArrayList<>(4, Point2d.class);

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
   }

   @Override
   public void set(FootstepDataMessage message)
   {
      robotSide = message.getRobotSide();
      origin = message.getOrigin();
      trajectoryType = message.getTrajectoryType();
      swingHeight = message.getSwingHeight();
      position.set(message.getLocation());
      orientation.set(message.getOrientation());
      ArrayList<Point2d> originalPredictedContactPoints = message.getPredictedContactPoints();
      predictedContactPoints.clear();
      if (originalPredictedContactPoints != null)
      {
         for (int i = 0; i < originalPredictedContactPoints.size(); i++)
            predictedContactPoints.add().set(originalPredictedContactPoints.get(i));
      }
   }

   @Override
   public void set(FootstepDataCommand other)
   {
      robotSide = other.robotSide;
      origin = other.origin;
      trajectoryType = other.trajectoryType;
      swingHeight = other.swingHeight;
      position.set(other.position);
      orientation.set(other.orientation);
      RecyclingArrayList<Point2d> otherPredictedContactPoints = other.predictedContactPoints;
      predictedContactPoints.clear();
      for (int i = 0; i < otherPredictedContactPoints.size(); i++)
         predictedContactPoints.add().set(otherPredictedContactPoints.get(i));
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setPose(Point3d position, Quat4d orientation)
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

   public void setPredictedContactPoints(RecyclingArrayList<Point2d> predictedContactPoints)
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

   public double getSwingHeight()
   {
      return swingHeight;
   }

   public Point3d getPosition()
   {
      return position;
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   public RecyclingArrayList<Point2d> getPredictedContactPoints()
   {
      return predictedContactPoints;
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
