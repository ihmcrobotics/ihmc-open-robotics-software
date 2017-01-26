package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.walking.AdjustFootstepMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;

public class AdjustFootstepCommand implements Command<AdjustFootstepCommand, AdjustFootstepMessage>
{
   private RobotSide robotSide;
   private FootstepOrigin origin;
   private final Point3d adjustedPosition = new Point3d();
   private final Quat4d adjustedOrientation = new Quat4d();
   private final RecyclingArrayList<Point2d> predictedContactPoints = new RecyclingArrayList<>(4, Point2d.class);

   public AdjustFootstepCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      robotSide = null;
      origin = null;
      adjustedPosition.set(0.0, 0.0, 0.0);
      adjustedOrientation.set(0.0, 0.0, 0.0, 1.0);
      predictedContactPoints.clear();
   }

   @Override
   public void set(AdjustFootstepMessage message)
   {
      robotSide = message.getRobotSide();
      origin = message.getOrigin();
      adjustedPosition.set(message.getLocation());
      adjustedOrientation.set(message.getOrientation());
      List<Point2d> originalPredictedContactPoints = message.getPredictedContactPoints();
      predictedContactPoints.clear();
      if (originalPredictedContactPoints != null)
      {
         for (int i = 0; i < originalPredictedContactPoints.size(); i++)
            predictedContactPoints.add().set(originalPredictedContactPoints.get(i));
      }
   }

   @Override
   public void set(AdjustFootstepCommand other)
   {
      robotSide = other.robotSide;
      origin = other.origin;
      adjustedPosition.set(other.adjustedPosition);
      adjustedOrientation.set(other.adjustedOrientation);
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
      this.adjustedPosition.set(position);
      this.adjustedOrientation.set(orientation);
   }

   public void setOrigin(FootstepOrigin origin)
   {
      this.origin = origin;
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

   public Point3d getPosition()
   {
      return adjustedPosition;
   }

   public Quat4d getOrientation()
   {
      return adjustedOrientation;
   }

   public RecyclingArrayList<Point2d> getPredictedContactPoints()
   {
      return predictedContactPoints;
   }

   @Override
   public Class<AdjustFootstepMessage> getMessageClass()
   {
      return AdjustFootstepMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return origin != null && robotSide != null;
   }
}
