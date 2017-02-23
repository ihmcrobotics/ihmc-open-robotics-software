package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.AdjustFootstepMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;

public class AdjustFootstepCommand implements Command<AdjustFootstepCommand, AdjustFootstepMessage>
{
   private RobotSide robotSide;
   private FootstepOrigin origin;
   private final Point3D adjustedPosition = new Point3D();
   private final Quaternion adjustedOrientation = new Quaternion();
   private final RecyclingArrayList<Point2D> predictedContactPoints = new RecyclingArrayList<>(4, Point2D.class);

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
      List<Point2D> originalPredictedContactPoints = message.getPredictedContactPoints();
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
      RecyclingArrayList<Point2D> otherPredictedContactPoints = other.predictedContactPoints;
      predictedContactPoints.clear();
      for (int i = 0; i < otherPredictedContactPoints.size(); i++)
         predictedContactPoints.add().set(otherPredictedContactPoints.get(i));
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setPose(Point3D position, Quaternion orientation)
   {
      this.adjustedPosition.set(position);
      this.adjustedOrientation.set(orientation);
   }

   public void setOrigin(FootstepOrigin origin)
   {
      this.origin = origin;
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

   public Point3D getPosition()
   {
      return adjustedPosition;
   }

   public Quaternion getOrientation()
   {
      return adjustedOrientation;
   }

   public RecyclingArrayList<Point2D> getPredictedContactPoints()
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
