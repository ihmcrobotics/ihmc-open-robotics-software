package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.AdjustFootstepMessage;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class AdjustFootstepCommand implements Command<AdjustFootstepCommand, AdjustFootstepMessage>, FrameBasedCommand<AdjustFootstepMessage>
{
   private RobotSide robotSide;
   private final FramePoint adjustedPosition = new FramePoint();
   private final FrameOrientation adjustedOrientation = new FrameOrientation();
   private final RecyclingArrayList<Point2D> predictedContactPoints = new RecyclingArrayList<>(4, Point2D.class);

   private ReferenceFrame trajectoryFrame;

   public AdjustFootstepCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      robotSide = null;
      adjustedPosition.set(0.0, 0.0, 0.0);
      adjustedOrientation.set(0.0, 0.0, 0.0, 1.0);
      predictedContactPoints.clear();
   }

   @Override
   public void set(AdjustFootstepMessage message)
   {
      robotSide = message.getRobotSide();
      adjustedPosition.setIncludingFrame(trajectoryFrame, message.getLocation());
      adjustedOrientation.setIncludingFrame(trajectoryFrame, message.getOrientation());
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
      adjustedPosition.set(other.adjustedPosition);
      adjustedOrientation.set(other.adjustedOrientation);
      RecyclingArrayList<Point2D> otherPredictedContactPoints = other.predictedContactPoints;
      predictedContactPoints.clear();
      for (int i = 0; i < otherPredictedContactPoints.size(); i++)
         predictedContactPoints.add().set(otherPredictedContactPoints.get(i));
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, AdjustFootstepMessage message)
   {
      trajectoryFrame = resolver.getReferenceFrameFromNameBaseHashCode(message.getTrajectoryReferenceFrameId());
      set(message);
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setPose(Point3D position, Quaternion orientation)
   {
      adjustedPosition.set(position);
      adjustedOrientation.set(orientation);
   }

   public void setPredictedContactPoints(RecyclingArrayList<Point2D> predictedContactPoints)
   {
      this.predictedContactPoints.clear();
      for (int i = 0; i < predictedContactPoints.size(); i++)
         this.predictedContactPoints.add().set(predictedContactPoints.get(i));
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public FramePoint getPosition()
   {
      return adjustedPosition;
   }

   public FrameOrientation getOrientation()
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
      return robotSide != null;
   }

}
