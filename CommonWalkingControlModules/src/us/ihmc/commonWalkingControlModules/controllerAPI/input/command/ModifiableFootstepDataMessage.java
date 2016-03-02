package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class ModifiableFootstepDataMessage implements ControllerMessage<ModifiableFootstepDataMessage, FootstepDataMessage>
{
   private RobotSide robotSide;
   private FootstepOrigin origin;
   private TrajectoryType trajectoryType = TrajectoryType.DEFAULT;
   private double swingHeight = 0;
   private final Point3d position = new Point3d();
   private final Quat4d orientation = new Quat4d();
   private final RecyclingArrayList<Point2d> predictedContactPoints = new RecyclingArrayList<>(4, Point2d.class);

   public ModifiableFootstepDataMessage()
   {
      clearPredictedContactPoints();
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
   public void set(ModifiableFootstepDataMessage other)
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

   public void clearPredictedContactPoints()
   {
      predictedContactPoints.clear();
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
}
