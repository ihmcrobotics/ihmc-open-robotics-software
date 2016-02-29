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

public class ModifiableFootstepDataMessage
{
   private RobotSide robotSide;
   private FootstepOrigin origin;
   public TrajectoryType trajectoryType = TrajectoryType.DEFAULT;
   public double swingHeight = 0;
   private final Point3d position = new Point3d();
   private final Quat4d orientation = new Quat4d();
   private final RecyclingArrayList<Point2d> predictedContactPoints = new RecyclingArrayList<>(4, Point2d.class);

   public ModifiableFootstepDataMessage()
   {
      predictedContactPoints.clear();
   }

   public void set(FootstepDataMessage footstepDataMessage)
   {
      robotSide = footstepDataMessage.getRobotSide();
      origin = footstepDataMessage.getOrigin();
      trajectoryType = footstepDataMessage.getTrajectoryType();
      swingHeight = footstepDataMessage.getSwingHeight();
      position.set(footstepDataMessage.getLocation());
      orientation.set(footstepDataMessage.getOrientation());
      ArrayList<Point2d> originalPredictedContactPoints = footstepDataMessage.getPredictedContactPoints();
      predictedContactPoints.clear();
      if (originalPredictedContactPoints != null)
      {
         for (int i = 0; i < originalPredictedContactPoints.size(); i++)
            predictedContactPoints.add().set(originalPredictedContactPoints.get(i));
      }
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
}
