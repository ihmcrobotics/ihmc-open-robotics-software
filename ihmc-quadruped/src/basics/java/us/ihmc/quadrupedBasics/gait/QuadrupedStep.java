package us.ihmc.quadrupedBasics.gait;

import quadruped_msgs.msg.dds.QuadrupedStepMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedStepCommand;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class QuadrupedStep
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_RIGHT;
   private Point3D goalPosition = new Point3D(0.0, 0.0, 0.0);
   private double groundClearance = 0.0;
   private TrajectoryType trajectoryType = null;

   public QuadrupedStep()
   {
   }

   public QuadrupedStep(RobotQuadrant robotQuadrant, FramePoint3D goalPosition, double groundClearance)
   {
      setRobotQuadrant(robotQuadrant);
      setGoalPosition(goalPosition);
      setGroundClearance(groundClearance);
   }

   public QuadrupedStep(RobotQuadrant robotQuadrant, Point3DBasics goalPosition, double groundClearance)
   {
      this();
      setRobotQuadrant(robotQuadrant);
      setGoalPosition(goalPosition);
      setGroundClearance(groundClearance);
   }

   public QuadrupedStep(QuadrupedStep other)
   {
      this(other.getRobotQuadrant(), other.getGoalPositionInternal(), other.getGroundClearance());
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return robotQuadrant;
   }

   protected Point3DBasics getGoalPositionInternal()
   {
      return goalPosition;
   }

   public Point3DReadOnly getGoalPosition()
   {
      return goalPosition;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return worldFrame;
   }

   public void setRobotQuadrant(RobotQuadrant robotQuadrant)
   {
      this.robotQuadrant = robotQuadrant;
   }
   
   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType = trajectoryType;
   }

   public void setGoalPosition(Point3DReadOnly goalPosition)
   {
      getGoalPositionInternal().set(goalPosition);
   }

   public void setGoalPosition(FramePoint3D goalPosition)
   {
      ReferenceFrame originalFrame = goalPosition.getReferenceFrame();
      goalPosition.changeFrame(getReferenceFrame());
      getGoalPositionInternal().set(goalPosition);
      goalPosition.changeFrame(originalFrame);
   }

   public double getGroundClearance()
   {
      return groundClearance;
   }

   public void setGroundClearance(double groundClearance)
   {
      this.groundClearance = groundClearance;
   }

   public void set(QuadrupedStep other)
   {
      setRobotQuadrant(other.getRobotQuadrant());
      setGoalPosition(other.getGoalPositionInternal());
      setGroundClearance(other.getGroundClearance());
      setTrajectoryType(other.getTrajectoryType());
   }

   public void set(QuadrupedStepCommand command)
   {
      setRobotQuadrant(command.getRobotQuadrant());
      setGoalPosition(command.getGoalPosition());
      setGroundClearance(command.getGroundClearance());
      setTrajectoryType(command.getTrajectoryType());
   }

   public void set(QuadrupedStepMessage message)
   {
      setRobotQuadrant(RobotQuadrant.fromByte(message.getRobotQuadrant()));
      setGoalPosition(message.getGoalPosition());
      setGroundClearance(message.getGroundClearance());
      setTrajectoryType(TrajectoryType.fromByte(message.getTrajectoryType()));
   }

   public void get(QuadrupedTimedStep other)
   {
      other.setRobotQuadrant(getRobotQuadrant());
      other.setGoalPosition(getGoalPositionInternal());
      other.setGroundClearance(getGroundClearance());
      other.setTrajectoryType(getTrajectoryType());
   }

   public boolean epsilonEquals(QuadrupedTimedStep other, double epsilon)
   {
      return getRobotQuadrant() == other.getRobotQuadrant() &&
             getGoalPositionInternal().epsilonEquals(other.getGoalPositionInternal(), epsilon) &&
             MathTools.epsilonEquals(getGroundClearance(), other.getGroundClearance(), epsilon) && 
             (getTrajectoryType() == other.getTrajectoryType());

   }

   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      QuadrupedStep other = (QuadrupedStep) obj;

      if (getRobotQuadrant() != other.getRobotQuadrant())
         return false;
      if (getGroundClearance() != other.getGroundClearance())
         return false;
      
      if (trajectoryType != other.trajectoryType)
         return false;

      return getGoalPosition().geometricallyEquals(other.getGoalPosition(), 0.0);
   }

   @Override public String toString()
   {
      String string = super.toString();
      string += "\nrobotQuadrant: " + getRobotQuadrant();
      string += "\ngoalPosition:" + getGoalPositionInternal();
      string += "\ngroundClearance: " + getGroundClearance();
      string += "\ntrajectoryType: " + getTrajectoryType();
      return string;
   }
}
