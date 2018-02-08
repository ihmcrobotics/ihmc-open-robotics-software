package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class HandTrajectoryCommand
      implements Command<HandTrajectoryCommand, HandTrajectoryMessage>, FrameBasedCommand<HandTrajectoryMessage>, EpsilonComparable<HandTrajectoryCommand>
{
   private RobotSide robotSide;
   private final SE3TrajectoryControllerCommand se3Trajectory;

   public HandTrajectoryCommand()
   {
      se3Trajectory = new SE3TrajectoryControllerCommand();
   }

   public HandTrajectoryCommand(RobotSide robotSide, ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame)
   {
      se3Trajectory = new SE3TrajectoryControllerCommand(dataFrame, trajectoryFrame);
      this.robotSide = robotSide;
   }

   public HandTrajectoryCommand(Random random)
   {
      se3Trajectory = new SE3TrajectoryControllerCommand(random);
      robotSide = RobotSide.generateRandomRobotSide(random);
   }

   @Override
   public void clear()
   {
      se3Trajectory.clear();
      robotSide = null;
   }

   public void clear(ReferenceFrame referenceFrame)
   {
      se3Trajectory.clear(referenceFrame);
      robotSide = null;
   }

   @Override
   public void set(HandTrajectoryCommand other)
   {
      se3Trajectory.set(other.se3Trajectory);
      robotSide = other.robotSide;
   }

   /**
    * Same as {@link #set(HandTrajectoryCommand)} but does not change the trajectory points.
    *
    * @param other
    */
   public void setPropertiesOnly(HandTrajectoryCommand other)
   {
      se3Trajectory.setPropertiesOnly(other.se3Trajectory);
      robotSide = other.robotSide;
   }

   @Override
   public void set(HandTrajectoryMessage message)
   {
      se3Trajectory.set(message.se3Trajectory);
      robotSide = message.getRobotSide();
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, HandTrajectoryMessage message)
   {
      se3Trajectory.set(resolver, message.se3Trajectory);
      robotSide = message.getRobotSide();
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public SE3TrajectoryControllerCommand getSE3Trajectory()
   {
      return se3Trajectory;
   }

   @Override
   public Class<HandTrajectoryMessage> getMessageClass()
   {
      return HandTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && se3Trajectory.isCommandValid();
   }

   @Override
   public boolean epsilonEquals(HandTrajectoryCommand other, double epsilon)
   {
      return robotSide == other.robotSide && se3Trajectory.epsilonEquals(other.se3Trajectory, epsilon);
   }
   
   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      se3Trajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      se3Trajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return se3Trajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return se3Trajectory.getExecutionTime();
   }
}
