package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmTrajectoryCommand extends JointspaceTrajectoryCommand<ArmTrajectoryCommand, ArmTrajectoryMessage>
{
   private RobotSide robotSide;

   public ArmTrajectoryCommand()
   {
      super();
      robotSide = null;
   }

   public ArmTrajectoryCommand(Random random)
   {
      super(random);
      robotSide = random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT;
   }

   @Override
   public void clear()
   {
      robotSide = null;
      super.clear();
   }

   public void clear(RobotSide robotSide)
   {
      this.robotSide = robotSide;
      super.clear();
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   @Override
   public void set(ArmTrajectoryMessage message)
   {
      clear(message.getRobotSide());
      super.set(message);
   }

   @Override
   public void set(ArmTrajectoryCommand other)
   {
      clear(other.getRobotSide());
      super.set(other);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public Class<ArmTrajectoryMessage> getMessageClass()
   {
      return ArmTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && super.isCommandValid();
   }
   
   @Override
   public boolean epsilonEquals(ArmTrajectoryCommand other, double epsilon)
   {
      if(robotSide != other.robotSide)
      {
         return false;
      }
      return super.epsilonEquals(other, epsilon);
   }
}
