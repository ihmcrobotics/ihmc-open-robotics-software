package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.humanoidRobotics.communication.packets.AbstractLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.JointspaceTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandLoadBearingMessage extends AbstractLoadBearingMessage<HandLoadBearingMessage>
{
   /** The robot side of that hand that will be load bearing. */
   public RobotSide robotSide;

   /** A boolean that determines whether hybrid load-bearing & jointspace control will be used. */
   public boolean useJointspaceCommand = false;

   /** The jointspace arm trajectory message that will be used for the hybrid control if {@link #useJointspaceCommand} is true. */
   public JointspaceTrajectoryMessage jointspaceTrajectory;

   /** the time to delay this command on the controller side before being executed **/
   public double executionDelayTime;

   public HandLoadBearingMessage()
   {
      super();
   }

   public HandLoadBearingMessage(RobotSide robotSide)
   {
      super();
      this.robotSide = robotSide;
   }

   public HandLoadBearingMessage(Random random)
   {
      super(random);
      robotSide = RandomNumbers.nextEnum(random, RobotSide.class);
      jointspaceTrajectory = new JointspaceTrajectoryMessage(random);
   }

   public void setJointspaceTrajectory(JointspaceTrajectoryMessage armTrajectoryMessage)
   {
      this.jointspaceTrajectory = armTrajectoryMessage;
   }

   public JointspaceTrajectoryMessage getJointspaceTrajectory()
   {
      return jointspaceTrajectory;
   }

   public void setUseJointspaceCommand(boolean useJointspaceCommand)
   {
      this.useJointspaceCommand = useJointspaceCommand;
   }

   public boolean isUseJointspaceCommand()
   {
      return useJointspaceCommand;
   }
   
   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * @return the time to delay this command in seconds
    */
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }
   
   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   public void setExecutionDelayTime(double delayTime)
   {
      this.executionDelayTime = delayTime;
   }

   @Override
   public boolean epsilonEquals(HandLoadBearingMessage other, double epsilon)
   {
      boolean robotSideEqual = robotSide == other.robotSide;

      boolean armTrajectoryEqual;
      if (jointspaceTrajectory == null && other.jointspaceTrajectory == null)
      {
         armTrajectoryEqual = true;
      }
      else if (jointspaceTrajectory == null && other.jointspaceTrajectory != null)
      {
         armTrajectoryEqual = false;
      }
      else if (jointspaceTrajectory != null && other.jointspaceTrajectory == null)
      {
         armTrajectoryEqual = false;
      }
      else
      {
         armTrajectoryEqual = jointspaceTrajectory.epsilonEquals(other.jointspaceTrajectory, epsilon);
      }

      boolean useArmTrajectoryEqual = useJointspaceCommand == other.useJointspaceCommand;

      return robotSideEqual && armTrajectoryEqual && useArmTrajectoryEqual && super.epsilonEquals(other, epsilon);
   }
}
