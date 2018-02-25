package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.JointspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.LoadBearingMessage;

public class HandLoadBearingMessage extends Packet<HandLoadBearingMessage>
{
   public static final byte ROBOT_SIDE_LEFT = 0;
   public static final byte ROBOT_SIDE_RIGHT = 1;

   /** The robot side of that hand that will be load bearing. */
   public byte robotSide;

   /** A boolean that determines whether hybrid load-bearing & jointspace control will be used. */
   public boolean useJointspaceCommand = false;

   /**
    * The jointspace arm trajectory message that will be used for the hybrid control if
    * {@link #useJointspaceCommand} is true.
    */
   public JointspaceTrajectoryMessage jointspaceTrajectory = new JointspaceTrajectoryMessage();

   /** the time to delay this command on the controller side before being executed **/
   public double executionDelayTime;

   public LoadBearingMessage loadBearingMessage = new LoadBearingMessage();

   public HandLoadBearingMessage()
   {
      loadBearingMessage = new LoadBearingMessage();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(HandLoadBearingMessage other)
   {
      robotSide = other.robotSide;
      useJointspaceCommand = other.useJointspaceCommand;
      jointspaceTrajectory = new JointspaceTrajectoryMessage();
      jointspaceTrajectory.set(other.jointspaceTrajectory);
      executionDelayTime = other.executionDelayTime;
      loadBearingMessage = other.loadBearingMessage;
      setPacketInformation(other);
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

   public boolean getUseJointspaceCommand()
   {
      return useJointspaceCommand;
   }

   public LoadBearingMessage getLoadBearingMessage()
   {
      return loadBearingMessage;
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (loadBearingMessage != null)
         loadBearingMessage.setUniqueId(uniqueId);
   }

   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * 
    * @return the time to delay this command in seconds
    */
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }

   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * 
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

      return robotSideEqual && armTrajectoryEqual && useArmTrajectoryEqual && loadBearingMessage.epsilonEquals(other.loadBearingMessage, epsilon);
   }
}
