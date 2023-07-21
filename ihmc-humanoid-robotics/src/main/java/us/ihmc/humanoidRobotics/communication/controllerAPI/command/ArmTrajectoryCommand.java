package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmTrajectoryCommand implements Command<ArmTrajectoryCommand, ArmTrajectoryMessage>, EpsilonComparable<ArmTrajectoryCommand>
{
   private long sequenceId;
   private RobotSide robotSide;
   private boolean forceExecution = false;
   private RequestedMode requestedMode = null;
   private final JointspaceTrajectoryCommand jointspaceTrajectory;

   public ArmTrajectoryCommand()
   {
      sequenceId = 0;
      robotSide = null;
      setForceExecution(false);
      jointspaceTrajectory = new JointspaceTrajectoryCommand();
   }

   public ArmTrajectoryCommand(Random random)
   {
      sequenceId = random.nextInt();
      robotSide = random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT;
      setForceExecution(random.nextBoolean());
      jointspaceTrajectory = new JointspaceTrajectoryCommand(random);
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      robotSide = null;
      setForceExecution(false);
      setRequestedMode(null);
      jointspaceTrajectory.clear();
   }

   public void clear(RobotSide robotSide)
   {
      clear();
      this.robotSide = robotSide;
   }

   @Override
   public void setFromMessage(ArmTrajectoryMessage message)
   {
      clear(RobotSide.fromByte(message.getRobotSide()));
      setForceExecution(message.getForceExecution());
      setRequestedMode(RequestedMode.fromByte(message.getRequestedMode()));
      sequenceId = message.getSequenceId();
      jointspaceTrajectory.setFromMessage(message.getJointspaceTrajectory());
   }

   @Override
   public void set(ArmTrajectoryCommand other)
   {
      clear(other.getRobotSide());
      setForceExecution(other.getForceExecution());
      setRequestedMode(other.getRequestedMode());
      sequenceId = other.sequenceId;
      jointspaceTrajectory.set(other.getJointspaceTrajectory());
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public void setForceExecution(boolean forceExecution)
   {
      this.forceExecution = forceExecution;
   }

   public void setRequestedMode(RequestedMode requestedMode)
   {
      this.requestedMode = requestedMode;
   }

   public boolean getForceExecution()
   {
      return forceExecution;
   }

   public RequestedMode getRequestedMode()
   {
      return requestedMode;
   }

   public JointspaceTrajectoryCommand getJointspaceTrajectory()
   {
      return jointspaceTrajectory;
   }

   @Override
   public Class<ArmTrajectoryMessage> getMessageClass()
   {
      return ArmTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && jointspaceTrajectory.isCommandValid();
   }

   @Override
   public boolean epsilonEquals(ArmTrajectoryCommand other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;
      if (!jointspaceTrajectory.epsilonEquals(other.jointspaceTrajectory, epsilon))
         return false;
      return true;
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      jointspaceTrajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      jointspaceTrajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return jointspaceTrajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return jointspaceTrajectory.getExecutionTime();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   public enum RequestedMode
   {
      TORQUE_CONTROL, POSITION_CONTROL;

      public static final RequestedMode[] values = values();

      public byte toByte()
      {
         return (byte) ordinal();
      }

      public static RequestedMode fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         return values[enumAsByte];
      }
   }
}
