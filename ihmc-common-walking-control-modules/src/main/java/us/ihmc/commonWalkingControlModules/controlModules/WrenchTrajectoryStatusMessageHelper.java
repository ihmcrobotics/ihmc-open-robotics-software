package us.ihmc.commonWalkingControlModules.controlModules;

import controller_msgs.msg.dds.WrenchTrajectoryStatusMessage;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WrenchTrajectoryControllerCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

public class WrenchTrajectoryStatusMessageHelper extends TrajectoryStatusMessageHelper<WrenchTrajectoryStatusMessage>
{
   private final WrenchTrajectoryStatusMessage statusMessage = new WrenchTrajectoryStatusMessage();

   public WrenchTrajectoryStatusMessageHelper(RigidBodyReadOnly endEffector)
   {
      statusMessage.getEndEffectorName().append(endEffector.getName());
   }

   public void registerNewTrajectory(WrenchTrajectoryControllerCommand command)
   {
      if (command.getExecutionMode() == ExecutionMode.OVERRIDE)
      {
         clear();
         registerNewTrajectory(command.getSequenceId(), 0.0, command.getTrajectoryEndTime());
      }
      else
      {
         registerNewTrajectory(command.getSequenceId(), command.getTrajectoryStartTime(), command.getTrajectoryEndTime());
      }
   }

   public WrenchTrajectoryStatusMessage pollStatusMessage()
   {
      TrajectoryStatus currentStatus = pollStatus();

      if (currentStatus == null)
         return null;

      statusMessage.setSequenceId(currentStatus.getSequenceID());
      statusMessage.setTimestamp(currentStatus.getTimeInTrajectory());
      statusMessage.setTrajectoryExecutionStatus(currentStatus.getStatus().toByte());

      return statusMessage;
   }

   public WrenchTrajectoryStatusMessage getStatusMessage()
   {
      return statusMessage;
   }
}
