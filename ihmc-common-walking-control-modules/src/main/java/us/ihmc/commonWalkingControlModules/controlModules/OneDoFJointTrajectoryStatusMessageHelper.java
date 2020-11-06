package us.ihmc.commonWalkingControlModules.controlModules;

import controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.OneDoFJointTrajectoryCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;

public class OneDoFJointTrajectoryStatusMessageHelper extends TrajectoryStatusMessageHelper<JointspaceTrajectoryStatusMessage>
{
   private final JointspaceTrajectoryStatusMessage statusMessage = new JointspaceTrajectoryStatusMessage();

   public OneDoFJointTrajectoryStatusMessageHelper(OneDoFJointReadOnly joint)
   {
      statusMessage.getJointNames().add(joint.getName());
      statusMessage.getActualJointPositions().add(Double.NaN);
      statusMessage.getDesiredJointPositions().add(Double.NaN);
      clear();
   }

   @Override
   public void clear()
   {
      super.clear();

      statusMessage.getActualJointPositions().set(0, Double.NaN);
      statusMessage.getDesiredJointPositions().set(0, Double.NaN);
   }

   public void registerNewTrajectory(OneDoFJointTrajectoryCommand command, ExecutionMode executionMode)
   {
      if (executionMode == ExecutionMode.OVERRIDE)
      {
         clear();
         registerNewTrajectory(command.getSequenceId(), 0.0, command.getTrajectoryTime());
      }
      else
      {
         registerNewTrajectory(command.getSequenceId(), command.getTrajectoryPoint(0).getTime(), command.getTrajectoryTime());
      }
   }

   public JointspaceTrajectoryStatusMessage pollStatusMessage(double qCurrent, double qDesired)
   {
      TrajectoryStatus currentStatus = pollStatus();

      if (currentStatus == null)
         return null;

      statusMessage.setSequenceId(currentStatus.getSequenceID());
      statusMessage.setTimestamp(currentStatus.getTimeInTrajectory());
      statusMessage.setTrajectoryExecutionStatus(currentStatus.getStatus().toByte());
      statusMessage.getActualJointPositions().set(0, qCurrent);
      statusMessage.getDesiredJointPositions().set(0, qDesired);

      return statusMessage;
   }
}
