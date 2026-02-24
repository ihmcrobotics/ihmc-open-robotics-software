package us.ihmc.commonWalkingControlModules.controlModules;

import controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;

public class JointspaceTrajectoryStatusMessageHelper extends TrajectoryStatusMessageHelper<JointspaceTrajectoryStatusMessage>
{
   private final JointspaceTrajectoryStatusMessage statusMessage = new JointspaceTrajectoryStatusMessage();

   public JointspaceTrajectoryStatusMessageHelper(OneDoFJointReadOnly[] joints)
   {
      for (OneDoFJointReadOnly joint : joints)
      {
         statusMessage.getJointNames().add(joint.getName());
         statusMessage.getActualJointPositions().add(Double.NaN);
         statusMessage.getDesiredJointPositions().add(Double.NaN);
      }
      clear();
   }

   @Override
   public void clear()
   {
      super.clear();

      for (int jointIndex = 0; jointIndex < statusMessage.getDesiredJointPositions().size(); jointIndex++)
      {
         statusMessage.getActualJointPositions().set(jointIndex, Double.NaN);
         statusMessage.getDesiredJointPositions().set(jointIndex, Double.NaN);
      }
   }

   public void registerNewTrajectory(JointspaceTrajectoryCommand command)
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

   public JointspaceTrajectoryStatusMessage pollStatusMessage(JointspaceFeedbackControlCommand feedbackControlCommand)
   {
      TrajectoryStatus currentStatus = pollStatus();

      if (currentStatus == null)
         return null;

      statusMessage.setSequenceId(currentStatus.getSequenceID());
      statusMessage.setTimestamp(currentStatus.getTimeInTrajectory());
      statusMessage.setTrajectoryExecutionStatus(currentStatus.getStatus().toByte());
      updateStatusInfo(feedbackControlCommand);

      return statusMessage;
   }

   private void updateStatusInfo(JointspaceFeedbackControlCommand feedbackControlCommand)
   {
      for (int jointIndex = 0; jointIndex < feedbackControlCommand.getNumberOfJoints(); jointIndex++)
      {
         OneDoFJointFeedbackControlCommand jointCommand = feedbackControlCommand.getJointCommand(jointIndex);
         double qCurrent = jointCommand.getJoint().getQ();
         double qDesired = jointCommand.getReferencePosition();
         statusMessage.getActualJointPositions().set(jointIndex, qCurrent);
         statusMessage.getDesiredJointPositions().set(jointIndex, qDesired);
      }
   }
}
