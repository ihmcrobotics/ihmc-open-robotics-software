package us.ihmc.commonWalkingControlModules.controlModules;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

public class TaskspaceTrajectoryStatusMessageHelper extends TrajectoryStatusMessageHelper<TaskspaceTrajectoryStatusMessage>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final TaskspaceTrajectoryStatusMessage statusMessage = new TaskspaceTrajectoryStatusMessage();
   private final FixedFramePose3DBasics controlFramePose = new FramePose3D(worldFrame);
   private final FixedFramePose3DBasics desiredPose = new FramePose3D(worldFrame);

   public TaskspaceTrajectoryStatusMessageHelper(RigidBodyReadOnly endEffector)
   {
      statusMessage.getEndEffectorName().append(endEffector.getName());
   }

   public TaskspaceTrajectoryStatusMessageHelper(String endEffectorName)
   {
      statusMessage.getEndEffectorName().append(endEffectorName);
      clear();
   }

   @Override
   public void clear()
   {
      super.clear();

      controlFramePose.setToNaN();
      desiredPose.setToNaN();
      statusMessage.getActualEndEffectorOrientation().setToNaN();
      statusMessage.getActualEndEffectorPosition().setToNaN();
      statusMessage.getDesiredEndEffectorOrientation().setToNaN();
      statusMessage.getDesiredEndEffectorPosition().setToNaN();
   }

   public void registerNewTrajectory(SE3TrajectoryControllerCommand command)
   {
      if (command.getExecutionMode() == ExecutionMode.OVERRIDE)
      {
         clear();
         registerNewTrajectory(command.getSequenceId(), 0.0, command.getLastTrajectoryPoint().getTime());
      }
      else
      {
         registerNewTrajectory(command.getSequenceId(), command.getTrajectoryPoint(0).getTime(), command.getLastTrajectoryPoint().getTime());
      }
   }

   public void registerNewTrajectory(SO3TrajectoryControllerCommand command)
   {
      if (command.getExecutionMode() == ExecutionMode.OVERRIDE)
      {
         clear();
         registerNewTrajectory(command.getSequenceId(), 0.0, command.getLastTrajectoryPoint().getTime());
      }
      else
      {
         registerNewTrajectory(command.getSequenceId(), command.getTrajectoryPoint(0).getTime(), command.getLastTrajectoryPoint().getTime());
      }
   }

   public void registerNewTrajectory(EuclideanTrajectoryControllerCommand command)
   {
      if (command.getExecutionMode() == ExecutionMode.OVERRIDE)
      {
         clear();
         registerNewTrajectory(command.getSequenceId(), 0.0, command.getLastTrajectoryPoint().getTime());
      }
      else
      {
         registerNewTrajectory(command.getSequenceId(), command.getTrajectoryPoint(0).getTime(), command.getLastTrajectoryPoint().getTime());
      }
   }

   public TaskspaceTrajectoryStatusMessage pollStatusMessage(SpatialFeedbackControlCommand feedbackControlCommand)
   {
      TrajectoryStatus currentStatus = pollStatus();

      if (currentStatus == null)
         return null;

      updateStatusCommonInfo(currentStatus);
      updateStatusInfo(feedbackControlCommand);

      return statusMessage;
   }

   public TaskspaceTrajectoryStatusMessage pollStatusMessage(OrientationFeedbackControlCommand feedbackControlCommand)
   {
      TrajectoryStatus currentStatus = pollStatus();

      if (currentStatus == null)
         return null;

      updateStatusCommonInfo(currentStatus);
      updateStatusInfo(feedbackControlCommand);

      return statusMessage;
   }

   public TaskspaceTrajectoryStatusMessage pollStatusMessage(PointFeedbackControlCommand feedbackControlCommand)
   {
      TrajectoryStatus currentStatus = pollStatus();

      if (currentStatus == null)
         return null;

      updateStatusCommonInfo(currentStatus);
      updateStatusInfo(feedbackControlCommand);

      return statusMessage;
   }

   public TaskspaceTrajectoryStatusMessage pollStatusMessage(FramePoint2DReadOnly desiredPositionInWorld, FramePoint2DReadOnly actualPositionInWorld)
   {
      TrajectoryStatus currentStatus = pollStatus();

      if (currentStatus == null)
         return null;

      updateStatusCommonInfo(currentStatus);
      desiredPositionInWorld.checkReferenceFrameMatch(worldFrame);
      actualPositionInWorld.checkReferenceFrameMatch(worldFrame);
      updateStatusInfo(desiredPositionInWorld, actualPositionInWorld);

      return statusMessage;
   }

   public TaskspaceTrajectoryStatusMessage pollStatusMessage(FramePoint3DReadOnly desiredPositionInWorld, FramePoint3DReadOnly actualPositionInWorld)
   {
      TrajectoryStatus currentStatus = pollStatus();

      if (currentStatus == null)
         return null;

      updateStatusCommonInfo(currentStatus);
      desiredPositionInWorld.checkReferenceFrameMatch(worldFrame);
      actualPositionInWorld.checkReferenceFrameMatch(worldFrame);
      updateStatusInfo(desiredPositionInWorld, actualPositionInWorld);

      return statusMessage;
   }

   private void updateStatusCommonInfo(TrajectoryStatus currentStatus)
   {
      statusMessage.setSequenceId(currentStatus.getSequenceID());
      statusMessage.setTimestamp(currentStatus.getTimeInTrajectory());
      statusMessage.setTrajectoryExecutionStatus(currentStatus.getStatus().toByte());
   }

   private void updateStatusInfo(SpatialFeedbackControlCommand feedbackControlCommand)
   {
      controlFramePose.setMatchingFrame(feedbackControlCommand.getControlFramePose());
      desiredPose.getOrientation().setMatchingFrame(feedbackControlCommand.getReferenceOrientation());
      desiredPose.getPosition().setMatchingFrame(feedbackControlCommand.getReferencePosition());

      updateStatusInfo(desiredPose, controlFramePose);
   }

   private void updateStatusInfo(OrientationFeedbackControlCommand feedbackControlCommand)
   {
      controlFramePose.getOrientation().setMatchingFrame(feedbackControlCommand.getBodyFixedOrientationToControl());
      controlFramePose.getPosition().setToNaN();

      desiredPose.getOrientation().setMatchingFrame(feedbackControlCommand.getReferenceOrientation());
      desiredPose.getPosition().setToNaN();

      updateStatusInfo(desiredPose, controlFramePose);
   }

   private void updateStatusInfo(PointFeedbackControlCommand feedbackControlCommand)
   {
      controlFramePose.getOrientation().setToNaN();
      controlFramePose.getPosition().setMatchingFrame(feedbackControlCommand.getBodyFixedPointToControl());

      desiredPose.getOrientation().setToNaN();
      desiredPose.getPosition().setMatchingFrame(feedbackControlCommand.getReferencePosition());

      updateStatusInfo(desiredPose, controlFramePose);
   }

   private void updateStatusInfo(Point2DReadOnly desiredPosition, Point2DReadOnly actualPosition)
   {
      statusMessage.getDesiredEndEffectorOrientation().setToNaN();
      statusMessage.getDesiredEndEffectorPosition().set(desiredPosition, Double.NaN);
      statusMessage.getActualEndEffectorOrientation().setToNaN();
      statusMessage.getActualEndEffectorPosition().set(actualPosition, Double.NaN);
   }

   private void updateStatusInfo(Point3DReadOnly desiredPosition, Point3DReadOnly actualPosition)
   {
      statusMessage.getDesiredEndEffectorOrientation().setToNaN();
      statusMessage.getDesiredEndEffectorPosition().set(desiredPosition);
      statusMessage.getActualEndEffectorOrientation().setToNaN();
      statusMessage.getActualEndEffectorPosition().set(actualPosition);
   }

   private void updateStatusInfo(Pose3DReadOnly desiredPose, Pose3DReadOnly actualPose)
   {
      statusMessage.getDesiredEndEffectorOrientation().set(desiredPose.getOrientation());
      statusMessage.getDesiredEndEffectorPosition().set(desiredPose.getPosition());
      statusMessage.getActualEndEffectorOrientation().set(actualPose.getOrientation());
      statusMessage.getActualEndEffectorPosition().set(actualPose.getPosition());
   }
}
