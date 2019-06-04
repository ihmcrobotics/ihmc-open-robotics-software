package us.ihmc.humanoidRobotics.communication.packets;

import java.util.List;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.packets.ObjectValidityChecker;
import us.ihmc.communication.packets.ObjectValidityChecker.ObjectErrorType;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

public abstract class PacketValidityChecker
{
   /**
    * Checks the validity of a {@link FootstepDataMessage}.
    *
    * @param message
    * @return null if the packet is valid, or the error message.
    */
   public static String validateFootstepDataMessage(FootstepDataMessage message)
   {
      ObjectErrorType packetFieldErrorType;

      packetFieldErrorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.getRobotSide()));
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s robotSide field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateTuple3d(message.getLocation());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s location field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateQuat4d(message.getOrientation());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s orientation field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (message.getPredictedContactPoints2d() != null)
      {
         for (int arrayListIndex = 0; arrayListIndex < message.getPredictedContactPoints2d().size(); arrayListIndex++)
         {
            packetFieldErrorType = ObjectValidityChecker.validateTuple3d(message.getPredictedContactPoints2d().get(arrayListIndex));

            if (packetFieldErrorType != null)
            {
               String messageClassName = message.getClass().getSimpleName();
               String errorMessage = messageClassName + "'s predictedContactPoints field " + packetFieldErrorType.getMessage();
               return errorMessage;
            }
         }
      }

      TrajectoryType trajectoryType = TrajectoryType.fromByte(message.getTrajectoryType());
      packetFieldErrorType = ObjectValidityChecker.validateEnum(trajectoryType);
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s trajectoryType field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      //TODO Check if thats supposed to be checked
      packetFieldErrorType = ObjectValidityChecker.validateDouble(message.getSwingHeight());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s swingHeight field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (trajectoryType == TrajectoryType.WAYPOINTS)
      {
         String messageClassName = message.getClass().getSimpleName();
         List<SE3TrajectoryPointMessage> swingTrajectory = message.getSwingTrajectory();

         if (swingTrajectory == null)
         {
            String errorMessage = messageClassName + " has no swing trajectory but trajectory type was set to " + TrajectoryType.WAYPOINTS.toString() + ".";
            return errorMessage;
         }

         if (swingTrajectory.size() > Footstep.maxNumberOfSwingWaypoints)
         {
            String errorMessage =
                  messageClassName + " has " + swingTrajectory.size() + " waypoints. Up to " + Footstep.maxNumberOfSwingWaypoints + " are allowed.";
            return errorMessage;
         }

         double lastTime = swingTrajectory.get(0).getTime();
         if (lastTime < 0.0)
         {
            String errorMessage = messageClassName + "'s swing trajectory can not start at time below zero.";
            return errorMessage;
         }
         for (int waypointIdx = 1; waypointIdx < swingTrajectory.size(); waypointIdx++)
         {
            double waypointTime = swingTrajectory.get(waypointIdx).getTime();
            if (waypointTime <= lastTime)
            {
               String errorMessage = messageClassName + "'s swing trajectory has non-increasing waypoint times.";
               return errorMessage;
            }
            lastTime = waypointTime;
         }

         if (message.getSwingDuration() > 0.0 && lastTime > message.getSwingDuration())
         {
            String errorMessage = messageClassName + "'s swing trajectory has waypoints with time larger then the swing time.";
            return errorMessage;
         }

         if (message.getSwingTrajectoryBlendDuration() < 0.0)
         {
            String errorMessage = messageClassName + "'s swing trajectory blend duration is less than zero.";
            return errorMessage;
         }

         if (message.getSwingTrajectoryBlendDuration() > 0.0 && message.getSwingTrajectory().get(0).getTime() > 1.0e-5)
         {
            String errorMessage = messageClassName + "'s swing trajectory blend duration is greater than zero, initial waypoint at t = 0.0 is missing.";
            return errorMessage;
         }
      }

      if (trajectoryType == TrajectoryType.CUSTOM)
      {
         String messageClassName = message.getClass().getSimpleName();
         List<Point3D> positionWaypoints = message.getCustomPositionWaypoints();
         if (positionWaypoints == null)
         {
            String errorMessage = messageClassName + "'s type is custom but no position waypoints were specified.";
            return errorMessage;
         }
      }

      return null;
   }

   /**
    * Checks the validity of a {@link FootstepDataListMessage}.
    *
    * @param message
    * @return null if the packet is valid, or the error message.
    */
   public static String validateFootstepDataListMessage(FootstepDataListMessage message)
   {
      ObjectErrorType packetFieldErrorType;

      packetFieldErrorType = ObjectValidityChecker.validateDouble(message.getDefaultSwingDuration());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s swingTime field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateDouble(message.getDefaultTransferDuration());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s transferTime field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (message.getFootstepDataList() != null)
      {
         for (int arrayListIndex = 0; arrayListIndex < message.getFootstepDataList().size(); arrayListIndex++)
         {
            FootstepDataMessage footstepData = message.getFootstepDataList().get(arrayListIndex);
            String footstepDataListErrorMessage = validateFootstepDataMessage(footstepData);

            if (footstepDataListErrorMessage != null)
            {
               String messageClassName = message.getClass().getSimpleName();
               String errorMessage = messageClassName + " field contains a FootstepData in which " + footstepDataListErrorMessage;
               return errorMessage;
            }
         }
      }

      return null;
   }

   /**
    * Checks the validity of a {@link QuadrupedStepMessage}.
    *
    * @param message
    * @return null if the packet is valid, or the error message.
    */
   public static String validateQuadrupedStepMessage(QuadrupedStepMessage message)
   {
      ObjectErrorType packetFieldErrorType;

      packetFieldErrorType = ObjectValidityChecker.validateEnum(RobotQuadrant.fromByte(message.getRobotQuadrant()));
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s robotQuadrant field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateTuple3d(message.getGoalPosition());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s goalPosition field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      //TODO Check if thats supposed to be checked
      packetFieldErrorType = ObjectValidityChecker.validateDouble(message.getGroundClearance());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s groundClearance field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   /**
    * Checks the validity of a {@link QuadrupedStepMessage}.
    *
    * @param message
    * @return null if the packet is valid, or the error message.
    */
   public static String validateTimeIntervalMessage(TimeIntervalMessage message)
   {
      ObjectErrorType packetFieldErrorType;

      packetFieldErrorType = ObjectValidityChecker.validateDouble(message.getStartTime());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s startTime field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateDouble(message.getEndTime());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s endTime field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   /**
    * Checks the validity of a {@link QuadrupedTimedStepMessage}.
    *
    * @param message
    * @return null if the packet is valid, or the error message.
    */
   public static String validateQuadrupedTimedStepMessage(QuadrupedTimedStepMessage message)
   {
      String stepErrorMessage = validateQuadrupedStepMessage(message.getQuadrupedStepMessage());
      if (stepErrorMessage != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + " step field which " + stepErrorMessage;
         return errorMessage;
      }

      String timeErrorMessage = validateTimeIntervalMessage(message.getTimeInterval());
      if (timeErrorMessage != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + " time interval field which " + timeErrorMessage;
         return errorMessage;
      }

      return null;
   }

   /**
    * Checks the validity of a {@link QuadrupedTimedStepListMessage}.
    *
    * @param message
    * @return null if the packet is valid, or the error message.
    */
   public static String validateQuadrupedTimedStepListMessage(QuadrupedTimedStepListMessage message)
   {
      if (message.getQuadrupedStepList() != null)
      {
         for (int arrayListIndex = 0; arrayListIndex < message.getQuadrupedStepList().size(); arrayListIndex++)
         {
            QuadrupedTimedStepMessage stepMessage = message.getQuadrupedStepList().get(arrayListIndex);
            String footstepDataListErrorMessage = validateQuadrupedTimedStepMessage(stepMessage);

            if (footstepDataListErrorMessage != null)
            {
               String messageClassName = message.getClass().getSimpleName();
               String errorMessage = messageClassName + " field contains a FootstepData in which " + footstepDataListErrorMessage;
               return errorMessage;
            }
         }
      }

      return null;
   }

   /**
    * Checks the validity of a {@link QuadrupedFootLoadBearingMessage}.
    *
    * @param message
    * @return null if the packet is valid, or the error message.
    */
   public static String validateQuadrupedFootLoadBearingRequestMessage(QuadrupedFootLoadBearingMessage message)
   {
      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateEnum(RobotQuadrant.fromByte(message.getRobotQuadrant()));
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s robotQuadrant field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   public static String validateSoleTrajectoryMessage(SoleTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      EuclideanTrajectoryPointMessage previousTrajectoryPoint = null;

      if (message.getPositionTrajectory().getTaskspaceTrajectoryPoints().isEmpty())
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = "Received " + messageClassName + " with no waypoint.";
         return errorMessage;
      }

      for (int i = 0; i < message.getPositionTrajectory().getTaskspaceTrajectoryPoints().size(); i++)
      {
         EuclideanTrajectoryPointMessage waypoint = message.getPositionTrajectory().getTaskspaceTrajectoryPoints().get(i);
         errorMessage = validateEuclideanTrajectoryPointMessage(waypoint, previousTrajectoryPoint, false);
         if (errorMessage != null)
         {
            String messageClassName = message.getClass().getSimpleName();
            errorMessage = "The " + messageClassName + "'s " + i + "th waypoint " + errorMessage;
            return errorMessage;
         }
         previousTrajectoryPoint = waypoint;
      }


      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateEnum(RobotQuadrant.fromByte(message.getRobotQuadrant()));
      if (errorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + "'s robotQuadrant field " + errorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   /**
    * Checks the validity of a {@link FootstepDataMessage}.
    *
    * @param message
    * @return null if the packet is valid, or the error message.
    */
   public static String validateAdjustFootstepMessage(AdjustFootstepMessage message)
   {
      ObjectErrorType packetFieldErrorType;

      packetFieldErrorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.getRobotSide()));
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s robotSide field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateTuple3d(message.getLocation());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s location field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateQuat4d(message.getOrientation());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s orientation field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (message.getPredictedContactPoints2d() != null)
      {
         for (int arrayListIndex = 0; arrayListIndex < message.getPredictedContactPoints2d().size(); arrayListIndex++)
         {
            packetFieldErrorType = ObjectValidityChecker.validateTuple3d(message.getPredictedContactPoints2d().get(arrayListIndex));

            if (packetFieldErrorType != null)
            {
               String messageClassName = message.getClass().getSimpleName();
               String errorMessage = messageClassName + "'s predictedContactPoints field " + packetFieldErrorType.getMessage();
               return errorMessage;
            }
         }
      }

      return null;
   }

   /**
    * Checks the validity of a {@link FootstepStatusMessage}.
    *
    * @param message
    * @return null if the packet is valid, or the error message.
    */
   public static String validateFootstepStatus(FootstepStatusMessage message)
   {
      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateTuple3d(message.getActualFootPositionInWorld());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s actualFootPositionInWorld field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateQuat4d(message.getActualFootOrientationInWorld());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s actualFootOrientationInWorld field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateEnum(FootstepStatus.fromByte(message.getFootstepStatus()));
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s status field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (message.getFootstepIndex() < 0)
      {
         String messageClassName = message.getClass().getSimpleName();
         return messageClassName + ": footstepIndex field should be non-negative";
      }

      return null;
   }

   public static String validateHandTrajectoryMessage(HandTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateSE3TrajectoryMessage(message.getSe3Trajectory());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.getRobotSide()));
      if (errorType != null)
      {
         errorMessage = "robotSide field " + errorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   public static String validateArmTrajectoryMessage(ArmTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateJointspaceTrajectoryMessage(message.getJointspaceTrajectory());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.getRobotSide()));
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + "'s robotSide field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   public static String validateHeadTrajectoryMessage(HeadTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateSO3TrajectoryMessage(message.getSo3Trajectory());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateChestTrajectoryMessage(ChestTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateSO3TrajectoryMessage(message.getSo3Trajectory());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateNeckTrajectoryMessage(NeckTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateJointspaceTrajectoryMessage(message.getJointspaceTrajectory());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateSpineTrajectoryMessage(SpineTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateJointspaceTrajectoryMessage(message.getJointspaceTrajectory());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateJointspaceTrajectoryMessage(JointspaceTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      if (message.getJointTrajectoryMessages() == null)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + "'s trajectory points are empty.";
         return errorMessage;
      }

      int numberOfJoints = message.getJointTrajectoryMessages().size();
      if (numberOfJoints == 0)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + " is empty.";
         return errorMessage;
      }

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointTrajectoryMessage oneJointTrajectoryMessage = message.getJointTrajectoryMessages().get(jointIndex);
         if (oneJointTrajectoryMessage != null)
         {
            errorMessage = validateOneJointTrajectoryMessage(oneJointTrajectoryMessage, false);
         }
         if (errorMessage != null)
         {
            String messageClassName = message.getClass().getSimpleName();
            errorMessage = messageClassName + " Error with the " + jointIndex + " " + OneDoFJointTrajectoryMessage.class.getSimpleName() + " : " + errorMessage;
            return errorMessage;
         }
      }

      return null;
   }

   public static String validateSE3TrajectoryMessage(SE3TrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      if (message.getFrameInformation().getDataReferenceFrameId() == NameBasedHashCodeTools.NULL_HASHCODE)
      {
         return message.getClass().getSimpleName() + " Expressed In Reference Frame Id Not Set";
      }

      if (message.getFrameInformation().getTrajectoryReferenceFrameId() == NameBasedHashCodeTools.NULL_HASHCODE)
      {
         return message.getClass().getSimpleName() + " Trajectory Reference Frame Id Not Set";
      }

      SE3TrajectoryPointMessage previousTrajectoryPoint = null;

      if (message.getTaskspaceTrajectoryPoints().size() == 0)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = "Received " + messageClassName + " with no waypoint.";
         return errorMessage;
      }

      for (int i = 0; i < message.getTaskspaceTrajectoryPoints().size(); i++)
      {
         SE3TrajectoryPointMessage waypoint = message.getTaskspaceTrajectoryPoints().get(i);
         errorMessage = validateSE3TrajectoryPointMessage(waypoint, previousTrajectoryPoint, false);
         if (errorMessage != null)
         {
            String messageClassName = message.getClass().getSimpleName();
            errorMessage = "The " + messageClassName + "'s " + i + "th waypoint " + errorMessage;
            return errorMessage;
         }
         previousTrajectoryPoint = waypoint;
      }

      if (message.getUseCustomControlFrame() && message.getControlFramePose() == null)
      {
         String messageClassName = message.getClass().getSimpleName();
         return "The control frame pose for " + messageClassName + " has to be set to be able to use it.";
      }

      return null;
   }

   public static String validateSO3TrajectoryMessage(SO3TrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return SO3TrajectoryMessage.class + " " + errorMessage;

      SO3TrajectoryPointMessage previousTrajectoryPoint = null;

      if (message.getTaskspaceTrajectoryPoints().size() == 0)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = "Received " + messageClassName + " with no waypoint.";
         return errorMessage;
      }

      if (message.getFrameInformation().getDataReferenceFrameId() == NameBasedHashCodeTools.NULL_HASHCODE)
      {
         return message.getClass().getSimpleName() + " Expressed In Reference Frame Id Not Set";
      }

      if (message.getFrameInformation().getTrajectoryReferenceFrameId() == NameBasedHashCodeTools.NULL_HASHCODE)
      {
         return message.getClass().getSimpleName() + " Trajectory Reference Frame Id Not Set";
      }

      for (int i = 0; i < message.getTaskspaceTrajectoryPoints().size(); i++)
      {
         SO3TrajectoryPointMessage waypoint = message.getTaskspaceTrajectoryPoints().get(i);
         errorMessage = validateSO3TrajectoryPointMessage(waypoint, previousTrajectoryPoint, false);
         if (errorMessage != null)
         {
            String messageClassName = message.getClass().getSimpleName();
            errorMessage = "The " + messageClassName + "'s " + i + "th waypoint " + errorMessage;
            return errorMessage;
         }
         previousTrajectoryPoint = waypoint;
      }

      if (message.getUseCustomControlFrame() && message.getControlFramePose() == null)
      {
         String messageClassName = message.getClass().getSimpleName();
         return "The control frame pose for " + messageClassName + " has to be set to be able to use it.";
      }

      return null;
   }

   public static String validatePelvisOrientationTrajectoryMessage(PelvisOrientationTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateSO3TrajectoryMessage(message.getSo3Trajectory());
      if (errorMessage != null)
         return PelvisOrientationTrajectoryMessage.class.getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validatePelvisTrajectoryMessage(PelvisTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateSE3TrajectoryMessage(message.getSe3Trajectory());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateQuadrupedBodyOrientationMessage(QuadrupedBodyOrientationMessage message)
   {
      String errorMessage = validatePacket(message);

      if (errorMessage == null)
         errorMessage = validateSO3TrajectoryMessage(message.getSo3Trajectory());
      if (errorMessage != null)
         return QuadrupedBodyOrientationMessage.class.getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateFootTrajectoryMessage(FootTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateSE3TrajectoryMessage(message.getSe3Trajectory());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.getRobotSide()));
      if (errorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + "'s robotSide field " + errorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   public static String validateFootLoadBearingMessage(FootLoadBearingMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateEnum(LoadBearingRequest.fromByte(message.getLoadBearingRequest()));
      if (errorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + "'s request field " + errorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   public static String validateGoHomeMessage(GoHomeMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateEnum(HumanoidBodyPart.fromByte(message.getHumanoidBodyPart()));
      if (errorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + "'s endEffector field " + errorType.getMessage();
         return errorMessage;
      }

      if (HumanoidBodyPart.fromByte(message.getHumanoidBodyPart()).isRobotSideNeeded())
      {
         errorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.getRobotSide()));
         if (RobotSide.fromByte(message.getRobotSide()) == null)
         {
            String messageClassName = message.getClass().getSimpleName();
            errorMessage = messageClassName + "'s robotSide field is null. It is required for the bodyPart " + message.getHumanoidBodyPart();
            return errorMessage;
         }
      }

      return null;
   }

   public static String validatePelvisHeightTrajectoryMessage(PelvisHeightTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      EuclideanTrajectoryPointMessage previousTrajectoryPoint = null;

      if (message.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().isEmpty())
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = "Received " + messageClassName + " with no waypoint.";
         return errorMessage;
      }

      for (int i = 0; i < message.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().size(); i++)
      {
         EuclideanTrajectoryPointMessage waypoint = message.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().get(i);
         errorMessage = validateEuclideanTrajectoryPointMessage(waypoint, previousTrajectoryPoint, false);
         if (errorMessage != null)
         {
            String messageClassName = message.getClass().getSimpleName();
            errorMessage = "The " + messageClassName + "'s " + i + "th waypoint " + errorMessage;
            return errorMessage;
         }
         previousTrajectoryPoint = waypoint;
      }

      return null;
   }

   public static String validateQuadrupedBodyHeightMessage(QuadrupedBodyHeightMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      EuclideanTrajectoryPointMessage previousTrajectoryPoint = null;

      if (message.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().isEmpty())
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = "Received " + messageClassName + " with no waypoint.";
         return errorMessage;
      }

      for (int i = 0; i < message.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().size(); i++)
      {
         EuclideanTrajectoryPointMessage waypoint = message.getEuclideanTrajectory().getTaskspaceTrajectoryPoints().get(i);
         errorMessage = validateEuclideanTrajectoryPointMessage(waypoint, previousTrajectoryPoint, false);
         if (errorMessage != null)
         {
            String messageClassName = message.getClass().getSimpleName();
            errorMessage = "The " + messageClassName + "'s " + i + "th waypoint " + errorMessage;
            return errorMessage;
         }
         previousTrajectoryPoint = waypoint;
      }

      return null;
   }

   public static String validateArmDesiredAccelerationsMessage(ArmDesiredAccelerationsMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateDesiredAccelerationsMessage(message.getDesiredAccelerations());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.getRobotSide()));
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + "'s robotSide field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   public static String validateNeckDesiredAccelerationsMessage(NeckDesiredAccelerationsMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateDesiredAccelerationsMessage(message.getDesiredAccelerations());
      if (errorMessage != null)
         return NeckDesiredAccelerationsMessage.class.getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateSpineDesiredAccelerationsMessage(SpineDesiredAccelerationsMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateDesiredAccelerationsMessage(message.getDesiredAccelerations());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateWholeBodyTrajectoryMessage(WholeBodyTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return errorMessage;

      if (!message.getLeftHandTrajectoryMessage().getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
      {
         if ((errorMessage = validateHandTrajectoryMessage(message.getLeftHandTrajectoryMessage())) != null)
            return errorMessage;
         else if (RobotSide.fromByte(message.getLeftHandTrajectoryMessage().getRobotSide()) != RobotSide.LEFT)
            return "The robotSide of leftHandTrajectoryMessage field is inconsistent with its name.";
      }
      if (!message.getLeftHandTrajectoryMessage().getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
      {
         if ((errorMessage = validateHandTrajectoryMessage(message.getRightHandTrajectoryMessage())) != null)
            return errorMessage;
         else if (RobotSide.fromByte(message.getRightHandTrajectoryMessage().getRobotSide()) != RobotSide.RIGHT)
            return "The robotSide of rightHandTrajectoryMessage field is inconsistent with its name.";
      }
      if (!message.getLeftArmTrajectoryMessage().getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
      {
         if ((errorMessage = validateArmTrajectoryMessage(message.getLeftArmTrajectoryMessage())) != null)
            return errorMessage;
         else if (RobotSide.fromByte(message.getLeftArmTrajectoryMessage().getRobotSide()) != RobotSide.LEFT)
            return "The robotSide of leftArmTrajectoryMessage field is inconsistent with its name.";
      }
      if (!message.getRightArmTrajectoryMessage().getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
      {
         if ((errorMessage = validateArmTrajectoryMessage(message.getRightArmTrajectoryMessage())) != null)
            return errorMessage;
         else if (RobotSide.fromByte(message.getRightArmTrajectoryMessage().getRobotSide()) != RobotSide.RIGHT)
            return "The robotSide of rightArmTrajectoryMessage field is inconsistent with its name.";
      }
      if (!message.getChestTrajectoryMessage().getSo3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
      {
         if ((errorMessage = validateChestTrajectoryMessage(message.getChestTrajectoryMessage())) != null)
            return errorMessage;
      }
      if (!message.getPelvisTrajectoryMessage().getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
      {
         if ((errorMessage = validatePelvisTrajectoryMessage(message.getPelvisTrajectoryMessage())) != null)
            return errorMessage;
      }
      if (!message.getHeadTrajectoryMessage().getSo3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
      {
         if ((errorMessage = validateHeadTrajectoryMessage(message.getHeadTrajectoryMessage())) != null)
            return errorMessage;
      }
      if (!message.getLeftFootTrajectoryMessage().getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
      {
         if ((errorMessage = validateFootTrajectoryMessage(message.getLeftFootTrajectoryMessage())) != null)
            return errorMessage;
         else if (RobotSide.fromByte(message.getLeftFootTrajectoryMessage().getRobotSide()) != RobotSide.LEFT)
            return "The robotSide of leftFootTrajectoryMessage field is inconsistent with its name.";
      }
      if (!message.getRightFootTrajectoryMessage().getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
      {
         if ((errorMessage = validateFootTrajectoryMessage(message.getRightFootTrajectoryMessage())) != null)
            return errorMessage;
         else if (RobotSide.fromByte(message.getRightFootTrajectoryMessage().getRobotSide()) != RobotSide.RIGHT)
            return "The robotSide of rightFootTrajectoryMessage field is inconsistent with its name.";
      }

      return null;
   }

   private static String validateSE3TrajectoryPointMessage(SE3TrajectoryPointMessage se3TrajectoryPoint, SE3TrajectoryPointMessage previousSE3TrajectoryPoint,
                                                           boolean checkId)
   {
      String errorMessage = validatePacket(se3TrajectoryPoint);
      if (errorMessage != null)
         return errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateTuple3d(se3TrajectoryPoint.getPosition());
      if (errorType != null)
         return "SE3 waypoint position field " + errorType.getMessage();

      errorType = ObjectValidityChecker.validateQuat4d(se3TrajectoryPoint.getOrientation());
      if (errorType != null)
         return "SE3 waypoint orientation field " + errorType.getMessage();

      errorType = ObjectValidityChecker.validateTuple3d(se3TrajectoryPoint.getLinearVelocity());
      if (errorType != null)
         return "SE3 waypoint linear velocity field " + errorType.getMessage();

      errorType = ObjectValidityChecker.validateTuple3d(se3TrajectoryPoint.getAngularVelocity());
      if (errorType != null)
         return "SE3 waypoint angular velocity field " + errorType.getMessage();
      ;

      double subTrajectoryTime = se3TrajectoryPoint.getTime();
      if (previousSE3TrajectoryPoint != null)
         subTrajectoryTime -= previousSE3TrajectoryPoint.getTime();

      errorType = ObjectValidityChecker.validateTrajectoryTime(subTrajectoryTime);
      if (errorType != null)
         return "SE3 waypoint time (relative to previous waypoint) " + errorType.getMessage();

      return null;
   }

   private static String validateEuclideanTrajectoryPointMessage(EuclideanTrajectoryPointMessage se3TrajectoryPoint,
                                                                 EuclideanTrajectoryPointMessage previousTrajectoryPoint, boolean checkId)
   {
      String errorMessage = validatePacket(se3TrajectoryPoint);
      if (errorMessage != null)
         return errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateTuple3d(se3TrajectoryPoint.getPosition());
      if (errorType != null)
         return "SE3 waypoint position field " + errorType.getMessage();

      errorType = ObjectValidityChecker.validateTuple3d(se3TrajectoryPoint.getLinearVelocity());
      if (errorType != null)
         return "SE3 waypoint linear velocity field " + errorType.getMessage();

      double subTrajectoryTime = se3TrajectoryPoint.getTime();
      if (previousTrajectoryPoint != null)
         subTrajectoryTime -= previousTrajectoryPoint.getTime();

      errorType = ObjectValidityChecker.validateTrajectoryTime(subTrajectoryTime);
      if (errorType != null)
         return "SE3 waypoint time (relative to previous waypoint) " + errorType.getMessage();

      return null;
   }


   private static String validateSO3TrajectoryPointMessage(SO3TrajectoryPointMessage so3TrajectoryPoint, SO3TrajectoryPointMessage previousSO3TrajectoryPoint,
                                                           boolean checkId)
   {
      String errorMessage = validatePacket(so3TrajectoryPoint);
      if (errorMessage != null)
         return errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateQuat4d(so3TrajectoryPoint.getOrientation());
      if (errorType != null)
         return "SO3 waypoint orientation field " + errorType.getMessage();

      errorType = ObjectValidityChecker.validateTuple3d(so3TrajectoryPoint.getAngularVelocity());
      if (errorType != null)
         return "SO3 waypoint angular velocity field " + errorType.getMessage();

      double subTrajectoryTime = so3TrajectoryPoint.getTime();
      if (previousSO3TrajectoryPoint != null)
         subTrajectoryTime -= previousSO3TrajectoryPoint.getTime();

      errorType = ObjectValidityChecker.validateTrajectoryTime(subTrajectoryTime);
      if (errorType != null)
         return "SO3 waypoint time (relative to previous waypoint) " + errorType.getMessage();

      return null;
   }

   private static String validateTrajectoryPoint1DMessage(TrajectoryPoint1DMessage waypoint1D, TrajectoryPoint1DMessage previousTrajectoryPoint1D,
                                                          boolean checkId)
   {
      String errorMessage = validatePacket(waypoint1D);
      if (errorMessage != null)
         return errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateDouble(waypoint1D.getPosition());
      if (errorType != null)
         return "1D waypoint orientation field " + errorType.getMessage();

      errorType = ObjectValidityChecker.validateDouble(waypoint1D.getVelocity());
      if (errorType != null)
         return "1D waypoint angular velocity field " + errorType.getMessage();

      double subTrajectoryTime = waypoint1D.getTime();
      if (previousTrajectoryPoint1D != null)
         subTrajectoryTime -= previousTrajectoryPoint1D.getTime();

      errorType = ObjectValidityChecker.validateTrajectoryTime(subTrajectoryTime);
      if (errorType != null)
         return "1D waypoint time (relative to previous waypoint) " + errorType.getMessage();

      return null;
   }

   private final static double MAX_ACCEPTED_JOINT_VELOCITY = 100.0;

   public static String validateOneJointTrajectoryMessage(OneDoFJointTrajectoryMessage message, boolean checkId)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return errorMessage;

      TrajectoryPoint1DMessage previousTrajectoryPoint = null;

      if (message.getTrajectoryPoints().size() == 0)
         return "The joint trajectory message has no waypoint.";

      for (int i = 0; i < message.getTrajectoryPoints().size(); i++)
      {
         TrajectoryPoint1DMessage waypoint = message.getTrajectoryPoints().get(i);
         errorMessage = validateTrajectoryPoint1DMessage(waypoint, previousTrajectoryPoint, false);
         if (errorMessage != null)
            return "The " + i + "th " + errorMessage;
         previousTrajectoryPoint = waypoint;
      }

      for (int waypointIndex = 0; waypointIndex < message.getTrajectoryPoints().size(); waypointIndex++)
      {
         TrajectoryPoint1DMessage waypoint = message.getTrajectoryPoints().get(waypointIndex);
         double waypointPosition = waypoint.getPosition();

         if (Math.abs(waypointPosition) > Math.PI)
            return "The " + waypointIndex + "th waypoint position is unreasonable: " + waypointPosition;

         double waypointVelocity = waypoint.getVelocity();

         if (Math.abs(waypointVelocity) > MAX_ACCEPTED_JOINT_VELOCITY)
            return "The " + waypointIndex + "th waypoint velocity is unreasonable: " + waypointVelocity;
      }

      return null;
   }

   public static String validatePacket(Packet<?> packet)
   {
      if (packet == null)
         return "is null.";
      return null;
   }

   public static String validateDesiredAccelerationsMessage(DesiredAccelerationsMessage message)
   {
      if (message == null)
         return "is null.";
      if (message.getDesiredJointAccelerations() == null)
      {
         return "desired acceleration buffer null";
      }
      if (message.getDesiredJointAccelerations().size() == 0)
      {
         return "desired acceleration buffer empty";
      }
      return null;
   }
}
