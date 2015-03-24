package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packetConsumers.ObjectValidityChecker.ObjectErrorType;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.DataType;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.communication.packets.walking.FootPosePacket;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.packets.walking.FootstepStatus;
import us.ihmc.communication.packets.walking.HeadOrientationPacket;

public abstract class PacketValidityChecker
{

   /**
    * Checks the validity of a {@link ComHeightPacket}.
    * @param packetToCheck
    * @return null if the packet is valid, or the error message.
    */
   public static String validateCoMHeightPacket(ComHeightPacket packetToCheck)
   {
      if (packetToCheck == null)
         return null;
      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateDouble(packetToCheck.getHeightOffset());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "heightOffset field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateTrajectoryTime(packetToCheck.getTrajectoryTime());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "trajectoryTime field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   /**
    * Checks the validity of a {@link HandPosePacket}.
    * @param packetToCheck
    * @param numberOfArmJoints
    * @return null if the packet is valid, or the error message.
    */
   public static String validateHandPosePacket(HandPosePacket packetToCheck, int numberOfArmJoints)
   {
      if (packetToCheck == null)
         return null;

      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateEnum(packetToCheck.getDataType());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "dataType field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (packetToCheck.getDataType() == DataType.HAND_POSE)
      {
         packetFieldErrorType = ObjectValidityChecker.validateTuple3d(packetToCheck.getPosition());
         if (packetFieldErrorType != null)
         {
            String errorMessage = "position field " + packetFieldErrorType.getMessage();
            return errorMessage;
         }

         packetFieldErrorType = ObjectValidityChecker.validateTuple4d(packetToCheck.getOrientation());
         if (packetFieldErrorType != null)
         {
            String errorMessage = "orientation field " + packetFieldErrorType.getMessage();
            return errorMessage;
         }

         packetFieldErrorType = ObjectValidityChecker.validateEnum(packetToCheck.getReferenceFrame());
         if (packetFieldErrorType != null)
         {
            String errorMessage = "frame field" + packetFieldErrorType.getMessage();
            return errorMessage;
         }
      }
      else
      {
         packetFieldErrorType = ObjectValidityChecker.validateArrayOfDouble(packetToCheck.getJointAngles(), numberOfArmJoints);
         if (packetFieldErrorType != null)
         {
            String errorMessage = "jointAngles field " + packetFieldErrorType.getMessage();
            return errorMessage;
         }
      }

      packetFieldErrorType = ObjectValidityChecker.validateTrajectoryTime(packetToCheck.getTrajectoryTime());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "trajectoryTime field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateEnum(packetToCheck.getRobotSide());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "robotSide field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      return null;
   }
   
   /**
    * Checks the validity of a {@link FootPosePacket}.
    * @param packetToCheck
    * @return null if the packet is valid, or the error message.
    */
   public static String validateFootPosePacket(FootPosePacket packetToCheck)
   {
      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateTuple3d(packetToCheck.getPosition());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "position field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateTuple4d(packetToCheck.getOrientation());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "orientation field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateTrajectoryTime(packetToCheck.getTrajectoryTime());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "trajectoryTime field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateEnum(packetToCheck.getRobotSide());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "robotSide field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   /**
    * Checks the validity of a {@link FootstepData}.
    * @param packetToCheck
    * @return null if the packet is valid, or the error message.
    */
   public static String validateFootstepData(FootstepData packetToCheck)
   {
      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateEnum(packetToCheck.getRobotSide());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "robotSide field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateTuple3d(packetToCheck.getLocation());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "location field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateTuple4d(packetToCheck.getOrientation());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "orientation field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (packetToCheck.getPredictedContactPoints() != null)
      {
         for (int arrayListIndex = 0; arrayListIndex < packetToCheck.getPredictedContactPoints().size(); arrayListIndex++)
         {
            packetFieldErrorType = ObjectValidityChecker.validateTuple2d(packetToCheck.getPredictedContactPoints().get(arrayListIndex));

            if (packetFieldErrorType != null)
            {
               String errorMessage = "predictedContactPoints field " + packetFieldErrorType.getMessage();
               return errorMessage;
            }

         }
      }

      packetFieldErrorType = ObjectValidityChecker.validateEnum(packetToCheck.getTrajectoryType());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "trajectoryType field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      //TODO Check if thats supposed to be checked
      packetFieldErrorType = ObjectValidityChecker.validateDouble(packetToCheck.getSwingHeight());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "swingHeight field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   /**
    * Checks the validity of a {@link FootstepDataList}.
    * @param packetToCheck
    * @return null if the packet is valid, or the error message.
    */
   public static String validateFootstepDataList(FootstepDataList packetToCheck)
   {
      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateDouble(packetToCheck.swingTime);
      if (packetFieldErrorType != null)
      {
         String errorMessage = "swingTime field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateDouble(packetToCheck.transferTime);
      if (packetFieldErrorType != null)
      {
         String errorMessage = "transferTime field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (packetToCheck.getDataList() != null)
      {
         for (int arrayListIndex = 0; arrayListIndex < packetToCheck.getDataList().size(); arrayListIndex++)
         {
            String footstepDataListErrorMessage = validateFootstepData(packetToCheck.getDataList().get(arrayListIndex));

            if (footstepDataListErrorMessage != null)
            {
               String errorMessage = "footstepDataList field contains a FootstepData in which " + footstepDataListErrorMessage;
               return errorMessage;
            }

         }
      }

      return null;
   }

   /**
    * Checks the validity of a {@link FootstepStatus}.
    * @param packetToCheck
    * @return null if the packet is valid, or the error message.
    */
   public static String validateFootstepStatus(FootstepStatus packetToCheck)
   {
      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateTuple3d(packetToCheck.getActualFootPositionInWorld());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "actualFootPositionInWorld field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateTuple4d(packetToCheck.getActualFootOrientationInWorld());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "actualFootOrientationInWorld field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateEnum(packetToCheck.getStatus());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "status field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (packetToCheck.getFootstepIndex() < 0)
         return "footstepIndex field should be positive";

      return null;
   }

   /**
    * Checks the validity of a {@link ChestOrientationPacket}.
    * @param packetToCheck
    * @return null if the packet is valid, or the error message.
    */
   public static String validateChestOrientationPacket(ChestOrientationPacket packetToCheck)
   {
      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateTuple4d(packetToCheck.getOrientation());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "orientation field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateTrajectoryTime(packetToCheck.getTrajectoryTime());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "trajectoryTime field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   /**
    * Checks the validity of a {@link HeadOrientationPacket}.
    * @param packetToCheck
    * @return null if the packet is valid, or the error message.
    */
   public static String validateHeadOrientationPacket(HeadOrientationPacket packetToCheck)
   {
      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateTuple4d(packetToCheck.getOrientation());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "quaternion field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateTrajectoryTime(packetToCheck.getTrajectoryTime());
      if (packetFieldErrorType != null)
      {
         String errorMessage = "trajectoryTime field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      return null;
   }
}
