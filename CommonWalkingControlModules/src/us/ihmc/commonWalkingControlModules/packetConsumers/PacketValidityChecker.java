package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicInteger;

import us.ihmc.commonWalkingControlModules.packetConsumers.ObjectValidityChecker.ObjectErrorType;
import us.ihmc.humanoidRobotics.communication.packets.SE3WaypointMessage;
import us.ihmc.humanoidRobotics.communication.packets.SO3WaypointMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.DesiredSteeringAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPosePacket.DataType;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.JointTrajectoryPoint;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.SteeringWheelInformationPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ComHeightPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataList;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadOrientationPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.robotSide.SideDependentList;

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

      if (!packetToCheck.isToHomePosition())
      {
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
      }

      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateTrajectoryTime(packetToCheck.getTrajectoryTime());
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
         return "footstepIndex field should be non-negative";

      return null;
   }

   /**
    * Checks the validity of a {@link ChestOrientationPacket}.
    * @param packetToCheck
    * @return null if the packet is valid, or the error message.
    */
   public static String validateChestOrientationPacket(ChestOrientationPacket packetToCheck)
   {
      // In this case the controller doesn't read the orientation
      if (!packetToCheck.isToHomeOrientation())
      {
         ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateTuple4d(packetToCheck.getOrientation());
         if (packetFieldErrorType != null)
         {
            String errorMessage = "orientation field " + packetFieldErrorType.getMessage();
            return errorMessage;
         }
      }

      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateTrajectoryTime(packetToCheck.getTrajectoryTime());
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
   
   /**
    * Checks the validity of a {@link ArmJointTrajectoryPacket}.
    * @param packetToCheck
    * @return null if the packet is valid, or the error message.
    */
   public static String validateArmJointTrajectoryPacket(ArmJointTrajectoryPacket packetToCheck)
   {
      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateEnum(packetToCheck.robotSide);
      if (packetFieldErrorType != null)
      {
         String errorMessage = "robotSide field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }
      
      if (packetToCheck.trajectoryPoints == null)
      {
         String errorMessage = "Trajectory pointes are empty.";
         return errorMessage;
      }
      
      int waypoints = packetToCheck.trajectoryPoints.length;
      if (waypoints == 0)
      {
         String errorMessage = "ArmJointTrajectoryPacket does not contain any points";
         return errorMessage;
      }
      
      double prevTime = 0.0;
      int joints = packetToCheck.trajectoryPoints[0].positions.length;
      
      for (int i = 0; i < waypoints; i++)
      {
         JointTrajectoryPoint trajectoryPoint = packetToCheck.trajectoryPoints[i];
         String errorMessage = validateTrajectoryPointPacket(trajectoryPoint);
         if (errorMessage != null)
         {
            return "ArmJointTrajectoryPacket: waypoint " + i + " - " + errorMessage;
         }
         
         if (trajectoryPoint.time <= prevTime)
         {
            return "ArmJointTrajectoryPacket: waypoint " + i + " has invalid time - needs to be bigger then " + prevTime;
         }
         prevTime = trajectoryPoint.time;
         
         if (trajectoryPoint.positions.length != joints || trajectoryPoint.velocities.length != joints)
         {
            return "ArmJointTrajectoryPacket contains waypoints with inconsistent number of joints";
         }
      }

      return null;
   }
   
   private final static double MAX_ACCEPTED_JOINT_VELOCITY = 100.0;
   /**
    * Checks the validity of a {@link TrajectoryPoint}.
    * @param packetToCheck
    * @return null if the packet is valid, or the error message.
    */
   public static String validateTrajectoryPointPacket(JointTrajectoryPoint packetToCheck)
   {
      ObjectErrorType errorTime = ObjectValidityChecker.validateTrajectoryTime(packetToCheck.time);
      if (errorTime != null)
      {
         return errorTime.getMessage();
      }
      
      int joints = packetToCheck.positions.length;
      if (packetToCheck.velocities.length != joints)
      {
         return "inconstistent size of position and velocity arrays";
      }
      
      ObjectErrorType errorPos = ObjectValidityChecker.validateArrayOfDouble(packetToCheck.positions, joints);
      if (errorPos != null)
      {
         return "positions array " + errorPos.getMessage();
      }
      
      ObjectErrorType errorVel = ObjectValidityChecker.validateArrayOfDouble(packetToCheck.velocities, joints);
      if (errorVel != null)
      {
         return "velocities array " + errorVel.getMessage();
      }
      
      for (int i = 0; i < joints; i++)
      {
         if (Math.abs(packetToCheck.positions[i]) > Math.PI)
         {
            return "joint angle values between -pi and pi expected for joint #" + i + " was " + packetToCheck.positions[i];
         }
         if (Math.abs(packetToCheck.velocities[i]) > MAX_ACCEPTED_JOINT_VELOCITY)
         {
            return "joint amgular velocity unreasonably high for joint #" + i + " was " + packetToCheck.velocities[i]
                  + " expecting less then " + MAX_ACCEPTED_JOINT_VELOCITY;
         }
      }
      
      return null;
   }

   public static boolean validateSteeringWheelInformationPacket(SteeringWheelInformationPacket packet, SideDependentList<AtomicInteger> steeringWheelIdAtomic, HumanoidGlobalDataProducer globalDataProducer)
   {
      boolean packetIsValid = true;

      if (packet.getRobotSide() == null)
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(SteeringWheelInformationPacket.class, "Steering hand side missing");
      }

      if (packet.getSteeringWheelCenter() == null)
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(SteeringWheelInformationPacket.class, "Steering wheel center missing");
      }

      if (packet.getSteeringWheelRotationAxis() == null)
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(SteeringWheelInformationPacket.class, "Steering wheel rotation axis missing");
      }

      if (packet.getSteeringWheelZeroAxis() == null)
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(SteeringWheelInformationPacket.class, "Steering wheel zero axis missing");
      }

      if (Double.isNaN(packet.getSteeringWheelRadius()))
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(SteeringWheelInformationPacket.class, "Steering wheel radius missing");
      }

      if (packet.getSteeringWheelId() <= 0)
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(SteeringWheelInformationPacket.class, "Invalid steering wheel ID, must be greater than or equal to 1");
      }

      if (packet.getSteeringWheelId() == steeringWheelIdAtomic.get(packet.getRobotSide()).get())
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(SteeringWheelInformationPacket.class,
               "Invalid steering wheel ID, must be different than the previous ID");
      }

      ObjectErrorType errorType = ObjectValidityChecker.validateTuple3d(packet.getSteeringWheelCenter());

      if (errorType != null)
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(SteeringWheelInformationPacket.class, "Steering wheel center " + errorType.getMessage());
      }

      errorType = ObjectValidityChecker.validateTuple3d(packet.getSteeringWheelRotationAxis());

      if (errorType != null)
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(SteeringWheelInformationPacket.class, "Steering wheel rotation axis " + errorType.getMessage());
      }

      errorType = ObjectValidityChecker.validateTuple3d(packet.getSteeringWheelCenter());

      if (errorType != null)
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(SteeringWheelInformationPacket.class, "Steering wheel zero axis " + errorType.getMessage());
      }

      return packetIsValid;
   }

   public static boolean validateDesiredSteeringAnglePacket(DesiredSteeringAnglePacket packet, SideDependentList<AtomicInteger> steeringWheelIdAtomic, HumanoidGlobalDataProducer globalDataProducer)
   {
      boolean packetIsValid = true;

      if (Double.isNaN(packet.getDesiredAbsoluteSteeringAngle()))
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(DesiredSteeringAnglePacket.class, "Desired steering angle missing");
      }

      if (steeringWheelIdAtomic.get(packet.getRobotSide()).get() == -1)
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(DesiredSteeringAnglePacket.class, "Never received SteeringWheelInformationPacket");
      }

      if (packet.getSteeringWheelId() <= 0)
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(DesiredSteeringAnglePacket.class, "Invalid steering wheel ID, must be greater than or equal to 1");
      }

      if (packet.getSteeringWheelId() != steeringWheelIdAtomic.get(packet.getRobotSide()).get())
      {
         packetIsValid = false;
         globalDataProducer.notifyInvalidPacketReceived(DesiredSteeringAnglePacket.class,
               "Unexpected steering wheel ID, probably dropped the last SteeringWheelInformationPacket");
      }

      return packetIsValid;
   }

   public static boolean validateHandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage, HumanoidGlobalDataProducer globalDataProducer)
   {
      if (handTrajectoryMessage == null)
         return false;

      ObjectErrorType errorType;
      SE3WaypointMessage previousWaypoint = null;

      if (handTrajectoryMessage.getNumberOfWaypoints() == 0)
      {
         String errorMessage = "Received trajectory message with no waypoint.";
         globalDataProducer.notifyInvalidPacketReceived(handTrajectoryMessage.getClass(), errorMessage);
      }

      for (int i = 0; i < handTrajectoryMessage.getNumberOfWaypoints(); i++)
      {
         SE3WaypointMessage waypoint = handTrajectoryMessage.getWaypoint(i);
         String errorMessage = validateSE3Waypoint(waypoint, previousWaypoint);
         if (errorMessage != null)
         {
            errorMessage += "The " + i + "th";
            globalDataProducer.notifyInvalidPacketReceived(handTrajectoryMessage.getClass(), errorMessage);
            return false;
         }
      }

      errorType = ObjectValidityChecker.validateEnum(handTrajectoryMessage.getRobotSide());
      if (errorType != null)
      {
         String errorMessage = "robotSide field " + errorType.getMessage();
         globalDataProducer.notifyInvalidPacketReceived(handTrajectoryMessage.getClass(), errorMessage);
         return false;
      }

      return true;
   }

   public static boolean validateHeadTrajectoryMessage(HeadTrajectoryMessage headTrajectoryMessage, HumanoidGlobalDataProducer globalDataProducer)
   {
      if (headTrajectoryMessage == null)
         return false;

      SO3WaypointMessage previousWaypoint = null;

      if (headTrajectoryMessage.getNumberOfWaypoints() == 0)
      {
         String errorMessage = "Received trajectory message with no waypoint.";
         globalDataProducer.notifyInvalidPacketReceived(headTrajectoryMessage.getClass(), errorMessage);
      }

      for (int i = 0; i < headTrajectoryMessage.getNumberOfWaypoints(); i++)
      {
         SO3WaypointMessage waypoint = headTrajectoryMessage.getWaypoint(i);
         String errorMessage = validateSO3Waypoint(waypoint, previousWaypoint);
         if (errorMessage != null)
         {
            errorMessage += "The " + i + "th";
            globalDataProducer.notifyInvalidPacketReceived(headTrajectoryMessage.getClass(), errorMessage);
            return false;
         }
      }

      return true;
   }

   public static boolean validateChestTrajectoryMessage(ChestTrajectoryMessage chestTrajectoryMessage, HumanoidGlobalDataProducer globalDataProducer)
   {
      if (chestTrajectoryMessage == null)
         return false;

      SO3WaypointMessage previousWaypoint = null;

      if (chestTrajectoryMessage.getNumberOfWaypoints() == 0)
      {
         String errorMessage = "Received trajectory message with no waypoint.";
         globalDataProducer.notifyInvalidPacketReceived(chestTrajectoryMessage.getClass(), errorMessage);
      }

      for (int i = 0; i < chestTrajectoryMessage.getNumberOfWaypoints(); i++)
      {
         SO3WaypointMessage waypoint = chestTrajectoryMessage.getWaypoint(i);
         String errorMessage = validateSO3Waypoint(waypoint, previousWaypoint);
         if (errorMessage != null)
         {
            errorMessage += "The " + i + "th";
            globalDataProducer.notifyInvalidPacketReceived(chestTrajectoryMessage.getClass(), errorMessage);
            return false;
         }
      }

      return true;
   }

   public static boolean validatePelvisTrajectoryMessage(PelvisTrajectoryMessage pelvisTrajectoryMessage, HumanoidGlobalDataProducer globalDataProducer)
   {
      if (pelvisTrajectoryMessage == null)
         return false;

      SE3WaypointMessage previousWaypoint = null;

      if (pelvisTrajectoryMessage.getNumberOfWaypoints() == 0)
      {
         String errorMessage = "Received trajectory message with no waypoint.";
         globalDataProducer.notifyInvalidPacketReceived(pelvisTrajectoryMessage.getClass(), errorMessage);
      }

      for (int i = 0; i < pelvisTrajectoryMessage.getNumberOfWaypoints(); i++)
      {
         SE3WaypointMessage waypoint = pelvisTrajectoryMessage.getWaypoint(i);
         String errorMessage = validateSE3Waypoint(waypoint, previousWaypoint);
         if (errorMessage != null)
         {
            errorMessage += "The " + i + "th";
            globalDataProducer.notifyInvalidPacketReceived(pelvisTrajectoryMessage.getClass(), errorMessage);
            return false;
         }
      }

      return true;
   }

   public static boolean validateFootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage, HumanoidGlobalDataProducer globalDataProducer)
   {
      if (footTrajectoryMessage == null)
         return false;

      ObjectErrorType errorType;
      SE3WaypointMessage previousWaypoint = null;

      if (footTrajectoryMessage.getNumberOfWaypoints() == 0)
      {
         String errorMessage = "Received trajectory message with no waypoint.";
         globalDataProducer.notifyInvalidPacketReceived(footTrajectoryMessage.getClass(), errorMessage);
      }

      for (int i = 0; i < footTrajectoryMessage.getNumberOfWaypoints(); i++)
      {
         SE3WaypointMessage waypoint = footTrajectoryMessage.getWaypoint(i);
         String errorMessage = validateSE3Waypoint(waypoint, previousWaypoint);
         if (errorMessage != null)
         {
            errorMessage += "The " + i + "th";
            globalDataProducer.notifyInvalidPacketReceived(footTrajectoryMessage.getClass(), errorMessage);
            return false;
         }
      }

      errorType = ObjectValidityChecker.validateEnum(footTrajectoryMessage.getRobotSide());
      if (errorType != null)
      {
         String errorMessage = "robotSide field " + errorType.getMessage();
         globalDataProducer.notifyInvalidPacketReceived(footTrajectoryMessage.getClass(), errorMessage);
         return false;
      }

      return true;
   }

   public static boolean validateEndEffectorLoadBearingMessage(EndEffectorLoadBearingMessage endEffectorLoadBearingMessage, HumanoidGlobalDataProducer globalDataProducer)
   {
      if (endEffectorLoadBearingMessage == null)
         return false;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateEnum(endEffectorLoadBearingMessage.getEndEffector());
      if (errorType != null)
      {
         String errorMessage = "endEffector field " + errorType.getMessage();
         globalDataProducer.notifyInvalidPacketReceived(endEffectorLoadBearingMessage.getClass(), errorMessage);
         return false;
      }

      if (endEffectorLoadBearingMessage.getEndEffector().isRobotSideNeeded())
      {
         errorType = ObjectValidityChecker.validateEnum(endEffectorLoadBearingMessage.getRobotSide());
         if (endEffectorLoadBearingMessage.getRobotSide() == null)
         {
            String errorMessage = "robotSide field is null. It is required for the endEffector " + endEffectorLoadBearingMessage.getEndEffector();
            globalDataProducer.notifyInvalidPacketReceived(endEffectorLoadBearingMessage.getClass(), errorMessage);
            return false;
         }
      }

      return true;
   }

   private static String validateSE3Waypoint(SE3WaypointMessage se3Waypoint, SE3WaypointMessage previousSE3Waypoint)
   {
      if (se3Waypoint == null)
         return " is null.";

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateTuple3d(se3Waypoint.getPosition());
      if (errorType != null)
      {
         String errorMessage = "SE3 waypoint position field " + errorType.getMessage();
         return errorMessage;
      }

      errorType = ObjectValidityChecker.validateTuple4d(se3Waypoint.getOrientation());
      if (errorType != null)
      {
         String errorMessage = "SE3 waypoint orientation field " + errorType.getMessage();
         return errorMessage;
      }

      errorType = ObjectValidityChecker.validateTuple3d(se3Waypoint.getLinearVelocity());
      if (errorType != null)
      {
         String errorMessage = "SE3 waypoint linear velocity field " + errorType.getMessage();
         return errorMessage;
      }

      errorType = ObjectValidityChecker.validateTuple3d(se3Waypoint.getAngularVelocity());
      if (errorType != null)
      {
         String errorMessage = "SE3 waypoint angular velocity field " + errorType.getMessage();
         return errorMessage;
      }

      double subTrajectoryTime = se3Waypoint.getTime();
      if (previousSE3Waypoint != null)
         subTrajectoryTime -= previousSE3Waypoint.getTime();
         
      errorType = ObjectValidityChecker.validateTrajectoryTime(subTrajectoryTime);
      if (errorType != null)
      {
         String errorMessage = "SE3 waypoint time (relative to previous waypoint) " + errorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   private static String validateSO3Waypoint(SO3WaypointMessage so3Waypoint, SO3WaypointMessage previousSO3Waypoint)
   {
      if (so3Waypoint == null)
         return " is null.";

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateTuple4d(so3Waypoint.getOrientation());
      if (errorType != null)
      {
         String errorMessage = "SO3 waypoint orientation field " + errorType.getMessage();
         return errorMessage;
      }

      errorType = ObjectValidityChecker.validateTuple3d(so3Waypoint.getAngularVelocity());
      if (errorType != null)
      {
         String errorMessage = "SO3 waypoint angular velocity field " + errorType.getMessage();
         return errorMessage;
      }

      double subTrajectoryTime = so3Waypoint.getTime();
      if (previousSO3Waypoint != null)
         subTrajectoryTime -= previousSO3Waypoint.getTime();
         
      errorType = ObjectValidityChecker.validateTrajectoryTime(subTrajectoryTime);
      if (errorType != null)
      {
         String errorMessage = "SO3 waypoint time (relative to previous waypoint) " + errorType.getMessage();
         return errorMessage;
      }

      return null;
   }
}
