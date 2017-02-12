package us.ihmc.avatar.ros;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;

import geometry_msgs.Vector3;
import ihmc_msgs.ArmTrajectoryRosMessage;
import ihmc_msgs.ChestTrajectoryRosMessage;
import ihmc_msgs.FootTrajectoryRosMessage;
import ihmc_msgs.FootstepDataListRosMessage;
import ihmc_msgs.FootstepDataRosMessage;
import ihmc_msgs.HandTrajectoryRosMessage;
import ihmc_msgs.PelvisTrajectoryRosMessage;
import ihmc_msgs.Point2dRosMessage;
import ihmc_msgs.WholeBodyTrajectoryRosMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericROSTranslationTools;
import us.ihmc.utilities.ros.msgToPacket.converter.RosEnumConversionException;

public class IHMCROSTranslationRuntimeTools
{
   private static final MessageFactory messageFactory = GenericROSTranslationTools.getMessageFactory();

   public static Message convertToRosMessage(Packet<?> ihmcMessage)
         throws InvocationTargetException, IllegalAccessException, NoSuchMethodException, ClassNotFoundException
   {
      if(ihmcMessage == null)
      {
         return null;
      }
      Class<? extends Packet> aClass = ihmcMessage.getClass();
      try
      {
         Method convertToRosMessageMethod = IHMCROSTranslationRuntimeTools.class.getDeclaredMethod("customConvertToRosMessage", aClass);
         convertToRosMessageMethod.setAccessible(true);
         return (Message) convertToRosMessageMethod.invoke(null, ihmcMessage);
      }
      catch (NoSuchMethodException exception)
      {
         return GenericROSTranslationTools.convertIHMCMessageToRosMessage(ihmcMessage);
      }
   }

   public static Packet<?> convertToIHMCMessage(Message rosMessage)
         throws ClassNotFoundException, InvocationTargetException, IllegalAccessException, RosEnumConversionException, NoSuchFieldException,
         InstantiationException
   {
      if(rosMessage == null)
      {
         return null;
      }
      Class<?> aClass = Class.forName(rosMessage.toRawMessage().getType().replace("/", "."));

      try
      {
         Method convertToIHMCMessageMethod = IHMCROSTranslationRuntimeTools.class.getDeclaredMethod("customConvertToIHMCMessage", aClass);
         convertToIHMCMessageMethod.setAccessible(true);
         return (Packet<?>) convertToIHMCMessageMethod.invoke(null, rosMessage);
      }
      catch (NoSuchMethodException exception)
      {
         return GenericROSTranslationTools.convertRosMessageToIHMCMessage(rosMessage);
      }
   }

   private static Packet customConvertToIHMCMessage(FootstepDataListRosMessage message) throws Exception
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      footsteps.defaultSwingTime = message.getDefaultSwingTime();
      footsteps.defaultTransferTime = message.getDefaultTransferTime();
      footsteps.setUniqueId(message.getUniqueId());
      footsteps.executionMode = ExecutionMode.values[message.getExecutionMode()];
      footsteps.finalTransferTime = message.getFinalTransferTime();

      ArrayList<FootstepDataMessage> stepData = new ArrayList<>();
      for (FootstepDataRosMessage footstepDataRosMessage : message.getFootstepDataList())
      {
         stepData.add((FootstepDataMessage) convertToIHMCMessage(footstepDataRosMessage));
      }

      footsteps.footstepDataList = stepData;

      return footsteps;
   }

   private static Packet customConvertToIHMCMessage(FootstepDataRosMessage message) throws Exception
   {
      FootstepDataMessage ihmcMessage = new FootstepDataMessage();

      ihmcMessage.setOrigin(FootstepDataMessage.FootstepOrigin.values()[message.getOrigin()]);
      ihmcMessage.setRobotSide(RobotSide.values[message.getRobotSide()]);
      ihmcMessage.setLocation(new Point3d(GenericROSTranslationTools.convertVector3(message.getLocation())));
      ihmcMessage.setOrientation(new Quat4d(GenericROSTranslationTools.convertQuaternion(message.getOrientation())));
      ihmcMessage.setSwingHeight(message.getSwingHeight());
      ihmcMessage.setTrajectoryType(TrajectoryType.values()[message.getTrajectoryType()]);
      ihmcMessage.setUniqueId(message.getUniqueId());

      if (message.getHasTimings())
      {
         ihmcMessage.setTimings(message.getSwingTime(), message.getTransferTime());
      }

      if (message.getHasAbsoluteTime())
      {
         ihmcMessage.setAbsoluteTime(message.getSwingStartTime());
      }

      ArrayList<Point2d> predictedContactPoints = new ArrayList<>();
      for (Point2dRosMessage point2dRosMessage : message.getPredictedContactPoints())
      {
         predictedContactPoints.add(GenericROSTranslationTools.convertPoint2DRos(point2dRosMessage));
      }

      Point3d[] trajectoryWaypoints = new Point3d[message.getTrajectoryWaypoints().size()];
      for (int i = 0; i < message.getTrajectoryWaypoints().size(); i++)
      {
         trajectoryWaypoints[i] = new Point3d(GenericROSTranslationTools.convertVector3(message.getTrajectoryWaypoints().get(i)));
      }

      ihmcMessage.setPredictedContactPoints(predictedContactPoints);
      ihmcMessage.setTrajectoryWaypoints(trajectoryWaypoints);

      return ihmcMessage;
   }

   private static Packet customConvertToIHMCMessage(WholeBodyTrajectoryRosMessage message) throws Exception
   {
      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();

      wholeBodyTrajectoryMessage.setUniqueId(message.getUniqueId());
      wholeBodyTrajectoryMessage.leftArmTrajectoryMessage = (ArmTrajectoryMessage) convertToIHMCMessage(message.getLeftArmTrajectoryMessage());
      wholeBodyTrajectoryMessage.rightArmTrajectoryMessage = (ArmTrajectoryMessage) convertToIHMCMessage(message.getRightArmTrajectoryMessage());
      wholeBodyTrajectoryMessage.leftHandTrajectoryMessage = (HandTrajectoryMessage) convertToIHMCMessage(message.getLeftHandTrajectoryMessage());
      wholeBodyTrajectoryMessage.rightHandTrajectoryMessage = (HandTrajectoryMessage) convertToIHMCMessage(message.getRightHandTrajectoryMessage());
      wholeBodyTrajectoryMessage.leftFootTrajectoryMessage = (FootTrajectoryMessage) convertToIHMCMessage(message.getLeftFootTrajectoryMessage());
      wholeBodyTrajectoryMessage.rightFootTrajectoryMessage = (FootTrajectoryMessage) convertToIHMCMessage(message.getRightFootTrajectoryMessage());
      wholeBodyTrajectoryMessage.chestTrajectoryMessage = (ChestTrajectoryMessage) convertToIHMCMessage(message.getChestTrajectoryMessage());
      wholeBodyTrajectoryMessage.pelvisTrajectoryMessage = (PelvisTrajectoryMessage) convertToIHMCMessage(message.getPelvisTrajectoryMessage());

      return wholeBodyTrajectoryMessage;
   }

   private static Message customConvertToRosMessage(WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage) throws Exception
   {
      Class<? extends Packet> ihmcMessageClass = WholeBodyTrajectoryMessage.class;
      String rosMessageClassNameFromIHMCMessage = GenericROSTranslationTools.getRosMessageClassNameFromIHMCMessage(ihmcMessageClass.getSimpleName());
      RosMessagePacket rosAnnotation = ihmcMessageClass.getAnnotation(RosMessagePacket.class);

      WholeBodyTrajectoryRosMessage message = messageFactory.newFromType(rosAnnotation.rosPackage() + "/" + rosMessageClassNameFromIHMCMessage);

      checkForNullComponents(wholeBodyTrajectoryMessage);
      message.setUniqueId(wholeBodyTrajectoryMessage.getUniqueId());
      message.setChestTrajectoryMessage((ChestTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getChestTrajectoryMessage()));
      message.setLeftArmTrajectoryMessage((ArmTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getArmTrajectoryMessage(RobotSide.LEFT)));
      message.setRightArmTrajectoryMessage((ArmTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getArmTrajectoryMessage(RobotSide.RIGHT)));
      message.setPelvisTrajectoryMessage((PelvisTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage()));
      message.setLeftFootTrajectoryMessage((FootTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getFootTrajectoryMessage(RobotSide.LEFT)));
      message.setRightFootTrajectoryMessage((FootTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getFootTrajectoryMessage(RobotSide.RIGHT)));
      message.setLeftHandTrajectoryMessage((HandTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.LEFT)));
      message.setRightHandTrajectoryMessage((HandTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.RIGHT)));

      return message;
   }

   private static void checkForNullComponents(WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage)
   {
      if(wholeBodyTrajectoryMessage.getChestTrajectoryMessage() == null)
      {
         ChestTrajectoryMessage component = new ChestTrajectoryMessage();
         component.setUniqueId(Packet.INVALID_MESSAGE_ID);
         wholeBodyTrajectoryMessage.setChestTrajectoryMessage(component);
      }
      if(wholeBodyTrajectoryMessage.getArmTrajectoryMessage(RobotSide.LEFT) == null)
      {
         ArmTrajectoryMessage component = new ArmTrajectoryMessage();
         component.robotSide = RobotSide.LEFT;
         component.setUniqueId(Packet.INVALID_MESSAGE_ID);
         wholeBodyTrajectoryMessage.setArmTrajectoryMessage(component);
      }
      if(wholeBodyTrajectoryMessage.getArmTrajectoryMessage(RobotSide.RIGHT) == null)
      {
         ArmTrajectoryMessage component = new ArmTrajectoryMessage();
         component.robotSide = RobotSide.RIGHT;
         component.setUniqueId(Packet.INVALID_MESSAGE_ID);
         wholeBodyTrajectoryMessage.setArmTrajectoryMessage(component);
      }
      if(wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage() == null)
      {
         PelvisTrajectoryMessage component = new PelvisTrajectoryMessage();
         component.setUniqueId(Packet.INVALID_MESSAGE_ID);
         wholeBodyTrajectoryMessage.setPelvisTrajectoryMessage(component);
      }
      if(wholeBodyTrajectoryMessage.getFootTrajectoryMessage(RobotSide.LEFT) == null)
      {
         FootTrajectoryMessage component = new FootTrajectoryMessage();
         component.robotSide = RobotSide.LEFT;
         component.setUniqueId(Packet.INVALID_MESSAGE_ID);
         wholeBodyTrajectoryMessage.setFootTrajectoryMessage(component);
      }
      if(wholeBodyTrajectoryMessage.getFootTrajectoryMessage(RobotSide.RIGHT) == null)
      {
         FootTrajectoryMessage component = new FootTrajectoryMessage();
         component.robotSide = RobotSide.RIGHT;
         component.setUniqueId(Packet.INVALID_MESSAGE_ID);
         wholeBodyTrajectoryMessage.setFootTrajectoryMessage(component);
      }
      if(wholeBodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.LEFT) == null)
      {
         HandTrajectoryMessage component = new HandTrajectoryMessage();
         component.robotSide = RobotSide.LEFT;
         component.setUniqueId(Packet.INVALID_MESSAGE_ID);
         wholeBodyTrajectoryMessage.setHandTrajectoryMessage(component);
      }
      if(wholeBodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.RIGHT) == null)
      {
         HandTrajectoryMessage component = new HandTrajectoryMessage();
         component.robotSide = RobotSide.RIGHT;
         component.setUniqueId(Packet.INVALID_MESSAGE_ID);
         wholeBodyTrajectoryMessage.setHandTrajectoryMessage(component);
      }
   }

   private static Message customConvertToRosMessage(FootstepDataMessage footstep) throws Exception
   {
      Class<? extends Packet> ihmcMessageClass = FootstepDataMessage.class;
      String rosMessageClassNameFromIHMCMessage = GenericROSTranslationTools.getRosMessageClassNameFromIHMCMessage(ihmcMessageClass.getSimpleName());
      RosMessagePacket rosAnnotation = ihmcMessageClass.getAnnotation(RosMessagePacket.class);

      FootstepDataRosMessage message = messageFactory.newFromType(rosAnnotation.rosPackage() + "/" + rosMessageClassNameFromIHMCMessage);

      message.setUniqueId(footstep.getUniqueId());
      message.setLocation(GenericROSTranslationTools.convertTuple3d(footstep.getLocation()));
      message.setOrientation(GenericROSTranslationTools.convertTuple4d(footstep.getOrientation()));
      message.setOrigin((byte) footstep.getOrigin().ordinal());
      message.setRobotSide((byte) footstep.getRobotSide().ordinal());
      message.setSwingHeight(footstep.getSwingHeight());
      message.setTrajectoryType((byte) footstep.getTrajectoryType().ordinal());

      if(footstep.hasTimings)
      {
         message.setSwingTime(footstep.swingTime);
         message.setTransferTime(footstep.transferTime);
      }
      else
      {
         message.setSwingTime(Double.NaN);
         message.setTransferTime(Double.NaN);
      }

      message.setHasTimings(footstep.hasTimings);

      if(footstep.hasAbsoluteTime)
      {
         message.setSwingStartTime(footstep.swingStartTime);
      }
      else
      {
         message.setSwingStartTime(0.0);
      }
      message.setHasAbsoluteTime(footstep.hasAbsoluteTime);

      List<Point2dRosMessage> predictedContatcPointsRos = new ArrayList<>();
      if (footstep.predictedContactPoints != null)
      {
         for (Point2d predictedContactPoint : footstep.predictedContactPoints)
         {
            predictedContatcPointsRos.add(GenericROSTranslationTools.convertPoint2d(predictedContactPoint));
         }
      }

      List<Vector3> trajectoryWaypoints = new ArrayList<>();
      if(footstep.trajectoryWaypoints != null)
      {
         for (Point3d trajectoryWaypoint : footstep.trajectoryWaypoints)
         {
            trajectoryWaypoints.add(GenericROSTranslationTools.convertTuple3d(trajectoryWaypoint));
         }
      }

      message.setPredictedContactPoints(predictedContatcPointsRos);
      message.setTrajectoryWaypoints(trajectoryWaypoints);

      return message;
   }

   private static Message customConvertToRosMessage(FootstepDataListMessage footstepList) throws Exception
   {
      Class<? extends Packet> ihmcMessageClass = FootstepDataListMessage.class;
      String rosMessageClassNameFromIHMCMessage = GenericROSTranslationTools.getRosMessageClassNameFromIHMCMessage(ihmcMessageClass.getSimpleName());
      RosMessagePacket rosAnnotation = ihmcMessageClass.getAnnotation(RosMessagePacket.class);

      FootstepDataListRosMessage message = messageFactory.newFromType(rosAnnotation.rosPackage() + "/" + rosMessageClassNameFromIHMCMessage);

      message.setDefaultSwingTime(footstepList.defaultSwingTime);
      message.setDefaultTransferTime(footstepList.defaultTransferTime);
      message.setUniqueId(footstepList.getUniqueId());
      message.setExecutionMode((byte) footstepList.executionMode.ordinal());
      message.setFinalTransferTime(footstepList.finalTransferTime);

      List<FootstepDataRosMessage> convertedFootsteps = new ArrayList<>();
      for (FootstepDataMessage footstepDataMessage : footstepList.footstepDataList)
      {
         convertedFootsteps.add((FootstepDataRosMessage) convertToRosMessage(footstepDataMessage));
      }

      message.setFootstepDataList(convertedFootsteps);

      return message;
   }

   public static String getROSMessageTypeStringFromIHMCMessageClass(Class outputType)
   {
      String rosMessageClassNameFromIHMCMessage = GenericROSTranslationTools.getRosMessageClassNameFromIHMCMessage(outputType.getSimpleName());
      RosMessagePacket annotation = (RosMessagePacket) outputType.getAnnotation(RosMessagePacket.class);

      if(annotation == null)
      {
         return null;
      }

      return annotation.rosPackage() + "/" + rosMessageClassNameFromIHMCMessage;
   }
}
