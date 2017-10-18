package us.ihmc.avatar.ros;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;

import geometry_msgs.Point;
import ihmc_msgs.*;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.humanoidRobotics.communication.packets.FrameInformation;
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
import us.ihmc.utilities.ros.msgToPacket.converter.CustomFieldConversions;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericROSTranslationTools;
import us.ihmc.utilities.ros.msgToPacket.converter.RosEnumConversionException;

public class IHMCROSTranslationRuntimeTools
{
   private static final MessageFactory messageFactory = GenericROSTranslationTools.getMessageFactory();
   private static final CustomFieldConversions customFieldConversions = CustomFieldConversions.getInstance();
   static
   {
      customFieldConversions.registerIHMCPacketFieldConverter(FootstepDataListMessage.class, IHMCROSTranslationRuntimeTools::customConvertToRosMessage);
      customFieldConversions.registerROSMessageFieldConverter(FootstepDataListRosMessage.class, IHMCROSTranslationRuntimeTools::customConvertToIHMCMessage);

      customFieldConversions.registerIHMCPacketFieldConverter(FootstepDataMessage.class, IHMCROSTranslationRuntimeTools::customConvertToRosMessage);
      customFieldConversions.registerROSMessageFieldConverter(FootstepDataRosMessage.class, IHMCROSTranslationRuntimeTools::customConvertToIHMCMessage);

      customFieldConversions.registerIHMCPacketFieldConverter(FrameInformation.class, IHMCROSTranslationRuntimeTools::convertFrameInformation);
      customFieldConversions.registerROSMessageFieldConverter(FrameInformationRosMessage.class, IHMCROSTranslationRuntimeTools::convertFrameInformationRosMessage);

      customFieldConversions.registerIHMCPacketFieldConverter(WholeBodyTrajectoryMessage.class, IHMCROSTranslationRuntimeTools::customConvertToRosMessage);
      customFieldConversions.registerROSMessageFieldConverter(WholeBodyTrajectoryRosMessage.class, IHMCROSTranslationRuntimeTools::customConvertToIHMCMessage);
   }

   public static Message convertToRosMessage(Packet<?> ihmcMessage)
         throws InvocationTargetException, IllegalAccessException, NoSuchMethodException, ClassNotFoundException
   {
      if(ihmcMessage == null)
      {
         return null;
      }
      Class<? extends Packet> aClass = ihmcMessage.getClass();

      if(customFieldConversions.containsConverterFor(aClass))
      {
         return customFieldConversions.convert(ihmcMessage);
      }
      else
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

      if(customFieldConversions.containsConverterFor(aClass))
      {
         return customFieldConversions.convert(rosMessage);
      }
      else
      {
         return GenericROSTranslationTools.convertRosMessageToIHMCMessage(rosMessage);
      }
   }

   private static Packet customConvertToIHMCMessage(FootstepDataListRosMessage message)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      footsteps.defaultSwingDuration = message.getDefaultSwingDuration();
      footsteps.defaultTransferDuration = message.getDefaultTransferDuration();
      footsteps.setUniqueId(message.getUniqueId());
      footsteps.executionMode = ExecutionMode.values[message.getExecutionMode()];
      footsteps.finalTransferDuration = message.getFinalTransferDuration();
      footsteps.executionTiming = ExecutionTiming.values[message.getExecutionTiming()];

      ArrayList<FootstepDataMessage> stepData = new ArrayList<>();
      for (FootstepDataRosMessage footstepDataRosMessage : message.getFootstepDataList())
      {
         try
         {
            stepData.add((FootstepDataMessage) convertToIHMCMessage(footstepDataRosMessage));
         }
         catch(ClassNotFoundException | InvocationTargetException | IllegalAccessException | RosEnumConversionException | NoSuchFieldException | InstantiationException e)
         {
            e.printStackTrace();
         }
      }

      footsteps.footstepDataList = stepData;

      return footsteps;
   }

   private static Packet customConvertToIHMCMessage(FootstepDataRosMessage message)
   {
      FootstepDataMessage ihmcMessage = new FootstepDataMessage();

      ihmcMessage.setRobotSide(RobotSide.values[message.getRobotSide()]);
      ihmcMessage.setLocation(new Point3D(GenericROSTranslationTools.convertPoint(message.getLocation())));
      ihmcMessage.setOrientation(new us.ihmc.euclid.tuple4D.Quaternion(GenericROSTranslationTools.convertQuaternion(message.getOrientation())));
      ihmcMessage.setSwingHeight(message.getSwingHeight());
      ihmcMessage.setTrajectoryType(TrajectoryType.values()[message.getTrajectoryType()]);
      ihmcMessage.setUniqueId(message.getUniqueId());
      ihmcMessage.setSwingDuration(message.getSwingDuration());
      ihmcMessage.setTransferDuration(message.getTransferDuration());

      ArrayList<Point2D> predictedContactPoints = new ArrayList<>();
      for (Point2dRosMessage point2dRosMessage : message.getPredictedContactPoints())
      {
         predictedContactPoints.add(GenericROSTranslationTools.convertPoint2DRos(point2dRosMessage));
      }

      Point3D[] trajectoryWaypoints = new Point3D[message.getPositionWaypoints().size()];
      for (int i = 0; i < message.getPositionWaypoints().size(); i++)
      {
         trajectoryWaypoints[i] = new Point3D(GenericROSTranslationTools.convertPoint(message.getPositionWaypoints().get(i)));
      }

      ihmcMessage.setPredictedContactPoints(predictedContactPoints);
      ihmcMessage.setCustomPositionWaypoints(trajectoryWaypoints);

      return ihmcMessage;
   }

   private static Packet customConvertToIHMCMessage(WholeBodyTrajectoryRosMessage message)
   {
      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();

      wholeBodyTrajectoryMessage.setUniqueId(message.getUniqueId());
      try
      {
         wholeBodyTrajectoryMessage.leftArmTrajectoryMessage = (ArmTrajectoryMessage) convertToIHMCMessage(message.getLeftArmTrajectoryMessage());
         wholeBodyTrajectoryMessage.rightArmTrajectoryMessage = (ArmTrajectoryMessage) convertToIHMCMessage(message.getRightArmTrajectoryMessage());
         wholeBodyTrajectoryMessage.leftHandTrajectoryMessage = (HandTrajectoryMessage) convertToIHMCMessage(message.getLeftHandTrajectoryMessage());
         wholeBodyTrajectoryMessage.rightHandTrajectoryMessage = (HandTrajectoryMessage) convertToIHMCMessage(message.getRightHandTrajectoryMessage());
         wholeBodyTrajectoryMessage.leftFootTrajectoryMessage = (FootTrajectoryMessage) convertToIHMCMessage(message.getLeftFootTrajectoryMessage());
         wholeBodyTrajectoryMessage.rightFootTrajectoryMessage = (FootTrajectoryMessage) convertToIHMCMessage(message.getRightFootTrajectoryMessage());
         wholeBodyTrajectoryMessage.chestTrajectoryMessage = (ChestTrajectoryMessage) convertToIHMCMessage(message.getChestTrajectoryMessage());
         wholeBodyTrajectoryMessage.pelvisTrajectoryMessage = (PelvisTrajectoryMessage) convertToIHMCMessage(message.getPelvisTrajectoryMessage());
      }
      catch(ClassNotFoundException | InvocationTargetException | IllegalAccessException | RosEnumConversionException | NoSuchFieldException | InstantiationException e)
      {
         e.printStackTrace();
      }

      return wholeBodyTrajectoryMessage;
   }

   private static Message customConvertToRosMessage(WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage)
   {
      Class<? extends Packet> ihmcMessageClass = WholeBodyTrajectoryMessage.class;
      String rosMessageClassNameFromIHMCMessage = GenericROSTranslationTools.getRosMessageClassNameFromIHMCMessage(ihmcMessageClass.getSimpleName());
      RosMessagePacket rosAnnotation = ihmcMessageClass.getAnnotation(RosMessagePacket.class);

      WholeBodyTrajectoryRosMessage message = messageFactory.newFromType(rosAnnotation.rosPackage() + "/" + rosMessageClassNameFromIHMCMessage);

      checkForNullComponents(wholeBodyTrajectoryMessage);
      message.setUniqueId(wholeBodyTrajectoryMessage.getUniqueId());
      try
      {
         message.setChestTrajectoryMessage((ChestTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getChestTrajectoryMessage()));
         message.setLeftArmTrajectoryMessage((ArmTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getArmTrajectoryMessage(RobotSide.LEFT)));
         message.setRightArmTrajectoryMessage((ArmTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getArmTrajectoryMessage(RobotSide.RIGHT)));
         message.setPelvisTrajectoryMessage((PelvisTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage()));
         message.setLeftFootTrajectoryMessage((FootTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getFootTrajectoryMessage(RobotSide.LEFT)));
         message.setRightFootTrajectoryMessage((FootTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getFootTrajectoryMessage(RobotSide.RIGHT)));
         message.setLeftHandTrajectoryMessage((HandTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.LEFT)));
         message.setRightHandTrajectoryMessage((HandTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.RIGHT)));
      }
      catch(InvocationTargetException | IllegalAccessException | NoSuchMethodException | ClassNotFoundException e)
      {
         e.printStackTrace();
      }

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

   private static Message customConvertToRosMessage(FootstepDataMessage footstep)
   {
      Class<? extends Packet> ihmcMessageClass = FootstepDataMessage.class;
      String rosMessageClassNameFromIHMCMessage = GenericROSTranslationTools.getRosMessageClassNameFromIHMCMessage(ihmcMessageClass.getSimpleName());
      RosMessagePacket rosAnnotation = ihmcMessageClass.getAnnotation(RosMessagePacket.class);

      FootstepDataRosMessage message = messageFactory.newFromType(rosAnnotation.rosPackage() + "/" + rosMessageClassNameFromIHMCMessage);

      message.setUniqueId(footstep.getUniqueId());
      message.setLocation(GenericROSTranslationTools.convertPoint3D(footstep.getLocation()));
      message.setOrientation(GenericROSTranslationTools.convertTuple4d(footstep.getOrientation()));
      message.setRobotSide((byte) footstep.getRobotSide().ordinal());
      message.setSwingHeight(footstep.getSwingHeight());
      message.setTrajectoryType((byte) footstep.getTrajectoryType().ordinal());
      message.setSwingDuration(footstep.swingDuration);
      message.setTransferDuration(footstep.transferDuration);

      List<Point2dRosMessage> predictedContatcPointsRos = new ArrayList<>();
      if (footstep.predictedContactPoints != null)
      {
         for (Point2D predictedContactPoint : footstep.predictedContactPoints)
         {
            predictedContatcPointsRos.add(GenericROSTranslationTools.convertPoint2d(predictedContactPoint));
         }
      }

      List<Point> positionWaypoints = new ArrayList<>();
      if(footstep.getCustomPositionWaypoints() != null)
      {
         for (Point3D trajectoryWaypoint : footstep.getCustomPositionWaypoints())
         {
            positionWaypoints.add(GenericROSTranslationTools.convertPoint3D(trajectoryWaypoint));
         }
      }

      message.setPredictedContactPoints(predictedContatcPointsRos);
      message.setPositionWaypoints(positionWaypoints);

      return message;
   }

   private static Message customConvertToRosMessage(FootstepDataListMessage footstepList)
   {
      Class<? extends Packet> ihmcMessageClass = FootstepDataListMessage.class;
      String rosMessageClassNameFromIHMCMessage = GenericROSTranslationTools.getRosMessageClassNameFromIHMCMessage(ihmcMessageClass.getSimpleName());
      RosMessagePacket rosAnnotation = ihmcMessageClass.getAnnotation(RosMessagePacket.class);

      FootstepDataListRosMessage message = messageFactory.newFromType(rosAnnotation.rosPackage() + "/" + rosMessageClassNameFromIHMCMessage);

      message.setDefaultSwingDuration(footstepList.defaultSwingDuration);
      message.setDefaultTransferDuration(footstepList.defaultTransferDuration);
      message.setUniqueId(footstepList.getUniqueId());
      message.setExecutionMode((byte) footstepList.executionMode.ordinal());
      message.setFinalTransferDuration(footstepList.finalTransferDuration);
      message.setExecutionTiming((byte) footstepList.getExecutionTiming().ordinal());

      List<FootstepDataRosMessage> convertedFootsteps = new ArrayList<>();
      for (FootstepDataMessage footstepDataMessage : footstepList.footstepDataList)
      {
         try
         {
            convertedFootsteps.add((FootstepDataRosMessage) convertToRosMessage(footstepDataMessage));
         }
         catch(InvocationTargetException | IllegalAccessException | NoSuchMethodException | ClassNotFoundException e)
         {
            e.printStackTrace();
         }
      }

      message.setFootstepDataList(convertedFootsteps);

      return message;
   }

   private static FrameInformationRosMessage convertFrameInformation(FrameInformation frameInformation)
   {
      Class<?> ihmcClass = FrameInformation.class;
      String rosMessageClassNameFromIHMCClass = GenericROSTranslationTools.getRosMessageClassNameFromIHMCMessage(ihmcClass.getSimpleName());
      RosMessagePacket rosAnnotation = ihmcClass.getAnnotation(RosMessagePacket.class);

      FrameInformationRosMessage message = messageFactory.newFromType(rosAnnotation.rosPackage() + "/" + rosMessageClassNameFromIHMCClass);

      message.setDataReferenceFrameId(frameInformation.getDataReferenceFrameId());
      message.setTrajectoryReferenceFrameId(frameInformation.getTrajectoryReferenceFrameId());

      return message;
   }

   private static FrameInformation convertFrameInformationRosMessage(FrameInformationRosMessage message)
   {
      FrameInformation frameInformation = new FrameInformation();

      frameInformation.setDataReferenceFrameId(message.getDataReferenceFrameId());
      frameInformation.setTrajectoryReferenceFrameId(message.getTrajectoryReferenceFrameId());

      return frameInformation;
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
