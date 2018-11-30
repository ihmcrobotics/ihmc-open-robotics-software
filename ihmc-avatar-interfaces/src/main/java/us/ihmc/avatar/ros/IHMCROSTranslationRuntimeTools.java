package us.ihmc.avatar.ros;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;

import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FrameInformation;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.QueueableMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import geometry_msgs.Point;
import ihmc_msgs.ArmTrajectoryRosMessage;
import ihmc_msgs.ChestTrajectoryRosMessage;
import ihmc_msgs.FootTrajectoryRosMessage;
import ihmc_msgs.FootstepDataListRosMessage;
import ihmc_msgs.FootstepDataRosMessage;
import ihmc_msgs.FrameInformationRosMessage;
import ihmc_msgs.HandTrajectoryRosMessage;
import ihmc_msgs.HeadTrajectoryRosMessage;
import ihmc_msgs.PelvisTrajectoryRosMessage;
import ihmc_msgs.Point2dRosMessage;
import ihmc_msgs.QueueableRosMessage;
import ihmc_msgs.SE3TrajectoryPointRosMessage;
import ihmc_msgs.WholeBodyTrajectoryRosMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.ros.msgToPacket.converter.CustomFieldConversions;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericROSTranslationTools;
import us.ihmc.utilities.ros.msgToPacket.converter.RosEnumConversionException;

/**
 * Provides generic and custom ROS<->Java translations.
 * <p>
 * If the {@link FootstepDataListMessage} or {@link WholeBodyTrajectoryMessage} definitions change,
 * you will need to update their custom translations here.
 * </p>
 * 
 */
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
      customFieldConversions.registerROSMessageFieldConverter(FrameInformationRosMessage.class,
                                                              IHMCROSTranslationRuntimeTools::convertFrameInformationRosMessage);

      customFieldConversions.registerIHMCPacketFieldConverter(WholeBodyTrajectoryMessage.class, IHMCROSTranslationRuntimeTools::customConvertToRosMessage);
      customFieldConversions.registerROSMessageFieldConverter(WholeBodyTrajectoryRosMessage.class, IHMCROSTranslationRuntimeTools::customConvertToIHMCMessage);
   }

   public static Message convertToRosMessage(Packet<?> ihmcMessage)
         throws InvocationTargetException, IllegalAccessException, NoSuchMethodException, ClassNotFoundException
   {
      if (ihmcMessage == null)
      {
         return null;
      }
      Class<? extends Packet> aClass = ihmcMessage.getClass();

      if (customFieldConversions.containsConverterFor(aClass))
      {
         return customFieldConversions.convert(ihmcMessage);
      }
      else
      {
         return GenericROSTranslationTools.convertIHMCMessageToRosMessage(ihmcMessage);
      }
   }

   public static Packet<?> convertToIHMCMessage(Message rosMessage) throws ClassNotFoundException, InvocationTargetException, IllegalAccessException,
         RosEnumConversionException, NoSuchFieldException, InstantiationException, IllegalArgumentException, NoSuchMethodException, SecurityException
   {
      if (rosMessage == null)
      {
         return null;
      }
      Class<?> aClass = Class.forName(rosMessage.toRawMessage().getType().replace("/", "."));

      if (customFieldConversions.containsConverterFor(aClass))
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

      footsteps.setDefaultSwingDuration(message.getDefaultSwingDuration());
      footsteps.setDefaultTransferDuration(message.getDefaultTransferDuration());
      footsteps.setUniqueId(message.getUniqueId());
      try
      {
         footsteps.getQueueingProperties().set((QueueableMessage) convertToIHMCMessage(message.getQueueingProperties()));
      }
      catch (ClassNotFoundException | InvocationTargetException | IllegalAccessException | NoSuchFieldException | InstantiationException
            | RosEnumConversionException | IllegalArgumentException | NoSuchMethodException | SecurityException e1)
      {
         e1.printStackTrace();
      }
      footsteps.setFinalTransferDuration(message.getFinalTransferDuration());
      footsteps.setExecutionTiming(message.getExecutionTiming());
      footsteps.setOffsetFootstepsWithExecutionError(message.getOffsetFootstepsWithExecutionError());

      ArrayList<FootstepDataMessage> stepData = new ArrayList<>();
      for (FootstepDataRosMessage footstepDataRosMessage : message.getFootstepDataList())
      {
         try
         {
            stepData.add((FootstepDataMessage) convertToIHMCMessage(footstepDataRosMessage));
         }
         catch (ClassNotFoundException | InvocationTargetException | IllegalAccessException | RosEnumConversionException | NoSuchFieldException
               | InstantiationException | IllegalArgumentException | NoSuchMethodException | SecurityException e)
         {
            e.printStackTrace();
         }
      }

      MessageTools.copyData(stepData, footsteps.getFootstepDataList());

      return footsteps;
   }

   private static Packet customConvertToIHMCMessage(FootstepDataRosMessage message)
   {
      FootstepDataMessage ihmcMessage = new FootstepDataMessage();

      ihmcMessage.setRobotSide(message.getRobotSide());
      ihmcMessage.getLocation().set(new Point3D(GenericROSTranslationTools.convertPoint(message.getLocation())));
      ihmcMessage.getOrientation().set(new us.ihmc.euclid.tuple4D.Quaternion(GenericROSTranslationTools.convertQuaternion(message.getOrientation())));
      ihmcMessage.setSwingHeight(message.getSwingHeight());
      ihmcMessage.setTrajectoryType(message.getTrajectoryType());
      ihmcMessage.setUniqueId(message.getUniqueId());
      ihmcMessage.setSwingDuration(message.getSwingDuration());
      ihmcMessage.setTransferDuration(message.getTransferDuration());
      ihmcMessage.setTouchdownDuration(message.getTouchdownDuration());
      ihmcMessage.setSwingTrajectoryBlendDuration(message.getSwingTrajectoryBlendDuration());

      ArrayList<Point2D> predictedContactPoints = new ArrayList<>();
      for (Point2dRosMessage point2dRosMessage : message.getPredictedContactPoints())
      {
         predictedContactPoints.add(GenericROSTranslationTools.convertPoint2DRos(point2dRosMessage));
      }

      Point3D[] trajectoryWaypoints = new Point3D[message.getCustomPositionWaypoints().size()];
      for (int i = 0; i < message.getCustomPositionWaypoints().size(); i++)
      {
         trajectoryWaypoints[i] = new Point3D(GenericROSTranslationTools.convertPoint(message.getCustomPositionWaypoints().get(i)));
      }

      for (SE3TrajectoryPointRosMessage rosTrajectoryPoint : message.getSwingTrajectory())
      {
         try
         {
            ihmcMessage.getSwingTrajectory().add().set((SE3TrajectoryPointMessage) convertToIHMCMessage(rosTrajectoryPoint));
         }
         catch (ClassNotFoundException | InvocationTargetException | IllegalAccessException | NoSuchFieldException | InstantiationException
               | IllegalArgumentException | NoSuchMethodException | SecurityException | RosEnumConversionException e)
         {
            e.printStackTrace();
         }
      }

      HumanoidMessageTools.packPredictedContactPoints(predictedContactPoints, ihmcMessage);
      MessageTools.copyData(trajectoryWaypoints, ihmcMessage.getCustomPositionWaypoints());

      return ihmcMessage;
   }

   private static Packet customConvertToIHMCMessage(WholeBodyTrajectoryRosMessage message)
   {
      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();

      wholeBodyTrajectoryMessage.setUniqueId(message.getUniqueId());
      try
      {
         wholeBodyTrajectoryMessage.getLeftArmTrajectoryMessage().set((ArmTrajectoryMessage) convertToIHMCMessage(message.getLeftArmTrajectoryMessage()));
         wholeBodyTrajectoryMessage.getRightArmTrajectoryMessage().set((ArmTrajectoryMessage) convertToIHMCMessage(message.getRightArmTrajectoryMessage()));
         wholeBodyTrajectoryMessage.getLeftHandTrajectoryMessage().set((HandTrajectoryMessage) convertToIHMCMessage(message.getLeftHandTrajectoryMessage()));
         wholeBodyTrajectoryMessage.getRightHandTrajectoryMessage().set((HandTrajectoryMessage) convertToIHMCMessage(message.getRightHandTrajectoryMessage()));
         wholeBodyTrajectoryMessage.getLeftFootTrajectoryMessage().set((FootTrajectoryMessage) convertToIHMCMessage(message.getLeftFootTrajectoryMessage()));
         wholeBodyTrajectoryMessage.getRightFootTrajectoryMessage().set((FootTrajectoryMessage) convertToIHMCMessage(message.getRightFootTrajectoryMessage()));
         wholeBodyTrajectoryMessage.getChestTrajectoryMessage().set((ChestTrajectoryMessage) convertToIHMCMessage(message.getChestTrajectoryMessage()));
         wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().set((PelvisTrajectoryMessage) convertToIHMCMessage(message.getPelvisTrajectoryMessage()));
         wholeBodyTrajectoryMessage.getHeadTrajectoryMessage().set((HeadTrajectoryMessage) convertToIHMCMessage(message.getHeadTrajectoryMessage()));
      }
      catch (ClassNotFoundException | InvocationTargetException | IllegalAccessException | RosEnumConversionException | NoSuchFieldException
            | InstantiationException | IllegalArgumentException | NoSuchMethodException | SecurityException e)
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
         message.setLeftArmTrajectoryMessage((ArmTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getLeftArmTrajectoryMessage()));
         message.setRightArmTrajectoryMessage((ArmTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getRightArmTrajectoryMessage()));
         message.setPelvisTrajectoryMessage((PelvisTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage()));
         message.setLeftFootTrajectoryMessage((FootTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getLeftFootTrajectoryMessage()));
         message.setRightFootTrajectoryMessage((FootTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getRightFootTrajectoryMessage()));
         message.setLeftHandTrajectoryMessage((HandTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getLeftHandTrajectoryMessage()));
         message.setRightHandTrajectoryMessage((HandTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getRightHandTrajectoryMessage()));
         message.setHeadTrajectoryMessage((HeadTrajectoryRosMessage) convertToRosMessage(wholeBodyTrajectoryMessage.getHeadTrajectoryMessage()));
      }
      catch (InvocationTargetException | IllegalAccessException | NoSuchMethodException | ClassNotFoundException e)
      {
         e.printStackTrace();
      }

      return message;
   }

   private static void checkForNullComponents(WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage)
   {
      if (wholeBodyTrajectoryMessage.getChestTrajectoryMessage() == null)
      {
         ChestTrajectoryMessage component = new ChestTrajectoryMessage();
         wholeBodyTrajectoryMessage.getChestTrajectoryMessage().set(component);
      }
      if (wholeBodyTrajectoryMessage.getLeftArmTrajectoryMessage() == null)
      {
         ArmTrajectoryMessage component = new ArmTrajectoryMessage();
         component.setRobotSide(RobotSide.LEFT.toByte());
         wholeBodyTrajectoryMessage.getLeftArmTrajectoryMessage().set(component);
      }
      if (wholeBodyTrajectoryMessage.getRightArmTrajectoryMessage() == null)
      {
         ArmTrajectoryMessage component = new ArmTrajectoryMessage();
         component.setRobotSide(RobotSide.RIGHT.toByte());
         wholeBodyTrajectoryMessage.getRightArmTrajectoryMessage().set(component);
      }
      if (wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage() == null)
      {
         PelvisTrajectoryMessage component = new PelvisTrajectoryMessage();
         wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().set(component);
      }
      if (wholeBodyTrajectoryMessage.getLeftFootTrajectoryMessage() == null)
      {
         FootTrajectoryMessage component = new FootTrajectoryMessage();
         component.setRobotSide(RobotSide.LEFT.toByte());
         wholeBodyTrajectoryMessage.getLeftFootTrajectoryMessage().set(component);
      }
      if (wholeBodyTrajectoryMessage.getRightFootTrajectoryMessage() == null)
      {
         FootTrajectoryMessage component = new FootTrajectoryMessage();
         component.setRobotSide(RobotSide.RIGHT.toByte());
         wholeBodyTrajectoryMessage.getRightFootTrajectoryMessage().set(component);
      }
      if (wholeBodyTrajectoryMessage.getLeftHandTrajectoryMessage() == null)
      {
         HandTrajectoryMessage component = new HandTrajectoryMessage();
         component.setRobotSide(RobotSide.LEFT.toByte());
         wholeBodyTrajectoryMessage.getLeftHandTrajectoryMessage().set(component);
      }
      if (wholeBodyTrajectoryMessage.getRightHandTrajectoryMessage() == null)
      {
         HandTrajectoryMessage component = new HandTrajectoryMessage();
         component.setRobotSide(RobotSide.RIGHT.toByte());
         wholeBodyTrajectoryMessage.getRightHandTrajectoryMessage().set(component);
      }
      if (wholeBodyTrajectoryMessage.getHeadTrajectoryMessage() == null)
      {
         HeadTrajectoryMessage component = new HeadTrajectoryMessage();
         wholeBodyTrajectoryMessage.getHeadTrajectoryMessage().set(component);
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
      message.setRobotSide(footstep.getRobotSide());
      message.setSwingHeight(footstep.getSwingHeight());
      message.setTrajectoryType(footstep.getTrajectoryType());
      message.setSwingDuration(footstep.getSwingDuration());
      message.setTransferDuration(footstep.getTransferDuration());
      message.setTouchdownDuration(footstep.getTouchdownDuration());
      message.setSwingTrajectoryBlendDuration(footstep.getSwingTrajectoryBlendDuration());

      List<Point2dRosMessage> predictedContatcPointsRos = new ArrayList<>();
      if (footstep.getPredictedContactPoints2d() != null)
      {
         for (Point2D predictedContactPoint : HumanoidMessageTools.unpackPredictedContactPoints(footstep))
         {
            predictedContatcPointsRos.add(GenericROSTranslationTools.convertPoint2d(predictedContactPoint));
         }
      }

      List<Point> positionWaypoints = new ArrayList<>();
      if (footstep.getCustomPositionWaypoints() != null)
      {
         List<Point3D> customPositionWaypoints = footstep.getCustomPositionWaypoints();
         for (int i = 0; i < customPositionWaypoints.size(); i++)
         {
            positionWaypoints.add(GenericROSTranslationTools.convertPoint3D(customPositionWaypoints.get(i)));
         }
      }

      List<SE3TrajectoryPointRosMessage> rosSwingTrajectory = new ArrayList<>();
      List<SE3TrajectoryPointMessage> swingTrajectory = footstep.getSwingTrajectory();
      for (int i = 0; i < swingTrajectory.size(); i++)
      {
         try
         {
            rosSwingTrajectory.add((SE3TrajectoryPointRosMessage) convertToRosMessage(swingTrajectory.get(i)));
         }
         catch (InvocationTargetException | IllegalAccessException | NoSuchMethodException | ClassNotFoundException e)
         {
            e.printStackTrace();
         }
      }
      message.setSwingTrajectory(rosSwingTrajectory);

      message.setPredictedContactPoints(predictedContatcPointsRos);
      message.setCustomPositionWaypoints(positionWaypoints);

      return message;
   }

   private static Message customConvertToRosMessage(FootstepDataListMessage footstepList)
   {
      Class<? extends Packet> ihmcMessageClass = FootstepDataListMessage.class;
      String rosMessageClassNameFromIHMCMessage = GenericROSTranslationTools.getRosMessageClassNameFromIHMCMessage(ihmcMessageClass.getSimpleName());
      RosMessagePacket rosAnnotation = ihmcMessageClass.getAnnotation(RosMessagePacket.class);

      FootstepDataListRosMessage message = messageFactory.newFromType(rosAnnotation.rosPackage() + "/" + rosMessageClassNameFromIHMCMessage);

      message.setDefaultSwingDuration(footstepList.getDefaultSwingDuration());
      message.setDefaultTransferDuration(footstepList.getDefaultTransferDuration());
      message.setUniqueId(footstepList.getUniqueId());
      message.setFinalTransferDuration(footstepList.getFinalTransferDuration());
      message.setOffsetFootstepsWithExecutionError(footstepList.getOffsetFootstepsWithExecutionError());
      message.setExecutionTiming(footstepList.getExecutionTiming());
      try
      {
         message.setQueueingProperties((QueueableRosMessage) convertToRosMessage(footstepList.getQueueingProperties()));
      }
      catch (InvocationTargetException | IllegalAccessException | NoSuchMethodException | ClassNotFoundException e1)
      {
         e1.printStackTrace();
      }

      List<FootstepDataRosMessage> convertedFootsteps = new ArrayList<>();
      List<FootstepDataMessage> footstepDataList = footstepList.getFootstepDataList();
      for (int i = 0; i < footstepDataList.size(); i++)
      {
         FootstepDataMessage footstepDataMessage = footstepDataList.get(i);
         try
         {
            convertedFootsteps.add((FootstepDataRosMessage) convertToRosMessage(footstepDataMessage));
         }
         catch (InvocationTargetException | IllegalAccessException | NoSuchMethodException | ClassNotFoundException e)
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

      if (annotation == null)
      {
         return null;
      }

      return annotation.rosPackage() + "/" + rosMessageClassNameFromIHMCMessage;
   }
}
