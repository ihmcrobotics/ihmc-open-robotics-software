package us.ihmc.darpaRoboticsChallenge.ros;

import static us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter.convertByteToEnum;
import static us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter.convertEnumToByte;
import static us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter.convertPoint3dToVector3;
import static us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter.convertQuat4dToQuaternion;
import static us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter.convertQuaternionToQuat4d;
import static us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter.convertVector3ToPoint3d;
import static us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter.convertVector3ToVector3d;
import static us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter.convertVector3ToVector3f;
import static us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter.convertVector3dToVector3;
import static us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter.convertVector3fToVector3;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.vecmath.Point2d;

import ihmc_msgs.FingerStatePacketMessage;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import ihmc_msgs.ArmJointTrajectoryPacketMessage;
import ihmc_msgs.AtlasDesiredPumpPSIPacketMessage;
import ihmc_msgs.AtlasElectricMotorEnablePacketMessage;
import ihmc_msgs.AtlasWristSensorCalibrationRequestPacketMessage;
import ihmc_msgs.ChestOrientationPacketMessage;
import ihmc_msgs.ComHeightPacketMessage;
import ihmc_msgs.FootPosePacketMessage;
import ihmc_msgs.FootstepDataListMessage;
import ihmc_msgs.FootstepDataMessage;
import ihmc_msgs.FootstepStatusMessage;
import ihmc_msgs.HandComplianceControlParametersPacketMessage;
import ihmc_msgs.HandPosePacketMessage;
import ihmc_msgs.HeadOrientationPacketMessage;
import ihmc_msgs.HighLevelStateChangePacketMessage;
import ihmc_msgs.HighLevelStatePacketMessage;
import ihmc_msgs.JointAnglesPacketMessage;
import ihmc_msgs.JointTrajectoryPointMessage;
import ihmc_msgs.LegCompliancePacketMessage;
import ihmc_msgs.MultiJointAnglePacketMessage;
import ihmc_msgs.PauseCommandMessage;
import ihmc_msgs.Point2dMessage;
import ihmc_msgs.SingleJointAnglePacketMessage;
import ihmc_msgs.StopMotionPacketMessage;
import ihmc_msgs.WholeBodyTrajectoryPacketMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangePacket;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStatePacket;
import us.ihmc.humanoidRobotics.communication.packets.LegCompliancePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.FingerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasDesiredPumpPSIPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorEnablePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorPacketEnum;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasWristSensorCalibrationRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandComplianceControlParametersPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.JointTrajectoryPoint;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopMotionPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.ComHeightPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataList;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus.Status;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadOrientationPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseCommand;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.JointAnglesPacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.MultiJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.SingleJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class DRCROSMessageConverter
{
   private static final MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

   public static Message convertToRosMessage(Packet<?> packet)
   {
      if (packet instanceof HandPosePacket)
         return convertToRosMessage((HandPosePacket) packet);
      else if (packet instanceof ComHeightPacket)
         return convertToRosMessage((ComHeightPacket) packet);
      else if (packet instanceof FootPosePacket)
         return convertToRosMessage((FootPosePacket) packet);
      else if (packet instanceof FootstepData)
         return convertToRosMessage((FootstepData) packet);
      else if (packet instanceof FootstepDataList)
         return convertToRosMessage((FootstepDataList) packet);
      else if (packet instanceof FootstepStatus)
         return convertToRosMessage((FootstepStatus) packet);
      else if (packet instanceof ChestOrientationPacket)
         return convertToRosMessage((ChestOrientationPacket) packet);
      else if (packet instanceof HeadOrientationPacket)
         return convertToRosMessage((HeadOrientationPacket) packet);
      else if (packet instanceof PauseCommand)
         return convertToRosMessage((PauseCommand) packet);
      else if (packet instanceof HighLevelStatePacket)
         return convertToRosMessage((HighLevelStatePacket) packet);
      else if (packet instanceof ArmJointTrajectoryPacket)
         return convertToRosMessage((ArmJointTrajectoryPacket) packet);
      else if (packet instanceof JointAnglesPacket)
         return convertToRosMessage((JointAnglesPacket) packet);
      else if (packet instanceof AtlasElectricMotorEnablePacket)
         return convertToRosMessage((AtlasElectricMotorEnablePacket) packet);
      else if (packet instanceof AtlasDesiredPumpPSIPacket)
         return convertToRosMessage((AtlasDesiredPumpPSIPacket) packet);
      else if (packet instanceof AtlasWristSensorCalibrationRequestPacket)
         return convertToRosMessage((AtlasWristSensorCalibrationRequestPacket) packet);
      else if (packet instanceof HighLevelStateChangePacket)
         return convertToRosMessage((HighLevelStateChangePacket) packet);
      else if (packet instanceof MultiJointAnglePacket)
         return convertToRosMessage((MultiJointAnglePacket) packet);
      else if (packet instanceof HandComplianceControlParametersPacket)
         return convertToRosMessage((HandComplianceControlParametersPacket) packet);
      else if (packet instanceof LegCompliancePacket)
         return convertToRosMessage((LegCompliancePacket) packet);
      else if (packet instanceof WholeBodyTrajectoryPacket)
         return convertToRosMessage((WholeBodyTrajectoryPacket) packet);
      else if (packet instanceof StopMotionPacket)
         return convertToRosMessage((StopMotionPacket) packet);
      else if (packet instanceof HandDesiredConfigurationMessage)
         return convertToRosMessage((HandDesiredConfigurationMessage) packet);
      else
         return null;
   }

   public static Packet<?> convertToPacket(Message message)
   {
      if (message instanceof HandPosePacketMessage)
         return convertToPacket((HandPosePacketMessage) message);
      else if (message instanceof ComHeightPacketMessage)
         return convertToPacket((ComHeightPacketMessage) message);
      else if (message instanceof FootPosePacketMessage)
         return convertToPacket((FootPosePacketMessage) message);
      else if (message instanceof FootstepDataMessage)
         return convertToPacket((FootstepDataMessage) message);
      else if (message instanceof FootstepDataListMessage)
         return convertToPacket((FootstepDataListMessage) message);
      else if (message instanceof FootstepStatusMessage)
         return convertToPacket((FootstepStatusMessage) message);
      else if (message instanceof ChestOrientationPacketMessage)
         return convertToPacket((ChestOrientationPacketMessage) message);
      else if (message instanceof HeadOrientationPacketMessage)
         return convertToPacket((HeadOrientationPacketMessage) message);
      else if (message instanceof PauseCommandMessage)
         return convertToPacket((PauseCommandMessage) message);
      else if (message instanceof HighLevelStatePacketMessage)
         return convertToPacket((HighLevelStatePacketMessage) message);
      else if (message instanceof ArmJointTrajectoryPacketMessage)
         return convertToPacket((ArmJointTrajectoryPacketMessage) message);
      else if (message instanceof JointAnglesPacketMessage)
         return convertToPacket((JointAnglesPacketMessage) message);
      else if (message instanceof AtlasElectricMotorEnablePacketMessage)
         return convertToPacket((AtlasElectricMotorEnablePacketMessage) message);
      else if (message instanceof AtlasDesiredPumpPSIPacketMessage)
         return convertToPacket((AtlasDesiredPumpPSIPacketMessage) message);
      else if (message instanceof AtlasWristSensorCalibrationRequestPacketMessage)
         return convertToPacket((AtlasWristSensorCalibrationRequestPacketMessage) message);
      else if (message instanceof HighLevelStateChangePacketMessage)
         return convertToPacket((HighLevelStateChangePacketMessage) message);
      else if (message instanceof MultiJointAnglePacketMessage)
         return convertToPacket((MultiJointAnglePacketMessage) message);
      else if (message instanceof HandComplianceControlParametersPacketMessage)
         return convertToPacket((HandComplianceControlParametersPacketMessage) message);
      else if (message instanceof LegCompliancePacketMessage)
         return convertToPacket((LegCompliancePacketMessage) message);
      else if (message instanceof WholeBodyTrajectoryPacketMessage)
         return convertToPacket((WholeBodyTrajectoryPacketMessage) message);
      else if (message instanceof StopMotionPacketMessage)
         return convertToPacket((StopMotionPacketMessage) message);
      else if (message instanceof FingerStatePacketMessage)
         return convertToPacket((FingerStatePacketMessage) message);
      else
         return null;
   }

   private static Packet<?> convertToPacket(StopMotionPacketMessage message)
   {
      StopMotionPacket ret = new StopMotionPacket();
      ret.setUniqueId(message.getUniqueId());
      return ret;
   }

   public static StopMotionPacketMessage convertToRosMessage(StopMotionPacket packet)
   {
      StopMotionPacketMessage ret = messageFactory.newFromType("ihmc_msgs/StopMotionPacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      return ret;
   }

   private static Packet<?> convertToPacket(WholeBodyTrajectoryPacketMessage message)
   {
      WholeBodyTrajectoryPacket ret = new WholeBodyTrajectoryPacket(message.getNumWaypoints(), message.getNumJointsPerArm());ret.setUniqueId(message.getUniqueId());
      

      if (message.getPelvisWorldPosition().size() == 0)
         ret.pelvisWorldPosition = null;
      if (message.getPelvisLinearVelocity().size() == 0)
         ret.pelvisLinearVelocity = null;
      if (message.getPelvisAngularVelocity().size() == 0)
         ret.pelvisAngularVelocity = null;
      if (message.getPelvisWorldOrientation().size() == 0)
         ret.pelvisWorldOrientation = null;
      if (message.getChestWorldOrientation().size() == 0)
         ret.chestWorldOrientation = null;
      if (message.getChestAngularVelocity().size() == 0)
         ret.chestAngularVelocity = null;

      for (int i = 0; i < message.getNumWaypoints(); i++)
      {
         ret.timeAtWaypoint[i] = message.getTimeAtWaypoint()[i];
         if (ret.pelvisWorldPosition != null)
            ret.pelvisWorldPosition[i] = convertVector3ToPoint3d(message.getPelvisWorldPosition().get(i));
         if (ret.pelvisLinearVelocity != null)
            ret.pelvisLinearVelocity[i] = convertVector3ToVector3d(message.getPelvisLinearVelocity().get(i));
         if (ret.pelvisAngularVelocity != null)
            ret.pelvisAngularVelocity[i] = convertVector3ToVector3d(message.getPelvisAngularVelocity().get(i));
         if (ret.pelvisWorldOrientation != null)
            ret.pelvisWorldOrientation[i] = convertQuaternionToQuat4d(message.getPelvisWorldOrientation().get(i));
         if (ret.chestWorldOrientation != null)
            ret.chestWorldOrientation[i] = convertQuaternionToQuat4d(message.getChestWorldOrientation().get(i));
         if (ret.chestAngularVelocity != null)
            ret.chestAngularVelocity[i] = convertVector3ToVector3d(message.getChestAngularVelocity().get(i));
      }

      ret.rightArmTrajectory = convertToPacket(message.getRightArmTrajectory());
      ret.leftArmTrajectory = convertToPacket(message.getLeftArmTrajectory());

      return ret;
   }

   public static WholeBodyTrajectoryPacketMessage convertToRosMessage(WholeBodyTrajectoryPacket packet)
   {
      WholeBodyTrajectoryPacketMessage ret = messageFactory.newFromType("ihmc_msgs/WholeBodyTrajectoryPacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setNumWaypoints(packet.numWaypoints);
      ret.setNumJointsPerArm(packet.numJointsPerArm);
      ArrayList<Vector3> pelvisWorldPositions = new ArrayList<>();
      ArrayList<Vector3> pelvisLinearVelocity = new ArrayList<>();
      ArrayList<Vector3> pelvisAngularVelocity = new ArrayList<>();
      ArrayList<Quaternion> pelvisWorldOrientation = new ArrayList<>();
      ArrayList<Quaternion> chestWorldOrientation = new ArrayList<>();
      ArrayList<Vector3> chestAngularVelocity = new ArrayList<>();

      for (int i = 0; i < packet.numWaypoints; i++)
      {
         if (packet.pelvisWorldPosition != null)
            pelvisWorldPositions.add(convertPoint3dToVector3(packet.pelvisWorldPosition[i]));
         if (packet.pelvisLinearVelocity != null)
            pelvisLinearVelocity.add(convertVector3dToVector3(packet.pelvisLinearVelocity[i]));
         if (packet.pelvisAngularVelocity != null)
            pelvisAngularVelocity.add(convertVector3dToVector3(packet.pelvisAngularVelocity[i]));
         if (packet.pelvisWorldOrientation != null)
            pelvisWorldOrientation.add(convertQuat4dToQuaternion(packet.pelvisWorldOrientation[i]));
         if (packet.chestWorldOrientation != null)
            chestWorldOrientation.add(convertQuat4dToQuaternion(packet.chestWorldOrientation[i]));
         if (packet.chestAngularVelocity != null)
            chestAngularVelocity.add(convertVector3dToVector3(packet.chestAngularVelocity[i]));
      }

      ret.setTimeAtWaypoint(packet.timeAtWaypoint);
      ret.setPelvisWorldPosition(pelvisWorldPositions);
      ret.setPelvisLinearVelocity(pelvisLinearVelocity);
      ret.setPelvisAngularVelocity(pelvisAngularVelocity);
      ret.setPelvisWorldOrientation(pelvisWorldOrientation);
      ret.setChestWorldOrientation(chestWorldOrientation);
      ret.setChestAngularVelocity(chestAngularVelocity);

      ret.setRightArmTrajectory(convertToRosMessage(packet.rightArmTrajectory));
      ret.setLeftArmTrajectory(convertToRosMessage(packet.leftArmTrajectory));

      return ret;

   }

   public static HandComplianceControlParametersPacket convertToPacket(HandComplianceControlParametersPacketMessage message)
   {
      HandComplianceControlParametersPacket ret = new HandComplianceControlParametersPacket();ret.setUniqueId(message.getUniqueId());

      ret.setEnableLinearCompliance(message.getEnableLinearCompliance());
      ret.setEnableAngularCompliance(message.getEnableAngularCompliance());
      ret.setDesiredForce(convertVector3ToVector3f(message.getDesiredForce()));
      ret.setDesiredTorque(convertVector3ToVector3f(message.getDesiredTorque()));
      ret.robotSide = convertByteToEnum(RobotSide.class, message.getRobotSide());
      ret.wrenchDeadzones = message.getWrenchDeadzones();

      return ret;
   }

   public static LegCompliancePacket convertToPacket(LegCompliancePacketMessage message)
   {
      LegCompliancePacket ret = new LegCompliancePacket();ret.setUniqueId(message.getUniqueId());

      ret.robotSide = convertByteToEnum(RobotSide.class, message.getRobotSide());
      ret.maxVelocityDeltas = message.getMaxVelocityDeltas();

      return ret;
   }

   public static HandComplianceControlParametersPacketMessage convertToRosMessage(HandComplianceControlParametersPacket packet)
   {
      HandComplianceControlParametersPacketMessage ret = messageFactory.newFromType("ihmc_msgs/HandComplianceControlParametersPacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setDesiredForce(convertVector3fToVector3(packet.getDesiredForce()));
      ret.setDesiredTorque(convertVector3fToVector3(packet.getDesiredTorque()));
      ret.setEnableAngularCompliance(packet.getEnableAngularCompliance());
      ret.setEnableLinearCompliance(packet.getEnableLinearCompliance());
      ret.setWrenchDeadzones(packet.getWrenchDeadzones());
      ret.setRobotSide(convertEnumToByte(packet.getRobotSide()));

      return ret;
   }

   public static LegCompliancePacketMessage convertToRosMessage(LegCompliancePacket packet)
   {
      LegCompliancePacketMessage ret = messageFactory.newFromType("ihmc_msgs/LegCompliancePacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setMaxVelocityDeltas(packet.maxVelocityDeltas);
      ret.setRobotSide(convertEnumToByte(packet.robotSide));

      return ret;
   }

   public static AtlasElectricMotorEnablePacketMessage convertToRosMessage(AtlasElectricMotorEnablePacket packet)
   {
      AtlasElectricMotorEnablePacketMessage ret = messageFactory.newFromType("ihmc_msgs/AtlasElectricMotorEnablePacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setEnable(packet.enable);
      ret.setMotorEnableEnum(convertEnumToByte(packet.motorEnableEnum));

      return ret;
   }

   public static AtlasDesiredPumpPSIPacketMessage convertToRosMessage(AtlasDesiredPumpPSIPacket packet)
   {
      AtlasDesiredPumpPSIPacketMessage ret = messageFactory.newFromType("ihmc_msgs/AtlasDesiredPumpPSIPacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setDesiredPumpPsi(packet.desiredPumpPSI);

      return ret;
   }

   public static AtlasWristSensorCalibrationRequestPacketMessage convertToRosMessage(AtlasWristSensorCalibrationRequestPacket packet)
   {
      AtlasWristSensorCalibrationRequestPacketMessage ret = messageFactory.newFromType("ihmc_msgs/AtlasWristSensorCalibrationRequestPacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setRobotSide(convertEnumToByte(packet.getRobotSide()));

      return ret;
   }

   public static HandPosePacketMessage convertToRosMessage(HandPosePacket packet)
   {
      HandPosePacketMessage ret = messageFactory.newFromType("ihmc_msgs/HandPosePacketMessage");
      ret.setUniqueId(packet.getUniqueId());

      ret.setRobotSide(convertEnumToByte(packet.getRobotSide()));
      ret.setDataType(convertEnumToByte(packet.getDataType()));
      ret.setReferenceFrame(convertEnumToByte(packet.getReferenceFrame()));
      ret.setToHomePosition(packet.isToHomePosition());
      ret.setPosition(convertPoint3dToVector3(packet.getPosition()));
      ret.setOrientation(convertQuat4dToQuaternion(packet.getOrientation()));
      ret.setTrajectoryTime(packet.getTrajectoryTime());
      ret.setJointAngles(packet.getJointAngles());
      ret.setControlOrientation(packet.getControlOrientation());

      return ret;
   }

   public static AtlasElectricMotorEnablePacket convertToPacket(AtlasElectricMotorEnablePacketMessage message)
   {
      AtlasElectricMotorEnablePacket ret = new AtlasElectricMotorEnablePacket();ret.setUniqueId(message.getUniqueId());

      ret.enable = message.getEnable();
      ret.motorEnableEnum = convertByteToEnum(AtlasElectricMotorPacketEnum.class, message.getMotorEnableEnum());

      return ret;
   }

   public static AtlasWristSensorCalibrationRequestPacket convertToPacket(AtlasWristSensorCalibrationRequestPacketMessage message)
   {
      AtlasWristSensorCalibrationRequestPacket ret = new AtlasWristSensorCalibrationRequestPacket();ret.setUniqueId(message.getUniqueId());

      ret.robotSide = convertByteToEnum(RobotSide.class, message.getRobotSide());

      return ret;
   }

   public static AtlasDesiredPumpPSIPacket convertToPacket(AtlasDesiredPumpPSIPacketMessage message)
   {
      AtlasDesiredPumpPSIPacket ret = new AtlasDesiredPumpPSIPacket();ret.setUniqueId(message.getUniqueId());

      ret.desiredPumpPSI = message.getDesiredPumpPsi();

      return ret;
   }

   public static HandPosePacket convertToPacket(HandPosePacketMessage message)
   {
      HandPosePacket ret = new HandPosePacket(convertByteToEnum(RobotSide.class, message.getRobotSide()),
            convertByteToEnum(HandPosePacket.Frame.class, message.getReferenceFrame()), convertByteToEnum(HandPosePacket.DataType.class, message.getDataType()),
            convertVector3ToPoint3d(message.getPosition()), convertQuaternionToQuat4d(message.getOrientation()), message.getToHomePosition(),
            message.getTrajectoryTime(), message.getJointAngles());
      ret.setControlledOrientationAxes(message.getControlOrientation(), message.getControlOrientation(), message.getControlOrientation());
      return ret;
   }

   public static ComHeightPacketMessage convertToRosMessage(ComHeightPacket packet)
   {
      ComHeightPacketMessage ret = messageFactory.newFromType("ihmc_msgs/ComHeightPacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setHeightOffset(packet.getHeightOffset());
      ret.setTrajectoryTime(packet.getTrajectoryTime());

      return ret;
   }

   public static ComHeightPacket convertToPacket(ComHeightPacketMessage message)
   {
      ComHeightPacket ret = new ComHeightPacket(message.getHeightOffset(), message.getTrajectoryTime());ret.setUniqueId(message.getUniqueId());

      return ret;
   }

   public static FootPosePacketMessage convertToRosMessage(FootPosePacket packet)
   {
      FootPosePacketMessage ret = messageFactory.newFromType("ihmc_msgs/FootPosePacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setRobotSide(convertEnumToByte(packet.getRobotSide()));
      ret.setPosition(convertPoint3dToVector3(packet.getPosition()));
      ret.setOrientation(convertQuat4dToQuaternion(packet.getOrientation()));
      ret.setTrajectoryTime(packet.getTrajectoryTime());

      return ret;
   }

   public static FootPosePacket convertToPacket(FootPosePacketMessage message)
   {
      FootPosePacket ret = new FootPosePacket(convertByteToEnum(RobotSide.class, message.getRobotSide()), convertVector3ToPoint3d(message.getPosition()),
            convertQuaternionToQuat4d(message.getOrientation()), message.getTrajectoryTime());
      ret.setUniqueId(message.getUniqueId());
      return ret;
   }

   public static FootstepDataMessage convertToRosMessage(FootstepData packet)
   {
      FootstepDataMessage ret = messageFactory.newFromType("ihmc_msgs/FootstepDataMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setUniqueId(packet.getUniqueId());
      ret.setRobotSide(convertEnumToByte(packet.getRobotSide()));
      ret.setLocation(convertPoint3dToVector3(packet.getLocation()));
      ret.setOrientation(convertQuat4dToQuaternion(packet.getOrientation()));
      if (packet.getPredictedContactPoints() != null)
      {
         ArrayList<Point2dMessage> predictedContactPoints = new ArrayList<>();
         for (int i = 0; i < packet.getPredictedContactPoints().size(); i++)
         {
            predictedContactPoints.add(convertToRosMessage(packet.getPredictedContactPoints().get(i)));
         }
         ret.setPredictedContactPoints(predictedContactPoints);
      }
      ret.setTrajectoryType(convertEnumToByte(packet.getTrajectoryType()));
      ret.setSwingHeight(packet.getSwingHeight());

      return ret;
   }

   public static FootstepData convertToPacket(FootstepDataMessage message)
   {
      ArrayList<Point2d> predictedContactPoints = new ArrayList<>();
      for (int i = 0; i < message.getPredictedContactPoints().size(); i++)
      {
         predictedContactPoints.add(convertToPacket(message.getPredictedContactPoints().get(i)));
      }

      FootstepData ret = new FootstepData(convertByteToEnum(RobotSide.class, message.getRobotSide()), convertVector3ToPoint3d(message.getLocation()),
            convertQuaternionToQuat4d(message.getOrientation()), predictedContactPoints, convertByteToEnum(TrajectoryType.class, message.getTrajectoryType()),
            message.getSwingHeight());

      ret.setUniqueId(message.getUniqueId());
      return ret;
   }

   public static Point2d convertToPacket(Point2dMessage message)
   {
      Point2d ret = new Point2d(message.getX(), message.getY());

      return ret;
   }

   public static Point2dMessage convertToRosMessage(Point2d packet)
   {
      Point2dMessage ret = messageFactory.newFromType("ihmc_msgs/Point2dMessage");
      
      ret.setX(packet.getX());
      ret.setY(packet.getY());

      return ret;
   }

   public static FootstepDataListMessage convertToRosMessage(FootstepDataList packet)
   {
      List<FootstepDataMessage> footstepDataMessageList = new ArrayList<>();
      for (int i = 0; i < packet.size(); i++)
      {
         footstepDataMessageList.add(convertToRosMessage(packet.get(i)));
      }

      FootstepDataListMessage ret = messageFactory.newFromType("ihmc_msgs/FootstepDataListMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setFootstepDataList(footstepDataMessageList);
      ret.setSwingTime(packet.swingTime);
      ret.setTransferTime(packet.transferTime);

      return ret;
   }

   public static FootstepDataList convertToPacket(FootstepDataListMessage message)
   {
      ArrayList<FootstepData> footstepDataList = new ArrayList<>();
      for (int i = 0; i < message.getFootstepDataList().size(); i++)
      {
         footstepDataList.add(convertToPacket(message.getFootstepDataList().get(i)));
      }
      FootstepDataList ret = new FootstepDataList(footstepDataList, message.getSwingTime(), message.getTransferTime());
      ret.setUniqueId(message.getUniqueId());
      return ret;
   }

   public static FootstepStatusMessage convertToRosMessage(FootstepStatus packet)
   {
      FootstepStatusMessage ret = messageFactory.newFromType("ihmc_msgs/FootstepStatusMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setIsDoneWalking(packet.isDoneWalking());

      if (!packet.isDoneWalking())
      {
         ret.setStatus(convertEnumToByte(packet.getStatus()));
         ret.setFootstepIndex(packet.getFootstepIndex());
         if (packet.getActualFootPositionInWorld() != null)
            ret.setActualFootPositionInWorld(convertPoint3dToVector3(packet.getActualFootPositionInWorld()));
         if (packet.getActualFootOrientationInWorld() != null)
            ret.setActualFootOrientationInWorld(convertQuat4dToQuaternion(packet.getActualFootOrientationInWorld()));
         if (packet.getRobotSide() != null)
            ret.setRobotSide(convertEnumToByte(packet.getRobotSide()));
      }

      return ret;
   }

   public static FootstepStatus convertToPacket(FootstepStatusMessage message)
   {
      FootstepStatus ret = new FootstepStatus(convertByteToEnum(Status.class, message.getStatus()), message.getFootstepIndex(),
            convertVector3ToPoint3d(message.getActualFootPositionInWorld()), convertQuaternionToQuat4d(message.getActualFootOrientationInWorld()),
            convertByteToEnum(RobotSide.class, message.getRobotSide()));
      ret.setUniqueId(message.getUniqueId());
      return ret;
   }

   public static ChestOrientationPacketMessage convertToRosMessage(ChestOrientationPacket packet)
   {
      ChestOrientationPacketMessage ret = messageFactory.newFromType("ihmc_msgs/ChestOrientationPacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setOrientation(convertQuat4dToQuaternion(packet.getOrientation()));
      ret.setToHomeOrientation(packet.isToHomeOrientation());
      ret.setTrajectoryTime(packet.getTrajectoryTime());

      return ret;
   }

   public static ChestOrientationPacket convertToPacket(ChestOrientationPacketMessage message)
   {
      ChestOrientationPacket ret = new ChestOrientationPacket(convertQuaternionToQuat4d(message.getOrientation()), message.getToHomeOrientation(),
            message.getTrajectoryTime());
      ret.setUniqueId(message.getUniqueId());
      return ret;
   }

   public static HeadOrientationPacketMessage convertToRosMessage(HeadOrientationPacket packet)
   {
      HeadOrientationPacketMessage ret = messageFactory.newFromType("ihmc_msgs/HeadOrientationPacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setOrientation(convertQuat4dToQuaternion(packet.getOrientation()));
      ret.setTrajectoryTime(packet.getTrajectoryTime());

      return ret;
   }

   public static HeadOrientationPacket convertToPacket(HeadOrientationPacketMessage message)
   {
      HeadOrientationPacket ret = new HeadOrientationPacket(convertQuaternionToQuat4d(message.getOrientation()), message.getTrajectoryTime());ret.setUniqueId(
         message.getUniqueId());

      return ret;
   }

   public static PauseCommandMessage convertToRosMessage(PauseCommand packet)
   {
      PauseCommandMessage ret = messageFactory.newFromType("ihmc_msgs/PauseCommandMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setPause(packet.isPaused());

      return ret;
   }

   public static PauseCommand convertToPacket(PauseCommandMessage message)
   {
      PauseCommand ret = new PauseCommand(message.getPause());
      ret.setUniqueId(message.getUniqueId());
      return ret;
   }

   public static HighLevelStatePacketMessage convertToRosMessage(HighLevelStatePacket packet)
   {
      HighLevelStatePacketMessage ret = messageFactory.newFromType("ihmc_msgs/HighLevelStatePacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setHighLevelState(convertEnumToByte(packet.getHighLevelState()));

      return ret;
   }

   public static HighLevelStatePacket convertToPacket(HighLevelStatePacketMessage message)
   {
      HighLevelStatePacket ret = new HighLevelStatePacket(convertByteToEnum(HighLevelState.class, message.getHighLevelState()));ret.setUniqueId(
         message.getUniqueId());

      return ret;
   }

   public static ArmJointTrajectoryPacketMessage convertToRosMessage(ArmJointTrajectoryPacket packet)
   {
      int waypoints = packet.trajectoryPoints.length;
      List<JointTrajectoryPointMessage> trajectoryPoints = new ArrayList<JointTrajectoryPointMessage>();

      for (int i = 0; i < waypoints; i++)
      {
         JointTrajectoryPoint point = packet.trajectoryPoints[i];
         JointTrajectoryPointMessage pointMessage = messageFactory.newFromType("ihmc_msgs/JointTrajectoryPointMessage");
         pointMessage.setPositions(point.positions);
         pointMessage.setVelocities(point.velocities);
         pointMessage.setTime(point.time);
         trajectoryPoints.add(pointMessage);
      }

      ArmJointTrajectoryPacketMessage ret = messageFactory.newFromType("ihmc_msgs/ArmJointTrajectoryPacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setRobotSide(convertEnumToByte(packet.robotSide));
      ret.setTrajectoryPoints(trajectoryPoints);

      return ret;
   }

   public static ArmJointTrajectoryPacket convertToPacket(ArmJointTrajectoryPacketMessage message)
   {
      int waypoints = message.getTrajectoryPoints().size();

      JointTrajectoryPoint[] trajectoryPoints = new JointTrajectoryPoint[waypoints];
      RobotSide robotSide = convertByteToEnum(RobotSide.class, message.getRobotSide());
      ArmJointTrajectoryPacket ret = new ArmJointTrajectoryPacket(robotSide, trajectoryPoints);ret.setUniqueId(message.getUniqueId());

      for (int i = 0; i < waypoints; i++)
      {
         JointTrajectoryPointMessage pointMessage = message.getTrajectoryPoints().get(i);
         ret.trajectoryPoints[i] = new JointTrajectoryPoint(pointMessage.getPositions(), pointMessage.getVelocities(), pointMessage.getTime());
      }

      return ret;
   }

   public static JointAnglesPacketMessage convertToRosMessage(JointAnglesPacket packet)
   {
      JointAnglesPacketMessage ret = messageFactory.newFromType("ihmc_msgs/JointAnglesPacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setLeftArmJointAngle(packet.leftArmJointAngle);
      ret.setRightArmJointAngle(packet.rightArmJointAngle);
      ret.setLeftLegJointAngle(packet.leftLegJointAngle);
      ret.setRightLegJointAngle(packet.rightLegJointAngle);
      ret.setSpineJointAngle(packet.spineJointAngle);
      ret.setNeckJointAngle(packet.neckJointAngle);
      ret.setTrajectoryTime(packet.trajectoryTime);

      return ret;
   }

   public static JointAnglesPacket convertToPacket(JointAnglesPacketMessage message)
   {
      JointAnglesPacket ret = new JointAnglesPacket(message.getLeftArmJointAngle().length, message.getLeftLegJointAngle().length,
            message.getSpineJointAngle().length);
      ret.setUniqueId(message.getUniqueId());
      ret.setArmJointAngle(RobotSide.LEFT, message.getLeftArmJointAngle());
      ret.setArmJointAngle(RobotSide.RIGHT, message.getRightArmJointAngle());
      ret.setLegJointAngle(RobotSide.LEFT, message.getLeftLegJointAngle());
      ret.setLegJointAngle(RobotSide.RIGHT, message.getRightLegJointAngle());
      ret.setSpineJointAngles(message.getSpineJointAngle());
      ret.setNeckJointAngle(message.getNeckJointAngle());
      ret.setTrajectoryTime(message.getTrajectoryTime());

      return ret;
   }

   public static MultiJointAnglePacketMessage convertToRosMessage(MultiJointAnglePacket packet)
   {
      MultiJointAnglePacketMessage ret = messageFactory.newFromType("ihmc_msgs/MultiJointAnglePacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      int numberOfJoints = packet.singleJointAnglePackets.length;
      SingleJointAnglePacketMessage[] singleJointPackets = new SingleJointAnglePacketMessage[numberOfJoints];

      for (int i = 0; i < numberOfJoints; i++)
      {
         SingleJointAnglePacketMessage singleJointMessage = messageFactory.newFromType("ihmc_msgs/SingleJointAnglePacketMessage");

         SingleJointAnglePacket singleJointPacket = packet.singleJointAnglePackets[i];
         singleJointMessage.setAngle(singleJointPacket.angle);
         singleJointMessage.setTrajcetoryTime(singleJointPacket.trajcetoryTime);
         singleJointMessage.setJointName(singleJointPacket.jointName);

         singleJointPackets[i] = singleJointMessage;
      }

      ret.setSingleJointAnglePackets(Arrays.asList(singleJointPackets));
      return ret;
   }

   public static MultiJointAnglePacket convertToPacket(MultiJointAnglePacketMessage message)
   {
      MultiJointAnglePacket ret = new MultiJointAnglePacket();ret.setUniqueId(message.getUniqueId());
      List<SingleJointAnglePacketMessage> singleJointPackets = message.getSingleJointAnglePackets();
      SingleJointAnglePacket[] singleJointAnglePackets = new SingleJointAnglePacket[singleJointPackets.size()];

      for (int i = 0; i < singleJointPackets.size(); i++)
      {
         String jointName = singleJointPackets.get(i).getJointName();
         double angle = singleJointPackets.get(i).getAngle();
         double trajcetoryTime = singleJointPackets.get(i).getTrajcetoryTime();

         singleJointAnglePackets[i] = new SingleJointAnglePacket(jointName, angle, trajcetoryTime, Double.NaN);
      }

      ret.singleJointAnglePackets = singleJointAnglePackets;

      return ret;
   }

   public static HighLevelStateChangePacketMessage convertToRosMessage(HighLevelStateChangePacket packet)
   {
      HighLevelStateChangePacketMessage ret = messageFactory.newFromType("ihmc_msgs/HighLevelStateChangePacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setInitialState(convertEnumToByte(packet.getInitialState()));
      ret.setEndState(convertEnumToByte(packet.getEndState()));

      return ret;
   }

   public static HighLevelStateChangePacket convertToPacket(HighLevelStateChangePacketMessage message)
   {
      HighLevelStateChangePacket ret = new HighLevelStateChangePacket(convertByteToEnum(HighLevelState.class, message.getInitialState()),
            convertByteToEnum(HighLevelState.class, message.getEndState()));

      return ret;
   }

   public static FingerStatePacketMessage convertToRosMessage(HandDesiredConfigurationMessage packet)
   {
      FingerStatePacketMessage ret = messageFactory.newFromType("ihmc_msgs/FingerStatePacketMessage");
      ret.setUniqueId(packet.getUniqueId());
      ret.setRobotSide(convertEnumToByte(packet.getRobotSide()));
      ret.setFingerState(convertEnumToByte(packet.getFingerState()));

      return ret;
   }

   public static HandDesiredConfigurationMessage convertToPacket(FingerStatePacketMessage message)
   {
      HandDesiredConfigurationMessage ret = new HandDesiredConfigurationMessage(convertByteToEnum(RobotSide.class, message.getRobotSide()),
            convertByteToEnum(FingerState.class, message.getFingerState()));
      ret.setUniqueId(message.getUniqueId());

      return ret;
   }
}
