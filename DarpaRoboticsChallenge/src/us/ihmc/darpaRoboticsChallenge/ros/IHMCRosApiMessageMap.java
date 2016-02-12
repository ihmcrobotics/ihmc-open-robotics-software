package us.ihmc.darpaRoboticsChallenge.ros;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import us.ihmc.humanoidRobotics.communication.packets.HighLevelStatePacket;
import us.ihmc.humanoidRobotics.communication.packets.LegCompliancePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasDesiredPumpPSIPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasElectricMotorEnablePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.AtlasWristSensorCalibrationRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandComplianceControlParametersPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.ComHeightPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadOrientationPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.JointAnglesPacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.MultiJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryPacket;

/**
 * Created by agrabertilton on 10/10/14.
 */
public class IHMCRosApiMessageMap
{
   public static final Class[] PACKET_LIST =
   {
	   HandPosePacket.class,
	   ComHeightPacket.class,
	   FootPosePacket.class,
	   FootstepDataMessage.class,
	   FootstepDataListMessage.class,
	   FootstepStatus.class,
	   ChestOrientationPacket.class,
	   HeadOrientationPacket.class,
	   PauseWalkingMessage.class,
	   HighLevelStatePacket.class,
	   ArmTrajectoryMessage.class,
	   JointAnglesPacket.class,
	   AtlasElectricMotorEnablePacket.class,
      AtlasWristSensorCalibrationRequestPacket.class,
      AtlasDesiredPumpPSIPacket.class,
      MultiJointAnglePacket.class,
      HandComplianceControlParametersPacket.class,
      LegCompliancePacket.class,
      WholeBodyTrajectoryPacket.class,
      StopAllTrajectoryMessage.class,
      HandDesiredConfigurationMessage.class
   };

   public static final Class[] INPUT_PACKET_LIST =
   {
      HandPosePacket.class,
      ComHeightPacket.class,
      FootPosePacket.class,
      FootstepDataListMessage.class,
      ChestOrientationPacket.class,
      HeadOrientationPacket.class,
      PauseWalkingMessage.class,
      HighLevelStatePacket.class,
      ArmTrajectoryMessage.class,
      JointAnglesPacket.class,
      AtlasElectricMotorEnablePacket.class,
      AtlasWristSensorCalibrationRequestPacket.class,
      AtlasDesiredPumpPSIPacket.class,
      MultiJointAnglePacket.class,
      HandComplianceControlParametersPacket.class,
      LegCompliancePacket.class,
      WholeBodyTrajectoryPacket.class,
      StopAllTrajectoryMessage.class
   };

   public static final Class[] OUTPUT_PACKET_LIST =
   {
      FootstepStatus.class,
   };

   public static final Map<String, Class> PACKET_MESSAGE_NAME_MAP;
   public static final Map<String, Class> INPUT_PACKET_MESSAGE_NAME_MAP;
   public static final Map<String, Class> OUTPUT_PACKET_MESSAGE_NAME_MAP;
   public static final Map<Class, String> PACKET_TO_TOPIC_MAP = new HashMap<>();
   
   public static final Map<Class, String> MESSAGE_NAME_PACKET_MAP;
   
   public static final String JOINT_STATE_TOPIC = "/output/joint_states";

   static
   {
      PACKET_MESSAGE_NAME_MAP = setupMaps(PACKET_LIST);
      INPUT_PACKET_MESSAGE_NAME_MAP = setupMaps(INPUT_PACKET_LIST);
      OUTPUT_PACKET_MESSAGE_NAME_MAP = setupMaps(OUTPUT_PACKET_LIST);
      MESSAGE_NAME_PACKET_MAP = invertMap(PACKET_MESSAGE_NAME_MAP);

      //inputs
      PACKET_TO_TOPIC_MAP.put(HandPosePacket.class, "/control/hand_pose");
      PACKET_TO_TOPIC_MAP.put(ComHeightPacket.class, "/control/com_height");
      PACKET_TO_TOPIC_MAP.put(FootPosePacket.class, "/control/foot_pose");
      PACKET_TO_TOPIC_MAP.put(FootstepDataListMessage.class, "/control/footstep_list");
      PACKET_TO_TOPIC_MAP.put(ChestOrientationPacket.class, "/control/chest_orientation");
      PACKET_TO_TOPIC_MAP.put(HeadOrientationPacket.class, "/control/head_orientation");
      PACKET_TO_TOPIC_MAP.put(PauseWalkingMessage.class, "/control/pause_footstep_exec");
      PACKET_TO_TOPIC_MAP.put(HighLevelStatePacket.class, "/control/high_level_state");
      PACKET_TO_TOPIC_MAP.put(ArmTrajectoryMessage.class, "/control/arm_joint_trajectory");
      PACKET_TO_TOPIC_MAP.put(JointAnglesPacket.class, "/control/joint_angles");
      PACKET_TO_TOPIC_MAP.put(AtlasElectricMotorEnablePacket.class, "/control/enable_electric_motor");
      PACKET_TO_TOPIC_MAP.put(AtlasWristSensorCalibrationRequestPacket.class, "/control/wrist_sensor_calibration");
      PACKET_TO_TOPIC_MAP.put(AtlasDesiredPumpPSIPacket.class, "/control/desired_pump_psi");
      PACKET_TO_TOPIC_MAP.put(MultiJointAnglePacket.class, "/control/multi_joint_angles");
      PACKET_TO_TOPIC_MAP.put(HandComplianceControlParametersPacket.class, "/control/hand_compliance_control_parameters");
      PACKET_TO_TOPIC_MAP.put(LegCompliancePacket.class, "/control/leg_compliance_control_parameters");
      PACKET_TO_TOPIC_MAP.put(WholeBodyTrajectoryPacket.class, "/control/whole_body_trajectory");
      PACKET_TO_TOPIC_MAP.put(StopAllTrajectoryMessage.class, "/control/stop_motion");

      //outputs
      PACKET_TO_TOPIC_MAP.put(FootstepStatus.class, "/output/footstep_status");
   }

   private static Map<String, Class> setupMaps(Class[] packetList)
   {
      Map<String, Class> initMap = new HashMap<String, Class>(packetList.length);
      for (Class clazz : packetList)
      {
          initMap.put("ihmc_msgs/" + clazz.getSimpleName()+"Message", clazz);
      }
      return Collections.unmodifiableMap(initMap);
   }
   
   private static Map<Class, String> invertMap(Map<String, Class> original)
   {
      Map<Class, String> map = new HashMap<>();
      for(Entry<String, Class> entry : original.entrySet()){
         map.put(entry.getValue(), entry.getKey());
     }
      return Collections.unmodifiableMap(map);
   }

}
