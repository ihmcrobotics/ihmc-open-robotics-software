package us.ihmc.darpaRoboticsChallenge.ros;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import us.ihmc.communication.packets.HighLevelStatePacket;
import us.ihmc.communication.packets.LegCompliancePacket;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.*;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.communication.packets.walking.FootPosePacket;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.packets.walking.FootstepStatus;
import us.ihmc.communication.packets.walking.HeadOrientationPacket;
import us.ihmc.communication.packets.walking.PauseCommand;
import us.ihmc.communication.packets.wholebody.JointAnglesPacket;
import us.ihmc.communication.packets.wholebody.MultiJointAnglePacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;

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
	   FootstepData.class,
	   FootstepDataList.class,
	   FootstepStatus.class,
	   ChestOrientationPacket.class,
	   HeadOrientationPacket.class,
	   PauseCommand.class,
	   HighLevelStatePacket.class,
	   ArmJointTrajectoryPacket.class,
	   JointAnglesPacket.class,
	   AtlasElectricMotorEnablePacket.class,
      AtlasWristSensorCalibrationRequestPacket.class,
      AtlasDesiredPumpPSIPacket.class,
      MultiJointAnglePacket.class,
      HandComplianceControlParametersPacket.class,
      LegCompliancePacket.class,
      WholeBodyTrajectoryPacket.class
   };

   public static final Class[] INPUT_PACKET_LIST =
   {
      HandPosePacket.class,
      ComHeightPacket.class,
      FootPosePacket.class,
      FootstepDataList.class,
      ChestOrientationPacket.class,
      HeadOrientationPacket.class,
      PauseCommand.class,
      HighLevelStatePacket.class,
      ArmJointTrajectoryPacket.class,
      JointAnglesPacket.class,
      AtlasElectricMotorEnablePacket.class,
      AtlasWristSensorCalibrationRequestPacket.class,
      AtlasDesiredPumpPSIPacket.class,
      MultiJointAnglePacket.class,
      HandComplianceControlParametersPacket.class,
      LegCompliancePacket.class,
      WholeBodyTrajectoryPacket.class
   };

   public static final Class[] OUTPUT_PACKET_LIST =
   {
      FootstepStatus.class,
//      RobotConfigurationData.class
   };

   public static final Map<String, Class> PACKET_MESSAGE_NAME_MAP;
   public static final Map<String, Class> INPUT_PACKET_MESSAGE_NAME_MAP;
   public static final Map<String, Class> OUTPUT_PACKET_MESSAGE_NAME_MAP;
   public static final Map<Class, String> PACKET_TO_TOPIC_MAP = new HashMap<>();

   static
   {
      PACKET_MESSAGE_NAME_MAP = setupMaps(PACKET_LIST);
      INPUT_PACKET_MESSAGE_NAME_MAP = setupMaps(INPUT_PACKET_LIST);
      OUTPUT_PACKET_MESSAGE_NAME_MAP = setupMaps(OUTPUT_PACKET_LIST);

      //inputs
      PACKET_TO_TOPIC_MAP.put(HandPosePacket.class, "/control/hand_pose");
      PACKET_TO_TOPIC_MAP.put(ComHeightPacket.class, "/control/com_height");
      PACKET_TO_TOPIC_MAP.put(FootPosePacket.class, "/control/foot_pose");
      PACKET_TO_TOPIC_MAP.put(FootstepDataList.class, "/control/footstep_list");
      PACKET_TO_TOPIC_MAP.put(ChestOrientationPacket.class, "/control/chest_orientation");
      PACKET_TO_TOPIC_MAP.put(HeadOrientationPacket.class, "/control/head_orientation");
      PACKET_TO_TOPIC_MAP.put(PauseCommand.class, "/control/pause_footstep_exec");
      PACKET_TO_TOPIC_MAP.put(HighLevelStatePacket.class, "/control/high_level_state");
      PACKET_TO_TOPIC_MAP.put(ArmJointTrajectoryPacket.class, "/control/arm_joint_trajectory");
      PACKET_TO_TOPIC_MAP.put(JointAnglesPacket.class, "/control/joint_angles");
      PACKET_TO_TOPIC_MAP.put(AtlasElectricMotorEnablePacket.class, "/control/enable_electric_motor");
      PACKET_TO_TOPIC_MAP.put(AtlasWristSensorCalibrationRequestPacket.class, "/control/wrist_sensor_calibration");
      PACKET_TO_TOPIC_MAP.put(AtlasDesiredPumpPSIPacket.class, "/control/desired_pump_psi");
      PACKET_TO_TOPIC_MAP.put(MultiJointAnglePacket.class, "/control/multi_joint_angles");
      PACKET_TO_TOPIC_MAP.put(HandComplianceControlParametersPacket.class, "/control/hand_compliance_control_parameters");
      PACKET_TO_TOPIC_MAP.put(LegCompliancePacket.class, "/control/leg_compliance_control_parameters");
      PACKET_TO_TOPIC_MAP.put(WholeBodyTrajectoryPacket.class, "/control/whole_body_trajectory");

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

}
