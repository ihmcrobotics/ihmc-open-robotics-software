package us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector;

import java.util.HashMap;

import us.ihmc.communication.packets.bdi.BDIBehaviorStatusPacket;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModeResponsePacket;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorStatusPacket;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.ControlStatusPacket;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandRotateAboutAxisPacket;
import us.ihmc.communication.packets.sensing.DepthDataClearCommand;
import us.ihmc.communication.packets.sensing.HeadPosePacket;
import us.ihmc.communication.packets.sensing.IntrinsicCameraParametersPacket;
import us.ihmc.communication.packets.sensing.PelvisPoseErrorPacket;
import us.ihmc.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.communication.packets.sensing.RawIMUPacket;
import us.ihmc.communication.packets.sensing.VideoPacket;
import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.packets.walking.FootstepStatus;
import us.ihmc.communication.packets.walking.HeadOrientationPacket;
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.communication.packets.walking.SnapFootstepPacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;

public class PacketsForwardedToTheUi
{
   public static final long UI_JOINT_CONFIGURATION_UPDATE_MILLIS = 100l;
   public static final long UI_WRIST_FEET_SENSORS_UPDATE_MILLIS = 500l;
   public static final long UI_MULTISENSE_IMU_CHECK_MILLIS=5000l;
   
   public static Class[] PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE = {
      FootstepStatus.class,                          
      ScriptBehaviorStatusPacket.class,
      PelvisPoseErrorPacket.class,                   
      HumanoidBehaviorControlModeResponsePacket.class,
      BDIBehaviorStatusPacket.class,                 
      ControlStatusPacket.class,                     
      FootstepDataList.class,                        
      ComHeightPacket.class,                         
      HeadOrientationPacket.class,                   
      PelvisPosePacket.class,                        
      ChestOrientationPacket.class,                  
      SnapFootstepPacket.class,
      VideoPacket.class,
      HandPosePacket.class,
      HandRotateAboutAxisPacket.class,
      DepthDataClearCommand.class,
      IntrinsicCameraParametersPacket.class,
      PointCloudWorldPacket.class,
      HandJointAnglePacket.class,
      WholeBodyTrajectoryPacket.class
   };
   
   public static final HashMap<Class, Long> PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE_WITH_MINIMAL_INTERVALS = new HashMap<Class, Long>();
   static {
//      PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE_WITH_MINIMAL_INTERVALS.put(RobotPoseData.class, UI_JOINT_CONFIGURATION_UPDATE_MILLIS);
      PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE_WITH_MINIMAL_INTERVALS.put(CapturabilityBasedStatus.class, UI_JOINT_CONFIGURATION_UPDATE_MILLIS);
      PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE_WITH_MINIMAL_INTERVALS.put(RobotConfigurationData.class, UI_JOINT_CONFIGURATION_UPDATE_MILLIS);
      PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE_WITH_MINIMAL_INTERVALS.put(HeadPosePacket.class, UI_MULTISENSE_IMU_CHECK_MILLIS);
      PACKETS_ALLOWED_TO_BE_SENT_TO_THE_USER_INTERFACE_WITH_MINIMAL_INTERVALS.put(RawIMUPacket.class, UI_MULTISENSE_IMU_CHECK_MILLIS);
   }
   
   
}
