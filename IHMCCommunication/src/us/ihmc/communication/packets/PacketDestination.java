package us.ihmc.communication.packets;

public enum PacketDestination
{
   BROADCAST,
   CONTROLLER,
   NETWORK_PROCESSOR,
   UI,
   BEHAVIOR_MODULE,
   LEFT_HAND,
   RIGHT_HAND,
   SENSOR_MANAGER,
   ROS_MODULE,
   MOCAP_MODULE,
   TRAFFIC_SHAPER,
   MULTISENSE_TEST_MODULE,
   ROS_API,
   AUXILIARY_ROBOT_DATA_PUBLISHER,
   ZERO_POSE_PRODUCER,
   TEXT_TO_SPEECH,
   DRILL_DETECTOR,
   AUDIO_MODULE,
   KINEMATICS_TOOLBOX_MODULE,
   FOOTSTEP_PLANNING_TOOLBOX_MODULE,
   REA_MODULE, // Destination for the robot environment awareness module. Not yet available in the open source repo.
   HEIGHT_QUADTREE_TOOLBOX_MODULE,
   LIDAR_SCAN_LOGGER,
   OBJECT_DETECTOR;

   public static final PacketDestination[] values = values();

   public static final PacketDestination fromOrdinal(int ordinal)
   {
      for (PacketDestination packetDestination : values)
      {
         if (ordinal == packetDestination.ordinal())
            return packetDestination;
      }

      return null;
   }
}
