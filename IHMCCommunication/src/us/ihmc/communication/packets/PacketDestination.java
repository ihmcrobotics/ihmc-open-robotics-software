package us.ihmc.communication.packets;

public enum PacketDestination
{
   BROADCAST,
   CONTROLLER,
   NETWORK_PROCESSOR,
   UI,
   BEHAVIOR_MODULE,
   SCS_SENSORS,
   LEFT_HAND_MANAGER,
   RIGHT_HAND_MANAGER,
   SENSOR_MANAGER,
   PERCEPTION_MODULE,
   ROS_MODULE,
   MOCAP_MODULE,
   SCRIPTED_FOOTSTEP_DATA_LIST,
   TRAFFIC_SHAPER,
   MULTISENSE_TEST_MODULE,
   GFE, 
   AUXILIARY_ROBOT_DATA_PUBLISHER;

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
