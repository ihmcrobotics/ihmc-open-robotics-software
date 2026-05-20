package us.ihmc.communication.packets;

public enum PacketDestination
{
   CONTROLLER,
   BEHAVIOR_MODULE,
   KINEMATICS_TOOLBOX_MODULE;

   public static final PacketDestination[] values = values();
}
