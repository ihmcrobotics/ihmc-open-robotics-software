package us.ihmc.perception.sceneGraph.rigidBody.doors;

public enum DoorHardwareType
{
   UNKNOWN,
   LEVER_HANDLE,
   KNOB,
   PUSH_BAR,
   PULL_HANDLE;

   public static DoorHardwareType fromByte(byte b)
   {
      return DoorHardwareType.values()[b];
   }
}
