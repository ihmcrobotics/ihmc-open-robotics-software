package us.ihmc.perception.sceneGraph.rigidBody.doors;

public enum OpeningMechanismType
{
   UNKNOWN,
   LEVER_HANDLE,
   KNOB,
   PUSH_BAR,
   PULL_HANDLE;

   public static OpeningMechanismType fromByte(byte b)
   {
      return OpeningMechanismType.values()[b];
   }
}
