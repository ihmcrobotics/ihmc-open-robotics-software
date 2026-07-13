package us.ihmc.communication.ros2log;


/**
 * Verbose way of expressing ROS 2 log state. Setup as enum in case future states are needed.
 */
public enum ROS2LoggerRequestedState
{
   /* Start recording/replaying */
   START,
   /* Finish recording/replaying */
   FINISH;

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static ROS2LoggerRequestedState fromByte(byte enumAsByte)
   {
      if (enumAsByte < 0 || enumAsByte >= values().length)
         return null;
      return values()[enumAsByte];
   }
}
