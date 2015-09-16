package us.ihmc.humanoidRobotics.communication.packets.dataobjects;


public enum BlindWalkingSpeed
{
   SLOW, MEDIUM, FAST;

   public int getIntegerValue()
   {
      switch (this)
      {
      case SLOW:
         return 0;
      case MEDIUM:
         return 1;
      case FAST:
         return 2;
      default:
         throw new RuntimeException();
      }
   }

   public static BlindWalkingSpeed fromIntegerValue(int val)
   {
      switch (val)
      {
      case 0:
         return SLOW;
      case 1:
         return MEDIUM;
      case 2:
         return FAST;
      default:
         throw new RuntimeException();
      }
   }
}
