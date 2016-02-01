package us.ihmc.humanoidRobotics.communication.packets.dataobjects;

public enum BlindWalkingDirection
{
   FORWARD, BACKWARD, LEFT, RIGHT;
   
   
   public int getIntegerValue()
   {
      switch(this)
      {
      case FORWARD:
         return 0;
      case BACKWARD:
         return 1;
      case LEFT:
         return 2;
      case RIGHT:
         return 3;
      default:
         throw new RuntimeException();
      }
   }
   
   public static BlindWalkingDirection fromIntegerValue(int val)
   {
      switch(val)
      {
      case 0:
         return FORWARD;
      case 1:
         return BACKWARD;
      case 2:
         return LEFT;
      case 3:
         return RIGHT;
      default:
         throw new RuntimeException();
      }
   }
}