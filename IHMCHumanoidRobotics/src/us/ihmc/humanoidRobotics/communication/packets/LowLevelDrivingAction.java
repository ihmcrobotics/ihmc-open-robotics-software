package us.ihmc.humanoidRobotics.communication.packets;

public enum LowLevelDrivingAction
{
   /** [0,1] 0 = off 1 = max */
   GASPEDAL, /** [0,1] 0 = off 1 = max */
   FOOTBRAKE, /** [-pi,pi] */
   STEERING, /** -1 reverse, 1 forward */
   DIRECTION, /** -1 disengage, 1 engage, -2.0 redisengage, 2 enage */
   HANDBRAKE, /** in seconds */
   DO_NOTHING, /** Stop, let go of steering wheel and press brake */
   REINITIALIZE, /** Easy cheat mode */
   GET_IN_CAR;

   public int toInt()
   {
      switch (this)
      {
      case GASPEDAL:
         return 0;
      case FOOTBRAKE:
         return 1;
      case STEERING:
         return 2;
      case DIRECTION:
         return 3;
      case HANDBRAKE:
         return 4;
      case DO_NOTHING:
         return 5;
      case GET_IN_CAR:
         return 6;
      case REINITIALIZE:
         return 7;
      default:
         throw new RuntimeException("Unknown action");
      }
   }

   public static LowLevelDrivingAction fromInt(int value)
   {
      switch (value)
      {
      case 0:
         return GASPEDAL;
      case 1:
         return FOOTBRAKE;
      case 2:
         return STEERING;
      case 3:
         return DIRECTION;
      case 4:
         return HANDBRAKE;
      case 5:
         return DO_NOTHING;
      case 6:
         return GET_IN_CAR;
      case 7:
         return REINITIALIZE;
      default:
         throw new RuntimeException("Unknown action");
      }
   }
}