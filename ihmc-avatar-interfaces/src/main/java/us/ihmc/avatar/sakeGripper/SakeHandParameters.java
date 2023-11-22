package us.ihmc.avatar.sakeGripper;

public class SakeHandParameters
{
   public static int MAX_ANGLE_BETWEEN_FINGERS = 210;    // When hand is fully open, fingertips form 210 degrees angle
   public static int CLOSED_FINGER_ANGLE = -3;           // Joint angle of a finger is -3 degrees when fully closed
   public static int OPEN_FINGER_ANGLE = 102;            // Joint angle of a finger is 102 degrees when fully open
   public static int MAX_TORQUE_NEWTONS = 29;            // Sake hand can produce 39N of torque (roughly approximated)
   public static double MAX_RATIO_VALUE = 1.0;           // Some values are converted to ratios of 0.0 to 1.0
   public static double MIN_RATIO_VALUE = 0.0;

   public static String getErrorString(byte errorValue)
   {
      StringBuilder errorString = new StringBuilder();

      if ((errorValue & 0B00000001) == 1)
         errorString.append("[Input Voltage Error] ");
      if ((errorValue & 0B00000010) == 2)
         errorString.append("[Angle Limit Error]" );
      if ((errorValue & 0B00000100) == 4)
         errorString.append("[Overheating Error] ");
      if ((errorValue & 0B00001000) == 8)
         errorString.append("[Range Error] ");
      if ((errorValue & 0B00010000) == 16)
         errorString.append("[Checksum Error] ");
      if ((errorValue & 0B00100000) == 32)
         errorString.append("[Overload Error] ");
      if ((errorValue & 0B01000000) == 64)
         errorString.append("[Instruction Error] ");

      return errorString.toString();
   }
}
