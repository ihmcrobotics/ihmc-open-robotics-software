package us.ihmc.avatar.sakeGripper;

public class SakeHandParameters
{
   public static int MAX_ANGLE_BETWEEN_FINGERS = 210;    // When hand is fully open, fingertips form 210 degrees angle
   public static int CLOSED_FINGER_ANGLE = -3;           // Joint angle of a finger is -3 degrees when fully closed
   public static int OPEN_FINGER_ANGLE = 102;            // Joint angle of a finger is 102 degrees when fully open
   public static int MAX_TORQUE_NEWTONS = 29;            // Sake hand can produce 39N of torque (roughly approximated)
}
