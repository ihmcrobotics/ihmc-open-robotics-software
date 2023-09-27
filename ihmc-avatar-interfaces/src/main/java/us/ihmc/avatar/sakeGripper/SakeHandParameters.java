package us.ihmc.avatar.sakeGripper;

public class SakeHandParameters
{
   public enum SakeCommandOption
   {
      /*
         The following commands are matched to the HandSakeDesiredCommandMessage
         A goal value of -1.0 indicates the command is not associated with that goal value
         e.g. RESET does not change goal position; The hand remains in same position.
       */
      CALIBRATE(0, 0.0, 0.3),
      RESET(1, -1.0, 0.0),
      FULLY_OPEN(2, 1.0, 0.3),
      CLOSE(3, 0.1, 0.3),
      RELEASE(4, -1.0, 0.0),
      GOTO(5, -1.0, -1.0),
      GRIP(6, 0.0, 0.3),
      GRIP_HARD(7, 0.0, 1.0),

      // Below commands are custom defined commands that map to the GOTO command (commandNumber = 5)
      OPEN(5, 0.5, 0.3);

      private final int commandNumber;
      private final double goalPosition;
      private final double goalTorque;

      SakeCommandOption(int commandNumber, double goalPosition, double goalTorque)
      {
         this.commandNumber = commandNumber;
         this.goalPosition = goalPosition;
         this.goalTorque = goalTorque;
      }

      public final static SakeCommandOption[] values = values();

      public SakeCommandOption fromByte(byte enumAsByte)
      {
         return values[enumAsByte];
      }
      public byte toByte()
      {
         return (byte) ordinal();
      }
      public int getCommandNumber()
      {
         return commandNumber;
      }
      public double getGoalPosition()
      {
         return goalPosition;
      }
      public double getGoalTorque()
      {
         return goalTorque;
      }
   }

   public static int MAX_ANGLE_BETWEEN_FINGERS = 210;    // When hand is fully open, fingertips form 210 degrees angle
   public static int CLOSED_FINGER_ANGLE = -3;           // Joint angle of a finger is -3 degrees when fully closed
   public static int OPEN_FINGER_ANGLE = 102;            // Joint angle of a finger is 102 degrees when fully open
   public static int MAX_TORQUE_NEWTONS = 29;            // Sake hand can produce 39N of torque (roughly approximated)
}
