package us.ihmc.avatar.sakeGripper;

public enum SakeHandCommandOption
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

   SakeHandCommandOption(int commandNumber, double goalPosition, double goalTorque)
   {
      this.commandNumber = commandNumber;
      this.goalPosition = goalPosition;
      this.goalTorque = goalTorque;
   }

   public final static SakeHandCommandOption[] values = values();

   public SakeHandCommandOption fromByte(byte enumAsByte)
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
