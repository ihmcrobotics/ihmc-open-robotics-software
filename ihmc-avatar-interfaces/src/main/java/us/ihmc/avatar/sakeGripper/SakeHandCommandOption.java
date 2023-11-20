package us.ihmc.avatar.sakeGripper;

public enum SakeHandCommandOption
{
   /*
   The following commands are matched to the HandSakeDesiredCommandMessage
   A goal value of -1.0 indicates the command is not associated with that goal value
   e.g. RESET does not change goal position; The hand remains in same position.
   */
   FULLY_OPEN(0, 1.0, 0.3),
   CLOSE(1, 0.1, 0.3),
   GRIP(2, 0.0, 0.3),
   CUSTOM(3, -1.0, -1.0),
   CALIBRATE(4, 0.0, 0.3),

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
