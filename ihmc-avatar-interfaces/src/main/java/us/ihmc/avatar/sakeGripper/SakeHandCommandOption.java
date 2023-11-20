package us.ihmc.avatar.sakeGripper;

public enum SakeHandCommandOption
{
   /*
   The following commands are matched to the HandSakeDesiredCommandMessage
   A goal value of -1.0 indicates the command is not associated with that goal value
   e.g. RESET does not change goal position; The hand remains in same position.
   */
   FULLY_OPEN(0, 0.0, 0.3),
   CLOSE(1, 1.0, 0.3),
   GRIP(2, 1.0, 0.7),
   CUSTOM(3, -1.0, -1.0),
   CALIBRATE(4, 1.0, 0.3),

   // Below commands are predefined custom commands that map to the CUSTOM command (commandNumber = 3)
   OPEN(3, 0.5, 0.3),
   RESET(3, -1.0, 0.0);

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
