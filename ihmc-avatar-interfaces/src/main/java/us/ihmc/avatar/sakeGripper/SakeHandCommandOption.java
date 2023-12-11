package us.ihmc.avatar.sakeGripper;

public enum SakeHandCommandOption
{
   /*
   The following commands are matched to the HandSakeDesiredCommandMessage
   A goal value of -1.0 indicates the command is not associated with that goal value
   e.g. RESET does not change goal position; The hand remains in same position.
   */
   FULLY_OPEN(0, false, 0.0, 0.3),
   CLOSE(1, false, 1.0, 0.3),
   GRIP(2, false, 1.0, 0.7),
   CUSTOM(3, false, -1.0, -1.0),
   CALIBRATE(4, false, 1.0, 0.3),

   // Below commands are predefined custom commands that map to the CUSTOM command (commandNumber = 3)
   OPEN(3, false, 0.5, 0.3),
   RESET(3, true, -1.0, 0.0),
   CONFIRM_ERROR(3, true, -1.0, -1.0);

   private final int commandNumber;
   private final boolean errorConfirmation;
   private final double desiredPosition;
   private final double desiredTorque;

   SakeHandCommandOption(int commandNumber, boolean errorConfirmation, double desiredPosition, double desiredTorque)
   {
      this.commandNumber = commandNumber;
      this.errorConfirmation = errorConfirmation;
      this.desiredPosition = desiredPosition;
      this.desiredTorque = desiredTorque;
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
   public boolean getErrorConfirmation()
   {
      return errorConfirmation;
   }
   public double getDesiredPosition()
   {
      return desiredPosition;
   }
   public double getDesiredTorque()
   {
      return desiredTorque;
   }
}
