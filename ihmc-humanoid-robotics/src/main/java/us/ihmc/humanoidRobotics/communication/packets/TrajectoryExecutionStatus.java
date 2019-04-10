package us.ihmc.humanoidRobotics.communication.packets;

/**
 * Enum used when reporting an update about the execution of a trajectory in the controller.
 * 
 * @author Sylvain Bertrand
 */
public enum TrajectoryExecutionStatus
{
   /** The trajectory input was received and accepted, the execution just started. */
   STARTED,
   /** The trajectory input was received, accepted, and its execution just finished. */
   COMPLETED;

   public static final TrajectoryExecutionStatus[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static TrajectoryExecutionStatus fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
