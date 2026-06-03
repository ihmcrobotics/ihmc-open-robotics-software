package us.ihmc.communication.controllerAPI;

/**
 * Constants for controller message / command ID handling.
 * Replaces {@code Packet.INVALID_MESSAGE_ID} from the removed {@code Packet} base class.
 * Matches the IDL default for {@code ihmc_common_msgs/QueueableMessage.message_id} ({@code -1}).
 */
public final class ControllerMessageConstants
{
   public static final long INVALID_MESSAGE_ID = -1L;
   /** Default command / message id after {@link us.ihmc.communication.controllerAPI.command.QueueableCommand#clear()}. */
   public static final long VALID_MESSAGE_DEFAULT_ID = 0L;

   private ControllerMessageConstants()
   {
   }
}
