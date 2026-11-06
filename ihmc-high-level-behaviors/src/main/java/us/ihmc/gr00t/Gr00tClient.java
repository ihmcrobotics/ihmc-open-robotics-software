package us.ihmc.gr00t;

import org.msgpack.core.MessagePack;
import org.msgpack.core.MessageUnpacker;
import org.msgpack.value.Value;
import us.ihmc.openpi.OpenpiClient;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Objects;

/**
 * GR00T-specific adapter around {@link OpenpiClient}'s websocket and MessagePack transport.
 * <p>
 * The base client owns connection setup, observation serialization, and action-tensor decoding.
 * This class adds the part that is unique to a trained GR00T deployment: before the first
 * observation is sent, the server must advertise a contract describing the embodiment layout,
 * tensor sizes, action horizon, and sampling rate. Treating that metadata as a required handshake
 * prevents a visually plausible but incompatible checkpoint from commanding the robot with a
 * different joint order or time base.
 * <p>
 * Connection sequence:
 * <ol>
 *    <li>The websocket connects and receives the bridge's initial metadata message.</li>
 *    <li>{@link #validateServerMetadata(byte[])} extracts the nested {@code contract} map.</li>
 *    <li>Every required value is compared with the client configuration.</li>
 *    <li>Only an exact match allows {@link OpenpiClient} to begin sending observations.</li>
 * </ol>
 */
public class Gr00tClient extends OpenpiClient
{
   /** Semantic identifier for the exact ordering and meaning of every state/action element. */
   private final String expectedLayoutId;
   /** Number of action rows the deployed policy promises to make meaningful in each response. */
   private final int expectedActionHorizon;
   /** Dataset sampling rate used to interpret spacing between consecutive policy rows. */
   private final double expectedActionRateHz;

   /**
    * Configures both the generic wire representation and the GR00T embodiment contract.
    *
    * @param chunkLength allocated rows in the action tensor; may exceed the meaningful horizon
    * @param imageKeys left/right names expected by the deployed bridge
    * @param stateKey MessagePack key for the robot-state tensor
    * @param promptKey MessagePack key for the language instruction
    * @param actionKey MessagePack key containing the returned action tensor
    * @param expectedLayoutId versioned semantic layout identifier advertised by the bridge
    * @param expectedActionHorizon meaningful action rows returned per inference
    * @param expectedActionRateHz time base of consecutive action rows
    */
   public Gr00tClient(String host,
                      int port,
                      int stateSize,
                      int actionSize,
                      int chunkLength,
                      int imageWidth,
                      int imageHeight,
                      SideDependentList<String> imageKeys,
                      String stateKey,
                      String promptKey,
                      String actionKey,
                      String expectedLayoutId,
                      int expectedActionHorizon,
                      double expectedActionRateHz)
   {
      super(host, port, stateSize, actionSize, chunkLength, imageWidth, imageHeight, "", imageKeys, stateKey, promptKey, actionKey);
      if (expectedLayoutId == null || expectedLayoutId.isBlank())
         throw new IllegalArgumentException("Expected GR00T layout ID must not be blank");
      if (expectedActionHorizon <= 0 || expectedActionHorizon > chunkLength)
         throw new IllegalArgumentException("Expected GR00T action horizon must be in [1, " + chunkLength + "]");
      if (!Double.isFinite(expectedActionRateHz) || expectedActionRateHz <= 0.0)
         throw new IllegalArgumentException("Expected GR00T action rate must be finite and positive");
      this.expectedLayoutId = expectedLayoutId;
      this.expectedActionHorizon = expectedActionHorizon;
      this.expectedActionRateHz = expectedActionRateHz;
   }

   @Override
   protected void validateServerMetadata(byte[] metadata) throws Exception
   {
      // Use boxed values initialized to null so an omitted field fails the same exact-match check
      // as an incorrect field. Missing metadata must never silently fall back to local defaults.
      String layoutId = null;
      Integer stateSize = null;
      Integer actionSize = null;
      Integer actionHorizon = null;
      Double actionRateHz = null;
      try (MessageUnpacker unpacker = MessagePack.newDefaultUnpacker(metadata))
      {
         // The bridge may publish unrelated metadata alongside the contract. MessagePack values
         // are explicitly skipped so adding a server diagnostic does not break older clients.
         int metadataFields = unpacker.unpackMapHeader();
         for (int field = 0; field < metadataFields; field++)
         {
            String key = unpacker.unpackString();
            if (!"contract".equals(key))
            {
               unpacker.skipValue();
               continue;
            }

            int contractFields = unpacker.unpackMapHeader();
            for (int contractField = 0; contractField < contractFields; contractField++)
            {
               // Field order is intentionally irrelevant; MessagePack maps are not ordered wire
               // structs, and Python bridge implementations may emit keys in a different order.
               String contractKey = unpacker.unpackString();
               switch (contractKey)
               {
                  case "layout_id" -> layoutId = unpacker.unpackString();
                  case "state_size" -> stateSize = unpacker.unpackInt();
                  case "action_size" -> actionSize = unpacker.unpackInt();
                  case "action_horizon" -> actionHorizon = unpacker.unpackInt();
                  case "action_rate_hz" ->
                  {
                     // Python MessagePack encoders may choose an integer or floating-point number
                     // for values such as 10 Hz, so accept either numeric representation.
                     Value value = unpacker.unpackValue();
                     if (!value.isNumberValue())
                        throw new IllegalArgumentException("GR00T action_rate_hz is not numeric");
                     actionRateHz = value.asNumberValue().toDouble();
                  }
                  default -> unpacker.skipValue();
               }
            }
         }
      }

      // Tensor dimensions alone are insufficient: two 28-element layouts can assign completely
      // different meanings to the same offsets. Require both shape and semantic identity.
      requireMetadata("layout_id", expectedLayoutId, layoutId);
      requireMetadata("state_size", getStateSize(), stateSize);
      requireMetadata("action_size", getActionSize(), actionSize);
      requireMetadata("action_horizon", expectedActionHorizon, actionHorizon);
      requireMetadata("action_rate_hz", expectedActionRateHz, actionRateHz);
   }

   private static void requireMetadata(String field, Object expected, Object actual)
   {
      // Objects.equals deliberately makes a missing value (null) fail with the same actionable
      // error message as a value supplied by the wrong checkpoint.
      if (!Objects.equals(expected, actual))
         throw new IllegalArgumentException("Incompatible GR00T bridge " + field + ": expected " + expected + ", got " + actual);
   }
}
