package us.ihmc.gr00t;

import org.msgpack.core.MessagePack;
import org.msgpack.core.MessageUnpacker;
import org.msgpack.value.Value;
import us.ihmc.openpi.OpenpiClient;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Objects;

/** Client for a GR00T server using the configurable OpenPI websocket/msgpack protocol. */
public class Gr00tClient extends OpenpiClient
{
   private final String expectedLayoutId;
   private final int expectedActionHorizon;
   private final double expectedActionRateHz;

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
      String layoutId = null;
      Integer stateSize = null;
      Integer actionSize = null;
      Integer actionHorizon = null;
      Double actionRateHz = null;
      try (MessageUnpacker unpacker = MessagePack.newDefaultUnpacker(metadata))
      {
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
               String contractKey = unpacker.unpackString();
               switch (contractKey)
               {
                  case "layout_id" -> layoutId = unpacker.unpackString();
                  case "state_size" -> stateSize = unpacker.unpackInt();
                  case "action_size" -> actionSize = unpacker.unpackInt();
                  case "action_horizon" -> actionHorizon = unpacker.unpackInt();
                  case "action_rate_hz" ->
                  {
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

      requireMetadata("layout_id", expectedLayoutId, layoutId);
      requireMetadata("state_size", getStateSize(), stateSize);
      requireMetadata("action_size", getActionSize(), actionSize);
      requireMetadata("action_horizon", expectedActionHorizon, actionHorizon);
      requireMetadata("action_rate_hz", expectedActionRateHz, actionRateHz);
   }

   private static void requireMetadata(String field, Object expected, Object actual)
   {
      if (!Objects.equals(expected, actual))
         throw new IllegalArgumentException("Incompatible GR00T bridge " + field + ": expected " + expected + ", got " + actual);
   }
}
