package us.ihmc.gr00t;

import org.junit.jupiter.api.Test;
import org.msgpack.core.MessageBufferPacker;
import org.msgpack.core.MessagePack;
import us.ihmc.robotics.robotSide.SideDependentList;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;

class Gr00tClientTest
{
   @Test
   void validatesTheAdvertisedBridgeContract() throws Exception
   {
      Gr00tClient client = client();
      assertDoesNotThrow(() -> client.validateServerMetadata(metadata("alex_lever28_v1", 28, 28, 16, 10.0)));
      assertThrows(IllegalArgumentException.class,
                   () -> client.validateServerMetadata(metadata("wrong_layout", 28, 28, 16, 10.0)));
      assertThrows(IllegalArgumentException.class,
                   () -> client.validateServerMetadata(metadata("alex_lever28_v1", 28, 28, 8, 10.0)));
   }

   @Test
   void rejectsMissingContractMetadata()
   {
      Gr00tClient client = client();
      assertThrows(IllegalArgumentException.class, () -> client.validateServerMetadata(emptyMetadata()));
   }

   private static Gr00tClient client()
   {
      return new Gr00tClient("localhost",
                             8000,
                             28,
                             28,
                             50,
                             640,
                             480,
                             new SideDependentList<>("cam_zed_left", "cam_zed_right"),
                             "state",
                             "prompt",
                             "actions",
                             "alex_lever28_v1",
                             16,
                             10.0);
   }

   private static byte[] metadata(String layout, int stateSize, int actionSize, int horizon, double rate) throws Exception
   {
      MessageBufferPacker packer = MessagePack.newDefaultBufferPacker();
      packer.packMapHeader(1).packString("contract").packMapHeader(5);
      packer.packString("layout_id").packString(layout);
      packer.packString("state_size").packInt(stateSize);
      packer.packString("action_size").packInt(actionSize);
      packer.packString("action_horizon").packInt(horizon);
      packer.packString("action_rate_hz").packDouble(rate);
      packer.close();
      return packer.toByteArray();
   }

   private static byte[] emptyMetadata() throws Exception
   {
      MessageBufferPacker packer = MessagePack.newDefaultBufferPacker();
      packer.packMapHeader(0);
      packer.close();
      return packer.toByteArray();
   }
}
