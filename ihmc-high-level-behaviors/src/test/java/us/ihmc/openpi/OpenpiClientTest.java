package us.ihmc.openpi;

import org.junit.jupiter.api.Test;
import org.msgpack.core.MessageBufferPacker;
import org.msgpack.core.MessagePack;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.CompletableFuture;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertThrows;

class OpenpiClientTest
{
   @Test
   void acceptsIndependentActionSizeAndUnorderedResponseFields() throws Exception
   {
      OpenpiClient client = new OpenpiClient("localhost",
                                             8000,
                                             4,
                                             2,
                                             3,
                                             8,
                                             6,
                                             "go",
                                             new SideDependentList<>("left_rgb", "right_rgb"),
                                             "observation",
                                             "instruction",
                                             "trajectory");
      byte[] actionPayload = ByteBuffer.allocate(3 * 2 * Double.BYTES)
                                           .order(ByteOrder.nativeOrder())
                                           .putDouble(1.0)
                                           .putDouble(2.0)
                                           .putDouble(3.0)
                                           .putDouble(4.0)
                                           .putDouble(5.0)
                                           .putDouble(6.0)
                                           .array();

      MessageBufferPacker packer = MessagePack.newDefaultBufferPacker();
      packer.packMapHeader(5);
      packer.packString("horizon").packInt(2);
      packer.packString("extra").packString("ignored");
      packer.packString("server_timing").packMapHeader(1).packString("infer_ms").packDouble(12.5);
      packer.packString("trajectory").packMapHeader(4);
      packer.packString("shape").packArrayHeader(2).packInt(3).packInt(2);
      packer.packString("dtype").packString("<f8");
      packer.packString("data").packBinaryHeader(actionPayload.length).writePayload(actionPayload);
      packer.packString("__ndarray__").packBoolean(true);
      packer.packString("policy_timing").packMapHeader(1).packString("infer_ms").packDouble(7.25);
      packer.close();

      assertTrue(client.unpack(CompletableFuture.completedFuture(packer.toByteArray())));

      assertEquals(2, client.getActionSize());
      assertEquals(2, client.getHorizon());
      assertEquals(7.25f, client.getPolicyTimingMs());
      assertEquals(12.5f, client.getServerTimingMs());
      assertEquals(1.0, client.getActionChunk().asDoubleBuffer().get(0));
      assertEquals(6.0, client.getActionChunk().asDoubleBuffer().get(5));
   }

   @Test
   void malformedResponseClearsActionsAndHorizon() throws Exception
   {
      OpenpiClient client = new OpenpiClient("localhost", 8000, 4, 2, 3, 8, 6, "go",
                                             new SideDependentList<>("left_rgb", "right_rgb"),
                                             "observation", "instruction", "trajectory");
      MessageBufferPacker packer = MessagePack.newDefaultBufferPacker();
      packer.packMapHeader(1);
      packer.packString("trajectory").packMapHeader(4);
      packer.packString("shape").packArrayHeader(2).packInt(99).packInt(2);
      packer.packString("dtype").packString("<f8");
      packer.packString("data").packBinaryHeader(0);
      packer.packString("__ndarray__").packBoolean(true);
      packer.close();

      assertFalse(client.unpack(CompletableFuture.completedFuture(packer.toByteArray())));
      assertEquals(0, client.getHorizon());
      assertEquals(0.0, client.getActionChunk().asDoubleBuffer().get(0));
   }

   @Test
   void rejectsNonPositiveTensorDimensions()
   {
      assertThrows(IllegalArgumentException.class,
                   () -> new OpenpiClient("localhost",
                                          8000,
                                          4,
                                          0,
                                          3,
                                          8,
                                          6,
                                          "go",
                                          new SideDependentList<>("left_rgb", "right_rgb"),
                                          "observation",
                                          "instruction",
                                          "trajectory"));
   }
}
