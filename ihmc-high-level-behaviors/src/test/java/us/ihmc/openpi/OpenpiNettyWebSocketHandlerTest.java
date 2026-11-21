package us.ihmc.openpi;

import io.netty.channel.embedded.EmbeddedChannel;
import org.junit.jupiter.api.Test;

import java.net.URI;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.assertThrows;

class OpenpiNettyWebSocketHandlerTest
{
   @Test
   void disconnectCompletesPendingInferenceExceptionally() throws Exception
   {
      OpenpiNettyWebSocketHandler handler = new OpenpiNettyWebSocketHandler(new URI("ws://localhost:8000"));
      EmbeddedChannel channel = new EmbeddedChannel(handler);
      CompletableFuture<byte[]> response = handler.sendAndAwaitResponse(new byte[] {1, 2, 3});

      channel.close().sync();

      assertThrows(ExecutionException.class, () -> response.get(1, TimeUnit.SECONDS));
   }

   @Test
   void disconnectUnblocksInitialMetadataWait() throws Exception
   {
      OpenpiNettyWebSocketHandler handler = new OpenpiNettyWebSocketHandler(new URI("ws://localhost:8000"));
      EmbeddedChannel channel = new EmbeddedChannel(handler);

      channel.close().sync();

      assertThrows(RuntimeException.class, () -> handler.awaitFirstMessage(1, TimeUnit.SECONDS));
   }
}
