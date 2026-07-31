package us.ihmc.gr00t;

import us.ihmc.openpi.OpenpiClient;

/** Client for a GR00T server using the configurable OpenPI websocket/msgpack protocol. */
public class Gr00tClient extends OpenpiClient
{
   public Gr00tClient(String host, int port, int stateSize, int chunkLength, int imageWidth, int imageHeight)
   {
      super(host, port, stateSize, chunkLength, imageWidth, imageHeight, "");
   }
}
