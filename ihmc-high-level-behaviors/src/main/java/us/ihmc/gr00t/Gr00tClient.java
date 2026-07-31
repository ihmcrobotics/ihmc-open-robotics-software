package us.ihmc.gr00t;

import us.ihmc.openpi.OpenpiClient;
import us.ihmc.robotics.robotSide.SideDependentList;

/** Client for a GR00T server using the configurable OpenPI websocket/msgpack protocol. */
public class Gr00tClient extends OpenpiClient
{
   public Gr00tClient(String host, int port, int stateSize, int chunkLength, int imageWidth, int imageHeight)
   {
      super(host, port, stateSize, chunkLength, imageWidth, imageHeight, "");
   }

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
                      String actionKey)
   {
      super(host, port, stateSize, actionSize, chunkLength, imageWidth, imageHeight, "", imageKeys, stateKey, promptKey, actionKey);
   }
}
