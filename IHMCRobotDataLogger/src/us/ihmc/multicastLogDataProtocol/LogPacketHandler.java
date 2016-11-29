package us.ihmc.multicastLogDataProtocol;

import java.net.InetSocketAddress;
import java.nio.ByteBuffer;

import us.ihmc.robotDataLogger.LogDataHeader;

public interface LogPacketHandler
{
   /**
    * Gets called the first time a new timestamp is seen, even if the whole packet hasn't been received yet.
    * @param timestamp
    */
   public void timestampReceived(long timestamp);
   
   
   /**
    * Gets called when a new packet is available.
    * 
    * @param buffer
    */
   public void newDataAvailable(LogDataHeader header, ByteBuffer buffer);
   
   public void timeout();


   public void connected(InetSocketAddress localAddress);


   /**
    * Gets called when a keep alive packet got received. 
    * 
    * This method can be a no-op as the client handles the keep alives
    */
   public void keepAlive();
}
