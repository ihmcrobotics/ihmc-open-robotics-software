package us.ihmc.communication.remote;

import java.io.Serializable;

public class TCPAddress implements Serializable
{
   /**
    *
    */
   private static final long serialVersionUID = -7494269102464437835L;
   public String host;
   public int port;

   public TCPAddress(String host, int port)
   {
      this.host = host;
      this.port = port;
   }
}
