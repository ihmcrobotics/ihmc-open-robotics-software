package us.ihmc.robotDataLogger.websocket.client.discovery;

public class HTTPDataServerDescription
{
   private final String host;
   private final int port;
   private final boolean persistant;

   public HTTPDataServerDescription(String host, int port, boolean persistant)
   {
      this.host = host;
      this.port = port;
      this.persistant = persistant;
   }

   public String getHost()
   {
      return host;
   }

   public int getPort()
   {
      return port;
   }

   public boolean isPersistant()
   {
      return persistant;
   }
   
   @Override
   public String toString()
   {
      return host + ":" + port;
   }

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((host == null) ? 0 : host.hashCode());
      result = prime * result + port;
      return result;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      HTTPDataServerDescription other = (HTTPDataServerDescription) obj;
      if (host == null)
      {
         if (other.host != null)
            return false;
      }
      else if (!host.equals(other.host))
         return false;
      if (port != other.port)
         return false;
      return true;
   }

}
