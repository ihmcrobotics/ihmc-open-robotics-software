package us.ihmc.communication.remote;



public class DebugReflectiveTCPServer
{
   private ReflectiveTCPServer server;

   public void voidMethod()
   {
      @SuppressWarnings("unused")
      int i = 1 + 1;
   }

   public String stringMethod()
   {
      return "1+1";
   }

   public String getPose(String bodyName)
   {
      @SuppressWarnings("unused")
      int i = 1 + 1;

      return "1+1=" + bodyName;
   }


   public DebugReflectiveTCPServer()
   {
      // start tcp server
      server = new ReflectiveTCPServer(this);
      server.init(7777);
      System.out.println("server started on " + server.getHost() + ": " + server.getPort());
   }

   public static void main(String[] args)
   {
      new DebugReflectiveTCPServer();

      // wait around until terminated
      synchronized (Thread.currentThread())
      {
         try
         {
            Thread.currentThread().wait();
         }
         catch (InterruptedException ex)
         {
            ex.printStackTrace();
         }
      }

   }
}
