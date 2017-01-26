package us.ihmc.robotDataLogger.gui;

import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.nio.channels.ClosedByInterruptException;

import us.ihmc.multicastLogDataProtocol.LogDataProtocolSettings;
import us.ihmc.multicastLogDataProtocol.LogPacketHandler;
import us.ihmc.multicastLogDataProtocol.StreamingDataTCPClient;
import us.ihmc.tools.thread.ThreadTools;

public class GUICaptureReceiver extends Thread
{
   private final LogPacketHandler handler;
   private final NetworkInterface iface;
   private final InetAddress announceGroup;
   
   private volatile boolean running = true;
   
   public GUICaptureReceiver(NetworkInterface iface, InetAddress announceGroup, LogPacketHandler handler)
   {
      super("GUICaptureReceiverThread");

      this.handler = handler;
      this.announceGroup = announceGroup;
      this.iface = iface;

   }

   public void run()
   {
      while(running)
      {

         InetAddress address;
         try
         {
            System.out.println("Listening for new streams on " + iface);
            address = GUICaptureBroadcast.getIP(iface, announceGroup);
            System.out.println("Found stream. Connecting to " + address);
            StreamingDataTCPClient client = new StreamingDataTCPClient(address, LogDataProtocolSettings.UI_DATA_PORT, handler, 1);
            client.run(); // Run the runnable, making it blocking
         }
         catch (ClosedByInterruptException e)
         {
            System.out.println("Got interrupted. Closing GUICaptureReceiver");
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         
         // Check if running, could be interrupted and cleared by here. If interrupted in the sleep, it will continue immediately.
         if(running)
         {
            ThreadTools.sleep(5000); // Do not peg CPU when things fail
         }
      }
   }
   
   public void close()
   {
      running = false;
      interrupt();
      try
      {
         join();
      }
      catch (InterruptedException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }
}