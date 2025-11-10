package us.ihmc.robotDataVisualizer.logger.localLogging;

import us.ihmc.log.LogTools;

import java.net.NetworkInterface;
import java.net.SocketException;

// TODO move into the logger
public final class LocalLoggingTools
{
   private static final String DEBUG_NETWORK_INTERFACE_NAME = "ethernet0";
   public static final boolean LOGGING_LOCALLY;

   static
   {
      /*
       * We log locally to the robot if the debug network interface is NOT connected
       * and the /root/.ihmc/logs directory is a mountpoint (a flash drive is plugged in).
       */
      boolean debugNetworkInterfaceIsConnected = debugNetworkInterfaceIsConnected();
      boolean logDirectoryIsMountpoint = logDirectoryIsMountpoint();
      if (!debugNetworkInterfaceIsConnected && !logDirectoryIsMountpoint)
      {
         LogTools.warn("[Local logging] The debug network interface is not connected so we tried logging locally to a mountpoint at /root/.ihmc/logs.");
         LogTools.warn("[Local logging] However, there was no mountpoint at /root/.ihmc/logs. Is there a flash drive plugged in and mounted?");
      }

//      LOGGING_LOCALLY = logDirectoryIsMountpoint && !debugNetworkInterfaceIsConnected;
      LOGGING_LOCALLY = !debugNetworkInterfaceIsConnected;
   }

   private static boolean debugNetworkInterfaceIsConnected()
   {
      try
      {
         NetworkInterface iface = NetworkInterface.getByName(DEBUG_NETWORK_INTERFACE_NAME);
         return iface != null && iface.isUp() && !iface.isLoopback() && !iface.isVirtual();
      }
      catch (SocketException e)
      {
         return false;
      }
   }

   private static boolean logDirectoryIsMountpoint()
   {
      try
      {
         Process process = Runtime.getRuntime().exec("mountpoint /root/.ihmc/logs");
         return process.waitFor() == 0;
      }
      catch (Exception e)
      {
         return false;
      }
   }
}
