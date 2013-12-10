package us.ihmc.darpaRoboticsChallenge.processManagement;

import com.martiansoftware.jsap.JSAPException;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.fixedPointRepresentation.UnsignedByteTools;
import us.ihmc.utilities.net.tcpServer.DisconnectedException;
import us.ihmc.utilities.net.tcpServer.ReconnectingTCPServer;
import us.ihmc.utilities.processManagement.ShellOutProcessSpawner;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.net.ServerSocket;
import java.net.Socket;

public class DRCNetworkProcessorEnterpriseCloudDispatcherBackend implements Runnable
{
   private ShellOutProcessSpawner networkProcessorSpawner = new ShellOutProcessSpawner(true);
   private ReconnectingTCPServer commandServer;

   private final byte[] buffer;

   public DRCNetworkProcessorEnterpriseCloudDispatcherBackend()
   {
      try
      {
         commandServer = new ReconnectingTCPServer(DRCConfigParameters.NETWORK_PROCESSOR_CLOUD_DISPATCHER_BACKEND_TCP_PORT);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      buffer = commandServer.getBuffer();
   }

   public void run()
   {
      while (true)
      {
         try
         {
            commandServer.read(1);

            switch (UnsignedByteTools.toInt(buffer[0]))
            {
               case 0x00 :
                  spawnNetworkProcessor();

                  break;

               case 0x10 :
                  killNetworkProcessor();

                  break;

               case 0x11 :
                  restartNetworkProcessor();

                  break;

               case 0x22 :
                  startStreamingOutput();

                  break;

               default :
                  System.err.println("Invalid request: " + Integer.toHexString(UnsignedByteTools.toInt(buffer[0])));

                  break;
            }
         }
         catch (DisconnectedException e)
         {
            commandServer.reset();
         }

         if (networkProcessorSpawner.hasRunningProcesses())
         {
         }
         else
         {
         }

         commandServer.reset();
      }
   }

   private void startStreamingOutput()
   {
      new Thread(new Runnable()
      {
         @Override
         public void run()
         {
            ServerSocket server = null;
            try
            {
               server = new ServerSocket(DRCConfigParameters.CONTROLLER_CLOUD_DISPATCHER_BACKEND_TCP_PORT + 5);
               commandServer.write(new byte[] {UnsignedByteTools.fromInt(0x22)});
               Socket socket = server.accept();
               socket.setTcpNoDelay(true);
               OutputStream outputStream = socket.getOutputStream();
               System.setOut(new PrintStream(outputStream));
               System.setErr(new PrintStream(outputStream));
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
            catch (DisconnectedException e)
            {
               commandServer.reset();

               try
               {
                  server.close();
               }
               catch (IOException e1)
               {
                  e1.printStackTrace();
               }
            }
         }
      }).start();
   }

   private void spawnNetworkProcessor()
   {
      if (!networkProcessorSpawner.hasRunningProcesses())
      {
         networkProcessorSpawner.setInheritEnvironment(true);
         networkProcessorSpawner.spawn("/home/unknownid/atlas/runNetworkProcessor.sh");

         try
         {
            commandServer.write(new byte[] {UnsignedByteTools.fromInt(0x00)});
         }
         catch (DisconnectedException e)
         {
            commandServer.reset();
         }
      }
      else
         System.err.println("Network processor is already running. Try restarting.");
   }

   private void killNetworkProcessor()
   {
      networkProcessorSpawner.killAll();

      try
      {
         commandServer.write(new byte[] {UnsignedByteTools.fromInt(0x11)});
      }
      catch (DisconnectedException e)
      {
         commandServer.reset();
      }
   }

   private void restartNetworkProcessor()
   {
      killNetworkProcessor();
      ThreadTools.sleep(5000);
      spawnNetworkProcessor();
   }

   public static void main(String[] args) throws JSAPException
   {
      new Thread(new DRCNetworkProcessorEnterpriseCloudDispatcherBackend()).start();
   }
}
