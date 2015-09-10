package us.ihmc.simulationconstructionset.whiteBoard;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.utilities.io.DatagramInputStream;
import us.ihmc.utilities.io.DatagramOutputStream;

public class UDPYoWhiteBoard extends DataStreamYoWhiteBoard
{
   private static final boolean VERBOSE = false;

   private final String ipAddress;
   private final int sendPort, receivePort;
   private final boolean throwOutStalePackets;
   private DatagramInputStream datagramInputStream;
      
   public UDPYoWhiteBoard(String name, boolean runThisOneFirst, String ipAddress, int port, boolean throwOutStalePackets)
   {
      this(name, runThisOneFirst, ipAddress, port, port, throwOutStalePackets);
   }

   public UDPYoWhiteBoard(String name, boolean runThisOneFirst, String ipAddress, int sendPort, int receivePort, boolean throwOutStalePackets)
   {
      this(name, runThisOneFirst, ipAddress, sendPort, receivePort, throwOutStalePackets, false, null);
   }
   
   public UDPYoWhiteBoard(String name, boolean runThisOneFirst, String ipAddress, int sendPort, int receivePort, boolean throwOutStalePackets, boolean createYoVariablesOnConnect, YoVariableRegistry rootRegistry)
   {
      super(name, !runThisOneFirst, runThisOneFirst, createYoVariablesOnConnect, rootRegistry);

      this.ipAddress = ipAddress;
      this.sendPort = sendPort;
      this.receivePort = receivePort;
      this.throwOutStalePackets = throwOutStalePackets;
   }

   public void run()
   {
      try
      {
         datagramInputStream = new DatagramInputStream(receivePort);
         DatagramOutputStream outputStream = new DatagramOutputStream(sendPort, ipAddress);

         DataInputStream dataInputStream = new DataInputStream(datagramInputStream);
         DataOutputStream dataOutputStream = new DataOutputStream(outputStream);

         super.setDataStreams(dataInputStream, dataOutputStream);

         if (VERBOSE)
         {
            System.out.println("UDPYoWhiteBoard created and set input streams.");
         }

         super.run();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   protected void allowThrowOutStalePacketsIfYouWish()
   {
      if (throwOutStalePackets)
      {
         datagramInputStream.setThrowOutStalePackets(throwOutStalePackets);
      }
      
   }
}
