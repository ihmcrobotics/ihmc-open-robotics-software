package us.ihmc.simulationConstructionSetTools.whiteBoard;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.tools.io.DatagramInputStream;
import us.ihmc.tools.io.DatagramOutputStream;
import us.ihmc.commons.thread.ThreadTools;

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

         super.setupAndRunHandlingThread();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   public void startUDPThread()
   {
      ThreadTools.startAThread(new Runnable()
      {
         @Override
         public void run()
         {
            UDPYoWhiteBoard.this.run();
         }
      }, getName() + "UDPThread");
   }

   @Override
   protected void allowThrowOutStalePacketsIfYouWish()
   {
      if (throwOutStalePackets)
      {
         datagramInputStream.setThrowOutStalePackets(throwOutStalePackets);
      }
   }
}
