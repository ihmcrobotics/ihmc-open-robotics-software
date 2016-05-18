package us.ihmc.quadrupedRobotics.params;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.tools.thread.ThreadTools;

public class ParameterSaver
{
   private final String fileName;
   private final PacketCommunicator packetCommunicator;

   private boolean writeFinished = false;

   public ParameterSaver(String fileName, NetClassList netClassList) throws IOException
   {
      this.fileName = fileName;
      this.packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient("localhost", NetworkPorts.CONTROLLER_PORT, netClassList);
      this.packetCommunicator.attachListener(ParameterListPacket.class, new PacketConsumer<ParameterListPacket>()
      {
         @Override
         public void receivedPacket(ParameterListPacket packet)
         {
            writeParameters(packet.getParameters());
         }
      });
      this.packetCommunicator.connect();

      while (!packetCommunicator.isConnected())
      {
         ThreadTools.sleep(10);
      }
   }

   private void requestParameters()
   {
      packetCommunicator.send(new RequestParameterListPacket());

      System.out.println("Requesting parameter list...");
   }

   private void writeParameters(List<Parameter> parameters)
   {
      System.out.println("Got parameter list of length " + parameters.size());
      File file = new File(fileName);
      try (PrintWriter writer = new PrintWriter(new FileOutputStream(file)))
      {
         for (Parameter parameter : parameters)
         {
            writer.println(parameter.dump());
         }
      }
      catch (IOException e)
      {
         System.err.println("Failed to write parameter list: " + e.getMessage());
         e.printStackTrace();
      }

      System.out.println("Parameters written to " + fileName);
      writeFinished = true;
   }

   private void waitForWriteToFinish()
   {
      while (!writeFinished)
      {
         ThreadTools.sleep(10);
      }
   }

   public static void run(String[] args, NetClassList netClassList) throws IOException
   {
      if (args.length != 1)
      {
         System.err.println("usage: java ParameterSaver [out_file_name]");
         System.exit(1);
      }

      String fileName = args[0];

      ParameterSaver saver = new ParameterSaver(fileName, netClassList);
      saver.requestParameters();
      saver.waitForWriteToFinish();

      System.exit(0);
   }
}
