package us.ihmc.quadrupedRobotics.params;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.tools.thread.ThreadTools;

public class ParameterSaver
{
   private final Path path;
   private final PacketCommunicator packetCommunicator;

   private boolean writeFinished = false;

   public ParameterSaver(Path path, NetClassList netClassList) throws IOException
   {
      this.path = path;
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
      File file = path.toFile();
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

      System.out.println("Parameters written to " + path);
      writeFinished = true;
   }

   private void waitForWriteToFinish()
   {
      while (!writeFinished)
      {
         ThreadTools.sleep(10);
      }
   }

   public static void run(String[] args, Path defaultParametersPath, NetClassList netClassList) throws IOException
   {
      if (args.length != 1)
      {
         System.err.println("usage: java ParameterSaver [out_file_name]");
         System.exit(1);
      }

      Path path = Paths.get(args[0]);

      // If an absolute path is specified, use it as-is. If not, try to locate the robot-specific resources directory.
      if (!path.isAbsolute())
      {
         Path pathFromCwd = Paths.get("").toAbsolutePath().getParent().resolve(defaultParametersPath);

         // If $IHMC_WORKSPACE is defined, use it.
         if (System.getenv("IHMC_WORKSPACE") != null)
         {
            Path workspacePath = Paths.get(System.getenv("IHMC_WORKSPACE"));
            path = workspacePath.resolve(defaultParametersPath).resolve(path);
         }
         // If ../<defaultParametersPath> exists, use that.
         else if (Files.exists(pathFromCwd))
         {
            path = pathFromCwd.resolve(path);
         }
         else
         {
            System.err.println("If a relative path is supplied then either $IHMC_WORKSPACE must be defined or cwd must be your _IHMCWorkspace");
            System.exit(1);
         }
      }

      System.out.println("Saving parameters to: " + path);

      ParameterSaver saver = new ParameterSaver(path, netClassList);
      saver.requestParameters();
      saver.waitForWriteToFinish();

      System.exit(0);
   }
}
