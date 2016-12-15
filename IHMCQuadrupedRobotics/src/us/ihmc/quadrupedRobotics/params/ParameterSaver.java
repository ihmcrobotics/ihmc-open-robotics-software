package us.ihmc.quadrupedRobotics.params;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.concurrent.*;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.robotics.dataStructures.parameter.Parameter;

public class ParameterSaver
{
   private static final long REQUEST_TIMEOUT_MS = 5000; // 5s

   private final Path path;
   private final String host;
   private final NetworkPorts port;
   private final PacketCommunicator packetCommunicator;
   private CountDownLatch requestResponseLatch;

   public ParameterSaver(Path path, String host, NetworkPorts port, NetClassList netClassList)
   {
      this.path = path;
      this.host = host;
      this.port = port;

      this.packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(host, port, netClassList);
      packetCommunicator.attachListener(ParameterListPacket.class, new PacketConsumer<ParameterListPacket>()
      {
         @Override
         public void receivedPacket(ParameterListPacket packet)
         {
            System.out.println("Received parameter list of size: " + packet.getParameters().size());

            writeParameters(packet.getParameters());
         }
      });
   }

   private void connect() throws IOException, InterruptedException
   {
      System.out.println("Connecting to robot @ " + host + ":" + port);
      packetCommunicator.connect();
      while (!packetCommunicator.isConnected())
      {
         Thread.sleep(10);
      }
   }

   private void requestParameters()
   {
      System.out.println("Requesting parameter list");

      // Latch on request until parameter list response is written
      requestResponseLatch = new CountDownLatch(1);
      packetCommunicator.send(new RequestParameterListPacket());
   }

   private void writeParameters(List<Parameter> parameters)
   {
      System.out.println("Writing parameters file");

      File file = path.toFile();
      try (PrintWriter writer = new PrintWriter(new FileOutputStream(file)))
      {
         for (Parameter parameter : parameters)
         {
            writer.print(parameter.dump() + "\n");
         }
      }
      catch (IOException e)
      {
         System.err.println("Failed to write parameter list: " + e.getMessage());
         e.printStackTrace();
      }

      // Write is finished -- release latch
      requestResponseLatch.countDown();
   }

   private void waitForWriteToFinish() throws InterruptedException
   {
      // Wait for latch to be released at the end of the write
      ExecutorService timeoutExecutor = Executors.newSingleThreadExecutor();
      Future<Void> future = timeoutExecutor.submit(new Callable<Void>()
      {
         @Override
         public Void call() throws Exception
         {
            requestResponseLatch.await();
            return null;
         }
      });

      try
      {
         future.get(REQUEST_TIMEOUT_MS, TimeUnit.MILLISECONDS);
         future.cancel(true);
      }
      catch (TimeoutException e)
      {
         System.err.println("Request timed out");
      }
      catch (ExecutionException e)
      {
         System.err.println("Request failed: " + e.getMessage());
      }
   }

   public static void run(String[] args, Path defaultParametersPath, NetworkPorts port, NetClassList netClassList) throws IOException, InterruptedException
   {
      if (args.length != 2)
      {
         System.err.println("usage: java ParameterSaver [host] [out_file_name]");
         System.exit(1);
      }

      String host = args[0];
      Path path = Paths.get(args[1]);

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

      ParameterSaver saver = new ParameterSaver(path, host, port, netClassList);
      saver.connect();
      saver.requestParameters();
      saver.waitForWriteToFinish();

      System.exit(0);
   }
}
