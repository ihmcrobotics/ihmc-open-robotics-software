package us.ihmc.communication.remote;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.ParameterListPacket;
import us.ihmc.communication.packets.RequestParameterListPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.robotics.dataStructures.parameter.Parameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterSaver;

public class RemoteParameterSaver
{
   private static final long REQUEST_TIMEOUT_MS = 5000; // 5s

   private final String host;
   private final NetworkPorts port;
   private final PacketCommunicator packetCommunicator;

   private CountDownLatch requestResponseLatch;
   private ParameterSaver parameterSaver;

   public RemoteParameterSaver(Path path, String host, NetworkPorts port, NetClassList netClassList)
   {
      this.host = host;
      this.port = port;

      parameterSaver = new ParameterSaver(path);
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

      requestResponseLatch = new CountDownLatch(1);
      packetCommunicator.send(new RequestParameterListPacket());
   }

   private void writeParameters(List<Parameter> parameters)
   {
      parameterSaver.writeParameters(parameters);
      // Write is finished -- release latch
      requestResponseLatch.countDown();
   }

   public void waitForWriteToFinish() throws InterruptedException
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
         System.err.println("usage: java RemoteParameterSaver [host] [out_file_name]");
         System.exit(1);
      }

      String host = args[0];
      String filename = args[1];

      Path path = defaultParametersPath.resolve(filename);
      System.out.println("Saving parameters to: " + path);

      RemoteParameterSaver saver = new RemoteParameterSaver(path, host, port, netClassList);
      saver.connect();
      saver.requestParameters();
      saver.waitForWriteToFinish();

      System.exit(0);
   }
}
