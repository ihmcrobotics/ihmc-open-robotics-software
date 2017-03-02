package us.ihmc.ihmcPerception.objectDetector;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.BoundingBoxesPacket;
import us.ihmc.communication.packets.HeatMapPacket;
import us.ihmc.communication.packets.ObjectDetectorResultPacket;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.tools.thread.ThreadTools;

import java.awt.*;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.stream.IntStream;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class ValveDetectorProcess implements PacketConsumer<VideoPacket>
{
   private final PacketCommunicator valveDetectionServer = PacketCommunicator
         .createTCPPacketCommunicatorServer(NetworkPorts.VALVE_DETECTOR_SERVER_PORT, new IHMCCommunicationKryoNetClassList());
   private final PacketCommunicator detectionStatusClient = PacketCommunicator
         .createTCPPacketCommunicatorClient(NetworkParameters.getHost(NetworkParameterKeys.networkManager), NetworkPorts.VALVE_DETECTOR_FEEDBACK_PORT, new IHMCCommunicationKryoNetClassList());
   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private final ValveDetector valveDetector;

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory("ValveDetector"));

   public ValveDetectorProcess() throws Exception
   {
      this.valveDetector = new ValveDetector();

      valveDetectionServer.attachListener(VideoPacket.class, this);
      valveDetectionServer.connect();

      detectionStatusClient.connect();
   }

   @Override
   public void receivedPacket(VideoPacket packet)
   {
      executorService.submit(() ->
                             {
                                Pair<List<Rectangle>, ValveDetector.HeatMap> rectanglesAndHeatMaps = valveDetector
                                      .detect(jpegDecompressor.decompressJPEGDataToBufferedImage(packet.getData()));

                                rectanglesAndHeatMaps.getLeft().sort((r1, r2) -> -Integer.compare(r1.width * r1.height, r2.width * r2.height));

                                HeatMapPacket heatMapPacket = new HeatMapPacket();
                                heatMapPacket.width = rectanglesAndHeatMaps.getRight().w;
                                heatMapPacket.height = rectanglesAndHeatMaps.getRight().h;
                                heatMapPacket.data = rectanglesAndHeatMaps.getRight().data;
                                heatMapPacket.name = "Valve";

                                int[] packedBoxes = rectanglesAndHeatMaps.getLeft().stream()
                                                                         .flatMapToInt(rect -> IntStream.of(rect.x, rect.y, rect.width, rect.height)).toArray();
                                String[] names = new String[rectanglesAndHeatMaps.getLeft().size()];
                                for (int i = 0; i < names.length; i++)
                                {
                                   names[i] = "Valve " + i;
                                }
                                BoundingBoxesPacket boundingBoxesPacket = new BoundingBoxesPacket(packedBoxes, names);

                                ObjectDetectorResultPacket resultPacket = new ObjectDetectorResultPacket(heatMapPacket, boundingBoxesPacket);
                                resultPacket.setDestination(PacketDestination.BEHAVIOR_MODULE);

                                detectionStatusClient.send(resultPacket);
                             });
   }

   public static void main(String[] args) throws Exception
   {
      new ValveDetectorProcess();
   }
}
