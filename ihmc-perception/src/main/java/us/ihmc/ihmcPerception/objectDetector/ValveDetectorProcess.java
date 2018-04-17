package us.ihmc.ihmcPerception.objectDetector;

import java.awt.Rectangle;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.stream.IntStream;

import org.apache.commons.lang3.tuple.Pair;

import controller_msgs.msg.dds.BoundingBoxesPacket;
import controller_msgs.msg.dds.HeatMapPacket;
import controller_msgs.msg.dds.ObjectDetectorResultPacket;
import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;

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
                                      .detect(jpegDecompressor.decompressJPEGDataToBufferedImage(packet.getData().toArray()));

                                rectanglesAndHeatMaps.getLeft().sort((r1, r2) -> -Integer.compare(r1.width * r1.height, r2.width * r2.height));

                                HeatMapPacket heatMapPacket = new HeatMapPacket();
                                heatMapPacket.setWidth(rectanglesAndHeatMaps.getRight().w);
                                heatMapPacket.setHeight(rectanglesAndHeatMaps.getRight().h);
                                heatMapPacket.getData().add(rectanglesAndHeatMaps.getRight().data);
                                heatMapPacket.setName("Valve");

                                int[] packedBoxes = rectanglesAndHeatMaps.getLeft().stream()
                                                                         .flatMapToInt(rect -> IntStream.of(rect.x, rect.y, rect.width, rect.height)).toArray();
                                String[] names = new String[rectanglesAndHeatMaps.getLeft().size()];
                                for (int i = 0; i < names.length; i++)
                                {
                                   names[i] = "Valve " + i;
                                }
                                BoundingBoxesPacket boundingBoxesPacket = MessageTools.createBoundingBoxesPacket(packedBoxes, names);

                                ObjectDetectorResultPacket resultPacket = MessageTools.createObjectDetectorResultPacket(heatMapPacket, boundingBoxesPacket);
                                resultPacket.setDestination(PacketDestination.BEHAVIOR_MODULE.ordinal());

                                detectionStatusClient.send(resultPacket);
                             });
   }

   public static void main(String[] args) throws Exception
   {
      new ValveDetectorProcess();
   }
}