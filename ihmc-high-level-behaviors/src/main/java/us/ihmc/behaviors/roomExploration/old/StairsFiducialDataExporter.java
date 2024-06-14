package us.ihmc.behaviors.roomExploration.old;

import perception_msgs.msg.dds.DetectedFiducialPacket;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.ros2.ROS2Node;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.util.concurrent.atomic.AtomicReference;

public class StairsFiducialDataExporter
{
   public StairsFiducialDataExporter() throws IOException
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, getClass().getSimpleName());

      AtomicReference<DetectedFiducialPacket> latestDetectedFiducial = new AtomicReference<>();
      ros2Node.createSubscription(FiducialDetectorToolboxModule.getOutputTopic("Atlas").withTypeName(DetectedFiducialPacket.class),
                                  s -> latestDetectedFiducial.set(s.takeNextData()));

      AtomicReference<PlanarRegionsListMessage> latestRegions = new AtomicReference<>();
      ros2Node.createSubscription(REACommunicationProperties.outputTopic.withTypeName(PlanarRegionsListMessage.class),
                                  s -> latestRegions.set(s.takeNextData()));

      long exportFrequency = 7000;

      JSONSerializer<PlanarRegionsListMessage> regionSerializer = new JSONSerializer<>(new PlanarRegionsListMessagePubSubType());
      String regionsFile = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator;
      int regionIndex = 0;

      ThreadTools.sleep(10000);

      while (true)
      {
         ThreadTools.sleep(exportFrequency);

         if (latestRegions.get() != null)
         {
            PlanarRegionsListMessage regionsMessage = latestRegions.get();
            byte[] serializedRegions = regionSerializer.serializeToBytes(regionsMessage);
            String regionsFileName = regionsFile + "regions" + regionIndex++;

            LogTools.info("exporting regions " + regionsFileName);

            FileTools.ensureFileExists(new File(regionsFileName).toPath());
            FileOutputStream outputStream = new FileOutputStream(regionsFileName);
            PrintStream printStream = new PrintStream(outputStream);

            printStream.write(serializedRegions);
            printStream.flush();
            outputStream.close();
            printStream.close();
         }

         if (latestDetectedFiducial.get() != null)
         {
            DetectedFiducialPacket packet = latestDetectedFiducial.get();
            Pose3D pose = packet.getFiducialTransformToWorld();

            System.out.println("Fiducial pose:");
            System.out.println(pose);
            System.out.println("------");
         }
      }
   }

   public static void main(String[] args) throws IOException
   {
      new StairsFiducialDataExporter();
   }
}
