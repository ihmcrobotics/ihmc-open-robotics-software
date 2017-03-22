package us.ihmc.atlas.sensors;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.esotericsoftware.kryo.Kryo;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.time.DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.ihmcPerception.depthData.PointCloudDataReceiver;
import us.ihmc.ihmcPerception.depthData.PointCloudSource;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;

public class PointCloudDataReceiverSimulation implements Runnable, PacketConsumer<PointCloudWorldPacket>
{
   private final ScheduledExecutorService executor = Executors.newScheduledThreadPool(1);
   private final PointCloudDataReceiver pointCloudDataReceiver;
   private final ReferenceFrame lidarFrame;
   private long timestamp = 0;
   private final Random random = new Random(433456896807L);
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;

   private final RobotConfigurationData robotConfigurationData;


   private final Kryo kryo = new Kryo();
   public PointCloudDataReceiverSimulation() throws IOException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.REAL_ROBOT, true);
      robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      robotConfigurationData = new RobotConfigurationData(FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel), fullRobotModel.getForceSensorDefinitions(), null, fullRobotModel.getIMUDefinitions());
      PPSTimestampOffsetProvider ppsTimestampOffsetProvider = new DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider();

      PacketCommunicator sensorSuitePacketCommunicatorServer = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER,
            new IHMCCommunicationKryoNetClassList());
      sensorSuitePacketCommunicatorServer.connect();
      PacketCommunicator sensorSuitePacketCommunicatorClient = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER,
            new IHMCCommunicationKryoNetClassList());
      sensorSuitePacketCommunicatorClient.connect();

      sensorSuitePacketCommunicatorClient.attachListener(PointCloudWorldPacket.class, this);

      pointCloudDataReceiver = new PointCloudDataReceiver(robotModel, robotModel.getCollisionBoxProvider(), ppsTimestampOffsetProvider,
            robotModel.getContactPointParameters(), robotConfigurationDataBuffer, sensorSuitePacketCommunicatorServer);
      pointCloudDataReceiver.start();
      pointCloudDataReceiver.setLidarState(LidarState.ENABLE);
      lidarFrame = fullRobotModel.getLidarBaseFrame(robotModel.getSensorInformation().getLidarParameters()[0].getSensorNameInSdf());

      executor.scheduleAtFixedRate(this, 0, 25000000, TimeUnit.NANOSECONDS);
   }

   @Override
   public void run()
   {
      try
      {
         timestamp += 25000000;

         RobotConfigurationData clone = kryo.copy(robotConfigurationData);
         clone.setTimestamp(timestamp);
         robotConfigurationDataBuffer.receivedPacket(clone);

         ArrayList<Point3D> points = new ArrayList<>(1000);
         long[] timestamps = new long[1000];
         Arrays.fill(timestamps, timestamp);

         for (int i = 0; i < 1000; i++)
         {
            points.add(new Point3D(10.0 * random.nextDouble(), -10.0 + 10.0 * random.nextDouble(), -1.0 + 2.0 * random.nextDouble()));
         }

         pointCloudDataReceiver.receivedPointCloudData(ReferenceFrame.getWorldFrame(), lidarFrame, timestamps, points, PointCloudSource.QUADTREE,
               PointCloudSource.NEARSCAN);

      }
      catch(Throwable t)
      {
         t.printStackTrace();
      }
   }

   public static void main(String[] args) throws IOException
   {
      new PointCloudDataReceiverSimulation();
   }

   @Override
   public void receivedPacket(PointCloudWorldPacket packet)
   {
      System.out.println("Received packet");
      System.out.println("Quad tree size: " + packet.getGroundQuadTreeSupport().length);
      System.out.println("Decaying world scan size: " + packet.getDecayingWorldScan().length);
   }
}
