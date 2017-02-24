package us.ihmc.atlas.operatorInterfaceDebugging;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandSensorData;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.robotiq.data.RobotiqHandSensorData;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

import java.io.IOException;
import java.util.Arrays;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class AtlasNoSimPacketBlaster implements Runnable
{
   private static final long PACKET_SEND_PERIOD_MILLIS = 1;
   private PacketCommunicator packetCommunicator;
   private Random random = new Random();
   private AtlasRobotModel atlasRobotModel;
   private boolean includeFingerJoints;
   private OneDoFJoint[] jointList;
   private FullHumanoidRobotModel fullRobotModel;
   private int numberOfJoints;
   private double[] jointLowerLimits;
   private double[] jointUpperLimits;
   private IMUDefinition[] imuDefinitions;
   private ForceSensorDefinition[] forceSensorDefinitions;
   private int momentFixedPointMax;
   private int forceFixedPointMax;

   public AtlasNoSimPacketBlaster() throws IOException
   {
      atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false);
      fullRobotModel = atlasRobotModel.createFullRobotModel();
      
      initRobotConfiguration();
      
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.NETWORK_PROCESSOR_TO_UI_TCP_PORT, new IHMCCommunicationKryoNetClassList());
      packetCommunicator.attachStateListener(new NetStateListener()
      {
         @Override
         public void disconnected()
         {
            PrintTools.info("Disconnected");
         }
         
         @Override
         public void connected()
         {
            PrintTools.info("Connected");
         }
      });
      packetCommunicator.connect();
      ScheduledExecutorService threadExecutor = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      threadExecutor.scheduleAtFixedRate(this, 0, PACKET_SEND_PERIOD_MILLIS, TimeUnit.MILLISECONDS);
   }
   
   public void initRobotConfiguration()
   {
      if (includeFingerJoints)
         jointList = fullRobotModel.getOneDoFJoints();
      else
         jointList = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);

      numberOfJoints = jointList.length;
      jointLowerLimits = new double[numberOfJoints];
      jointUpperLimits = new double[numberOfJoints];
      imuDefinitions = fullRobotModel.getIMUDefinitions();

      forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();

      for (int i = 0; i < numberOfJoints; i++)
      {
         jointLowerLimits[i] = Math.max(-Math.PI, jointList[i].getJointLimitLower());
         jointUpperLimits[i] = Math.min(Math.PI, jointList[i].getJointLimitUpper());
      }
   }
   
   int debug = 0;
   int skip = 0;
   
   @Override
   public void run()
   {
      OneDoFJoint[] joints = Arrays.copyOf(jointList, jointList.length);

      RobotConfigurationData robotConfigurationData = new RobotConfigurationData(joints, forceSensorDefinitions, null, imuDefinitions);

      robotConfigurationData.setTimestamp(random.nextInt(1800) * Conversions.milliSecondsToNanoSeconds(100));

      robotConfigurationData.jointAngles = new float[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
      {
         float min = (float) jointLowerLimits[i];
         float max = (float) jointUpperLimits[i];
         robotConfigurationData.jointAngles[i] = min + random.nextFloat() * (max - min);
      }

//      robotConfigurationData.setRootTranslation(RandomTools.generateRandomVector(random, random.nextDouble() * 1000.0));
      robotConfigurationData.setRootTranslation(new Vector3D(random.nextDouble(), random.nextDouble(), 1.0 * random.nextDouble()));
//      robotConfigurationData.setRootTranslation(new Vector3d(0.0, 0.0, 1.0));
      robotConfigurationData.setRootOrientation(RandomTools.generateRandomQuaternion(random));

      for (int sensorNumber = 0; sensorNumber < forceSensorDefinitions.length; sensorNumber++)
      {
//         robotConfigurationData.momentAndForceDataAllForceSensors[sensorNumber].set(new DenseMatrix64F(Wrench.SIZE, 1));
         for (int i = 0; i < 3; i++)
         {
            robotConfigurationData.momentAndForceDataAllForceSensors[sensorNumber][i] = (float) (-momentFixedPointMax + 2.0 * random.nextDouble() * momentFixedPointMax);
         }
         for (int i = 3; i < 6; i++)
         {
            robotConfigurationData.momentAndForceDataAllForceSensors[sensorNumber][i] = (float) (-forceFixedPointMax + 2.0 * random.nextDouble() * forceFixedPointMax);
         }
      }
      
      PointCloudWorldPacket pointCloudWorldPacket = new PointCloudWorldPacket();
      pointCloudWorldPacket.setTimestamp(1);
//      pointCloudWorldPacket.setDecayingWorldScan(new Point3D[100]);
//      pointCloudWorldPacket.setGroundQuadTreeSupport(new Point3D[100]);
      
      if (skip++ > 20)
      {
         System.out.print(".");
         debug++;
         skip = 0;
      }
      
      if (debug > 500)
      {
         debug = 0;
         System.out.println();
         ThreadTools.sleepSeconds(5.0);
      }
      
      HandSensorData robotiqHandSensorData = new RobotiqHandSensorData();
      
      HandJointAnglePacket leftHandJointAnglePacket = new HandJointAnglePacket();
      double[][] leftFingerJointAngles = robotiqHandSensorData.getFingerJointAngles(RobotSide.LEFT);
      leftHandJointAnglePacket.setAll(RobotSide.LEFT, true, true, leftFingerJointAngles[0], leftFingerJointAngles[1], leftFingerJointAngles[2]);
      
      HandJointAnglePacket rightHandJointAnglePacket = new HandJointAnglePacket();
      double[][] rightFingerJointAngles = robotiqHandSensorData.getFingerJointAngles(RobotSide.RIGHT);
      rightHandJointAnglePacket.setAll(RobotSide.RIGHT, true, true, rightFingerJointAngles[0], rightFingerJointAngles[1], rightFingerJointAngles[2]);
      
      packetCommunicator.send(rightHandJointAnglePacket);
      packetCommunicator.send(leftHandJointAnglePacket);
//      packetCommunicator.send(pointCloudWorldPacket);
      packetCommunicator.send(robotConfigurationData);
   }
   
   public static void main(String[] args) throws IOException
   {
      new AtlasNoSimPacketBlaster();
   }
}
