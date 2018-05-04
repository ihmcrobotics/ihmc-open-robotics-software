package us.ihmc.avatar.networkProcessor.depthData;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import controller_msgs.msg.dds.PointCloudWorldPacket;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.WallAtDistanceEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class HumanoidPointCloudDataReceiverTest implements MultiRobotTestInterface
{
   private static final int MINIMUM_SCANS_TO_RECIEVE = 10; // GPU Benchmark
   private static final float SCAN_TOLERANCE = 0.001f;
   private static final double WALL_DISTANCE = 1.0;
   private static final boolean ALLOW_PERCENTAGE_OUT_OF_RANGE = true;
   private static final double PERCENT_ALLOWABLE_OUT_OF_RANGE = .05;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private int numberOfLidarScansConsumed = 0;
   private long numberOfLidarPointsConsumed = 0;
   private final ConcurrentLinkedQueue<AssertionError> errorQueue = new ConcurrentLinkedQueue<>();
//   private JMELidarScanVisualizer jmeLidarScanVisualizer;
   private DRCSimulationTestHelper testHelper;

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before: ");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (testHelper != null)
      {
         testHelper.destroySimulation();
         testHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationTest(estimatedDuration = 28.6)
   @Test(timeout = 140000)
   public void testIsReceivingScansAnd95PercentOfPointsAreCorrect() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

//      jmeLidarScanVisualizer = new JMELidarScanVisualizer();

      DRCNetworkModuleParameters drcNetworkModuleParameters = new DRCNetworkModuleParameters();
      drcNetworkModuleParameters.enableNetworkProcessor(true);
      drcNetworkModuleParameters.enableBehaviorModule(false);
      drcNetworkModuleParameters.enableBehaviorVisualizer(false);
      drcNetworkModuleParameters.enableROSAPICommunicator(true);
      drcNetworkModuleParameters.enableHandModule(true);
      drcNetworkModuleParameters.enableLocalControllerCommunicator(true);
      drcNetworkModuleParameters.enablePerceptionModule(true);
      drcNetworkModuleParameters.enableRosModule(false);
      drcNetworkModuleParameters.enableSensorModule(true);
      drcNetworkModuleParameters.enableUiModule(true);

      DRCObstacleCourseStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface = new WallAtDistanceEnvironment(WALL_DISTANCE);
      testHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      testHelper.setTestEnvironment(commonAvatarEnvironmentInterface);
      testHelper.setStartingLocation(startingLocation);
      testHelper.setNetworkProcessorParameters(drcNetworkModuleParameters);
      testHelper.createSimulation(getClass().getSimpleName());
      testHelper.setupCameraForUnitTest(new Point3D(1.8375, -0.16, 0.89), new Point3D(1.10, 8.30, 1.37));

      testHelper.simulateAndBlockAndCatchExceptions(1.1); // Wait for sim to initialize

      try
      {
         PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient("localhost", NetworkPorts.NETWORK_PROCESSOR_TO_UI_TCP_PORT, new IHMCCommunicationKryoNetClassList());
         packetCommunicator.attachListener(PointCloudWorldPacket.class, new PointCloudWorldConsumer());
         packetCommunicator.connect();

         testHelper.simulateAndBlockAndCatchExceptions(1.1); // Wait for KryoObjectClient to connect

//         DepthDataStateCommand lidarEnablePacket = new DepthDataStateCommand(LidarState.ENABLE);
//         lidarEnablePacket.setDestination(PacketDestination.SENSOR_MANAGER);
//         packetCommunicator.send(lidarEnablePacket);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      boolean success = testHelper.simulateAndBlockAndCatchExceptions(7.0);

      assertTrue(success);

      PrintTools.info(this, "Scans consumed: " + numberOfLidarScansConsumed);
      assertTrue("Lidar scans are not being received; numberOfLidarScansConsumed = " + numberOfLidarScansConsumed, numberOfLidarScansConsumed > MINIMUM_SCANS_TO_RECIEVE);

      PrintTools.info(this, "Number of points consumed: " + numberOfLidarPointsConsumed + " Points out of range: " + errorQueue.size() + " Percentage: " + ((double) errorQueue.size() / numberOfLidarPointsConsumed) + " less than .05");

      assertTrue("Too many points are out of range: errors: " + errorQueue.size() + "/" + numberOfLidarPointsConsumed, (double) errorQueue.size() / numberOfLidarPointsConsumed < PERCENT_ALLOWABLE_OUT_OF_RANGE);
      if (!ALLOW_PERCENTAGE_OUT_OF_RANGE)
         throwAllAssertionErrors();

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void throwAllAssertionErrors()
   {
      while (!errorQueue.isEmpty())
      {
         throw errorQueue.poll();
      }
   }

   private class PointCloudWorldConsumer implements PacketConsumer<PointCloudWorldPacket>
   {
      @Override
      public void receivedPacket(PointCloudWorldPacket pointCloud)
      {
         numberOfLidarScansConsumed++;
//         jmeLidarScanVisualizer.addPointCloud(Arrays.asList(pointCloud.getDecayingWorldScan()));

         try
         {
            List<Point3D32> lidarWorldPoints = Arrays.asList(HumanoidMessageTools.getDecayingWorldScan(pointCloud));
            numberOfLidarPointsConsumed += lidarWorldPoints.size();

            for (Point3D32 lidarWorldPoint : lidarWorldPoints)
            {
               if (lidarWorldPoint.getX() > 0.5)
               {
                  assertEquals(WALL_DISTANCE, lidarWorldPoint.getX(), SCAN_TOLERANCE);
               }
            }
         }
         catch (AssertionError assertionError)
         {
            errorQueue.add(assertionError);
         }
      }
   }
}
