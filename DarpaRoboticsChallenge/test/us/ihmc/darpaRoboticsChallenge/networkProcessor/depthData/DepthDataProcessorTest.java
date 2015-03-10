package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.vecmath.Point3d;

import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.sensing.DepthDataStateCommand;
import us.ihmc.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.communication.packets.sensing.PointCloudPacket;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCWallAtDistanceEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationNetworkTestHelper;
import us.ihmc.graphics3DAdapter.jme.util.JMELidarScanVisualizer;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;

public abstract class DepthDataProcessorTest implements MultiRobotTestInterface
{
   private static final int MINIMUM_SCANS_TO_RECIEVE = 60; // GPU Benchmark
   private static final float SCAN_TOLERANCE = 0.001f;
   private static final double WALL_DISTANCE = 1.0;
   private static final boolean ALLOW_PERCENTAGE_OUT_OF_RANGE = true;
   private static final double PERCENT_ALLOWABLE_OUT_OF_RANGE = .05;

   private int numberOfLidarScansConsumed = 0;
   private long numberOfLidarPointsConsumed = 0;
   private final ConcurrentLinkedQueue<AssertionError> errorQueue = new ConcurrentLinkedQueue<>();
   private JMELidarScanVisualizer jmeLidarScanVisualizer;
   
   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before: ");
   }

   @Ignore

	@EstimatedDuration
	@Test(timeout=300000) 
   public void testIsReceivingScansAnd95PercentOfPointsAreCorrect()
   {
      BambooTools.reportTestStartedMessage();
      
      jmeLidarScanVisualizer = new JMELidarScanVisualizer();
      
      DRCSimulationNetworkTestHelper drcSimulationTestHelper = new DRCSimulationNetworkTestHelper(getRobotModel(),
            new DRCWallAtDistanceEnvironment(WALL_DISTANCE),"",true,true);
      drcSimulationTestHelper.setupCamera(new Point3d(1.8375, -0.16, 0.89), new Point3d(1.10, 8.30, 1.37));
      drcSimulationTestHelper.addConsumer(PointCloudPacket.class, new LidarConsumer());
      
      drcSimulationTestHelper.connect();
      
      drcSimulationTestHelper.sendCommand(new DepthDataStateCommand(LidarState.ENABLE));
      
      boolean success = drcSimulationTestHelper.simulate(5);
      
      assertTrue(success);
      
      System.out.println("Scans consumed: " + numberOfLidarScansConsumed);
      assertTrue("Lidar scans are not being received; numberOfLidarScansConsumed = " + numberOfLidarScansConsumed, numberOfLidarScansConsumed > MINIMUM_SCANS_TO_RECIEVE);
      
      System.out.println("Number of points consumed: " + numberOfLidarPointsConsumed + " Points out of range: " + errorQueue.size() + " Percentage: "
            + ((double) errorQueue.size() / numberOfLidarPointsConsumed) + " less than .05");
      
      assertTrue("Too many points are out of range: ", (double) errorQueue.size() / numberOfLidarPointsConsumed < PERCENT_ALLOWABLE_OUT_OF_RANGE);
      if (!ALLOW_PERCENTAGE_OUT_OF_RANGE)
         throwAllAssertionErrors();
      
      BambooTools.reportTestFinishedMessage();
   }

   private void throwAllAssertionErrors()
   {
      while (!errorQueue.isEmpty())
      {
         throw errorQueue.poll();
      }
   }

   private class LidarConsumer implements PacketConsumer<PointCloudPacket>
   {
      @Override
      public void receivedPacket(PointCloudPacket pointCloud)
      {
         numberOfLidarScansConsumed++;
//         jmeLidarScanVisualizer.updateLidarNodeTransform(sparseLidarScan.getStartTransform());
         jmeLidarScanVisualizer.addPointCloud(Arrays.asList(pointCloud.getPoints3f()));

         try
         {
            List<Point3d> lidarWorldPoints = Arrays.asList(pointCloud.getPoints());
            numberOfLidarPointsConsumed += lidarWorldPoints.size();
            
            for (Point3d lidarWorldPoint : lidarWorldPoints)
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
