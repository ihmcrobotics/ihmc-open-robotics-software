package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import static org.junit.Assert.*;

import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.vecmath.Point3d;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCWallAtDistanceEnvironment;
import us.ihmc.darpaRoboticsChallenge.networking.DRCUserInterfaceNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationNetworkTestHelper;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.lidar.polarLidar.SparseLidarScan;
import us.ihmc.utilities.net.NetStateListener;
import us.ihmc.utilities.net.ObjectConsumer;

import com.yobotics.simulationconstructionset.simulatedSensors.DepthDataStateCommand;
import com.yobotics.simulationconstructionset.simulatedSensors.DepthDataStateCommand.LidarState;

public abstract class DepthDataProcessorTest implements MultiRobotTestInterface, NetStateListener
{
   private static final float SCAN_TOLERANCE = 0.001f;
   private static final double WALL_DISTANCE = 1.0;

   private int numberOfLidarScansConsumed = 0;
   private long numberOfLidarPointsConsumed = 0;
   private ConcurrentLinkedQueue<AssertionError> errorQueue = new ConcurrentLinkedQueue<>();

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before: ");
   }

   @Test
   public void testLidarGenerationAndTransmission()
   {
      BambooTools.reportTestStartedMessage();

      DRCSimulationNetworkTestHelper drcSimulationTestHelper = new DRCSimulationNetworkTestHelper(getRobotModel(), new DRCWallAtDistanceEnvironment(WALL_DISTANCE));
      drcSimulationTestHelper.setupCamera(new Point3d(1.8375, -0.16, 0.89), new Point3d(1.10, 8.30, 1.37));
      drcSimulationTestHelper.addNetStateListener(this);
      drcSimulationTestHelper.addConsumer(SparseLidarScan.class, new LidarConsumer());

      drcSimulationTestHelper.connect();

      drcSimulationTestHelper.sendCommand(new DepthDataStateCommand(LidarState.ENABLE));

      boolean success = drcSimulationTestHelper.simulate(5);

      assertTrue(success);
      assertTrue("Lidar scans are not being recieved.", numberOfLidarScansConsumed > 10);

      System.out.println("Number of points consumed: " + numberOfLidarPointsConsumed + " Points out of range: " + errorQueue.size() + " Percentage: " + ((double) errorQueue.size() / numberOfLidarPointsConsumed));
      
      // Assert that 95% of points are within range
      assertTrue("Too many points are out of range: ", (double) errorQueue.size() / numberOfLidarPointsConsumed < .05);
//      throwAllAssertionErrors();

      BambooTools.reportTestFinishedMessage();
   }

   private void throwAllAssertionErrors()
   {
      while (!errorQueue.isEmpty())
      {
         throw errorQueue.poll();
      }
   }

   private class LidarConsumer implements ObjectConsumer<SparseLidarScan>
   {
      @Override
      public void consumeObject(SparseLidarScan sparseLidarScan)
      {
         numberOfLidarScansConsumed++;

         try
         {
            List<Point3d> lidarWorldPoints = sparseLidarScan.getAllPoints();
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

   public void connected()
   {
      System.out.println(DRCUserInterfaceNetworkingManager.class.getSimpleName() + ": Connected!");
   }

   public void disconnected()
   {
      System.out.println(DRCUserInterfaceNetworkingManager.class.getSimpleName() + ": Disconnected.");
   }
}
