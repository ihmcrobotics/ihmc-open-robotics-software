package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import static org.junit.Assert.assertTrue;

import java.util.concurrent.ConcurrentLinkedQueue;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCWallAtDistanceEnvironment;
import us.ihmc.darpaRoboticsChallenge.networking.DRCUserInterfaceNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationNetworkTestHelper;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.compare.CompareTools;
import us.ihmc.utilities.lidar.polarLidar.SparseLidarScan;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.net.NetStateListener;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.test.JUnitTools;

import com.yobotics.simulationconstructionset.simulatedSensors.DepthDataStateCommand;
import com.yobotics.simulationconstructionset.simulatedSensors.DepthDataStateCommand.LidarState;

public abstract class DepthDataProcessorTest implements MultiRobotTestInterface, NetStateListener
{
   private static final double ANGLE_TOLERANCE = 5.0 / 180.0 * Math.PI;
   private static final float SCAN_TOLERANCE = 0.1f;
   private static final double ZERO_DEGREES = 0.0 * Math.PI;
   private static final double ONE_EIGHTY_DEGREES = 1.0 * Math.PI;
   private static final double NINETY_DEGREES = 0.5 * Math.PI;
   private static final double TWO_SEVENTY_DEGREES = 1.5 * Math.PI;
   private static final double WALL_DISTANCE = 1.0;

   private int numberOfLidarScansConsumed = 0;
   private SparseLidarScan zeroDegreeScan;
   private SparseLidarScan oneEightyDegreeScan;
   private SparseLidarScan ninetyDegreeScan;
   private SparseLidarScan twoSeventyDegreeScan;
   private ConcurrentLinkedQueue<ScanPair> scanComparisonQueue = new ConcurrentLinkedQueue<>();

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

      boolean success = drcSimulationTestHelper.simulate(0.5);

      assertTrue(success);
      assertTrue("Lidar scans are not being recieved.", numberOfLidarScansConsumed > 10);

      //compareAllScans();

      BambooTools.reportTestFinishedMessage();
   }

   private void compareAllScans()
   {
      while (!scanComparisonQueue.isEmpty())
      {
         ScanPair scanPair = scanComparisonQueue.poll();
         
         System.out.println("Comparing scan with " + scanPair.scan1.getIndexes().length + " ranges to scan with " + scanPair.scan2.getIndexes().length + " ranges.");
         
         JUnitTools.assertSparseLidarScanEquals(scanPair.scan1, scanPair.scan2.flipNew(), 1e-7, SCAN_TOLERANCE);
      }
   }

   private class LidarConsumer implements ObjectConsumer<SparseLidarScan>
   {
      @Override
      public void consumeObject(SparseLidarScan sparseLidarScan)
      {
         numberOfLidarScansConsumed++;

         verifyScan(sparseLidarScan);
      }
   }


   private void verifyScan(SparseLidarScan sparseLidarScan)
   {
      // Make sure the scans at 0 and 180 and at 90 and 270 are the same.

      checkAndAddScan(sparseLidarScan, ZERO_DEGREES);
      checkAndAddScan(sparseLidarScan, ONE_EIGHTY_DEGREES);
      checkAndAddScan(sparseLidarScan, NINETY_DEGREES);
      checkAndAddScan(sparseLidarScan, TWO_SEVENTY_DEGREES);

      if ((zeroDegreeScan != null) && (oneEightyDegreeScan != null))
      {
         scanComparisonQueue.add(new ScanPair(zeroDegreeScan, oneEightyDegreeScan));
         zeroDegreeScan = null;
         oneEightyDegreeScan = null;
      }

      if ((ninetyDegreeScan != null) && (twoSeventyDegreeScan != null))
      {
         scanComparisonQueue.add(new ScanPair(ninetyDegreeScan, twoSeventyDegreeScan));
         ninetyDegreeScan = null;
         twoSeventyDegreeScan = null;
      }

      // Make sure the scan at 45 is correct for the geometry presented to it.
   }

   private class ScanPair
   {
      SparseLidarScan scan1;
      SparseLidarScan scan2;

      public ScanPair(SparseLidarScan scan1, SparseLidarScan scan2)
      {
         this.scan1 = scan1;
         this.scan2 = scan2;
      }
   }


   private void checkAndAddScan(SparseLidarScan scan, double targetAngle)
   {
      Transform3D startTransform = scan.getStartTransform();
      Quat4d startRotationQuat = new Quat4d();
      startTransform.get(startRotationQuat);
      Vector3d rollPitchYaw = new Vector3d();
      RotationFunctions.getEulerAnglesFromQuat(rollPitchYaw, startRotationQuat);
      double lidarRotationAngle = rollPitchYaw.getZ() + Math.PI;

      if (CompareTools.withinTolerance(lidarRotationAngle, targetAngle, ANGLE_TOLERANCE))
      {
         if (targetAngle == ZERO_DEGREES)
         {
            zeroDegreeScan = scan;
         }
         else if (targetAngle == NINETY_DEGREES)
         {
            ninetyDegreeScan = scan;
         }
         else if (targetAngle == ONE_EIGHTY_DEGREES)
         {
            oneEightyDegreeScan = scan;
         }
         else if (targetAngle == TWO_SEVENTY_DEGREES)
         {
            twoSeventyDegreeScan = scan;
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
