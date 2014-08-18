package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import static org.junit.Assert.assertTrue;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCWallWorldEnvironment;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.networking.DRCUserInterfaceNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationNetworkTestHelper;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.lidar.polarLidar.AbstractLidarScan;
import us.ihmc.utilities.lidar.polarLidar.SparseLidarScan;
import us.ihmc.utilities.net.NetStateListener;
import us.ihmc.utilities.net.ObjectConsumer;

import com.yobotics.simulationconstructionset.simulatedSensors.DepthDataStateCommand;
import com.yobotics.simulationconstructionset.simulatedSensors.DepthDataStateCommand.LidarState;

public abstract class DepthDataProcessorTest implements MultiRobotTestInterface, NetStateListener
{
   private int numberOfLidarScansConsumed = 0;
   private AbstractLidarScan zeroDegreeScan;

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before: ");
   }

   @Test
   public void testLidarGenerationAndTransmission()
   {
      BambooTools.reportTestStartedMessage();

      DRCSimulationNetworkTestHelper drcSimulationTestHelper = new DRCSimulationNetworkTestHelper(getRobotModel(), new DRCWallWorldEnvironment(-10.0, 10.0));
      drcSimulationTestHelper.setupCamera(new Point3d(1.8375, -0.16, 0.89), new Point3d(1.10, 8.30, 1.37));
      drcSimulationTestHelper.addNetStateListener(this);
      drcSimulationTestHelper.addConsumer(SparseLidarScan.class, new LidarConsumer());

      drcSimulationTestHelper.connect();

      drcSimulationTestHelper.sendCommand(new DepthDataStateCommand(LidarState.ENABLE));

      boolean success = drcSimulationTestHelper.simulate(0.5);

      assertTrue(success);
      assertTrue("Lidar scans are not being recieved.", numberOfLidarScansConsumed > 10);

      BambooTools.reportTestFinishedMessage();
   }

   private class LidarConsumer implements ObjectConsumer<SparseLidarScan>
   {
      @Override
      public void consumeObject(SparseLidarScan sparseLidarScan)
      {
         numberOfLidarScansConsumed++;

         System.out.println(DepthDataProcessorTest.this.getClass().getSimpleName() + ": " + sparseLidarScan.getClass().getSimpleName() + " received!" + "count = "
                            + numberOfLidarScansConsumed);
         
         verifyScan(sparseLidarScan);
      }
   }
   
   private void verifyScan(SparseLidarScan sparseLidarScan)
   {
      // Make sure the scans at 0 and 180 and at 90 and 270 are the same.
      
      // Make sure the scan is correct for the geometry presented to it.
      
      Transform3D startTransform = sparseLidarScan.getStartTransform();
      
      sparseLidarScan.getAllPoints();
      
      
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
