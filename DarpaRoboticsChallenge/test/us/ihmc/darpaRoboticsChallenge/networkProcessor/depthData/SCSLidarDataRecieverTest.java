package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import static org.junit.Assert.assertTrue;

import java.io.IOException;

import javax.vecmath.Point3d;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCWallWorldEnvironment;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.networking.DRCUserInterfaceNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationNetworkTestHelper;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.lidar.polarLidar.SparseLidarScan;
import us.ihmc.utilities.net.NetStateListener;
import us.ihmc.utilities.net.ObjectConsumer;

import com.yobotics.simulationconstructionset.simulatedSensors.DepthDataStateCommand;
import com.yobotics.simulationconstructionset.simulatedSensors.DepthDataStateCommand.LidarState;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public abstract class SCSLidarDataRecieverTest implements MultiRobotTestInterface, NetStateListener
{
   private int numberOfLidarScansConsumed = 0;
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test. ");
   }

   @Test
   public void testSCSLidarDataReceiver() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCSimulationNetworkTestHelper drcSimulationTestHelper = new DRCSimulationNetworkTestHelper(getRobotModel(), new DRCWallWorldEnvironment());
      drcSimulationTestHelper.getDRCSimulationTestHelper().setupCameraForUnitTest(new Point3d(1.8375, -0.16, 0.89), new Point3d(1.10, 8.30, 1.37));

      // TODO Listen to tcp stream rather than local object communicator

      DRCUserInterfaceNetworkingManager networkingManager = new DRCUserInterfaceNetworkingManager("localhost", getRobotModel());
      networkingManager.attachStateListener(this);

      try
      {
         networkingManager.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      System.out.println(getClass().getSimpleName() + ": Waiting to connect...");
      while (!networkingManager.isConnected());

      networkingManager.getControllerHandler().send(new DepthDataStateCommand(LidarState.ENABLE));
      networkingManager.getVisualizationHandler().attachListener(SparseLidarScan.class, new LidarConsumer());

      boolean success = drcSimulationTestHelper.getDRCSimulationTestHelper().simulateAndBlockAndCatchExceptions(0.5);

      // Put drc robot in world, bring up window

      // TODO
      // Put known object in world
      // Get Object Communicator
      // Receive Sparse Lidar Packet
      // Check points

      assertTrue(success);
      assertTrue("Lidar scans are not being recieved.", numberOfLidarScansConsumed > 10);
      

      BambooTools.reportTestFinishedMessage();
   }

   private class LidarConsumer implements ObjectConsumer<SparseLidarScan>
   {
      @Override
      public void consumeObject(SparseLidarScan object)
      {
         numberOfLidarScansConsumed++;
         
         System.out.println(SCSLidarDataRecieverTest.this.getClass().getSimpleName() + ": " + object.getClass().getSimpleName() + " received!" + "count = " + numberOfLidarScansConsumed);
         
         verifyScan(object);
      }
   }

   public void verifyScan(SparseLidarScan sparseLidarScan)
   {
      System.out.println("Scan: ");
      
      for (int i = 0; i < sparseLidarScan.size(); i++)
      {
         System.out.print(sparseLidarScan.getRange(i) + ", ");
      }
      
      System.out.println();
   }

   public void connected()
   {
      System.out.println(DRCUserInterfaceNetworkingManager.class.getSimpleName() + ": Connected!");
   }

   public void disconnected()
   {
      System.out.println("DRCUserInterfaceNetworkingManager: Disconnected.");
   }
}
