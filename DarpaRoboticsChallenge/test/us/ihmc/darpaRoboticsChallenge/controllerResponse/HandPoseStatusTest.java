package us.ihmc.darpaRoboticsChallenge.controllerResponse;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.packets.manipulation.HandPoseStatus;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.net.KryoLocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.robotSide.RobotSide;

public abstract class HandPoseStatusTest implements MultiRobotTestInterface
{
   private KryoLocalObjectCommunicator networkObjectCommunicator = new KryoLocalObjectCommunicator(new IHMCCommunicationKryoNetClassList());
   private DRCDemo01NavigationEnvironment demo01NavEnvironmant = new DRCDemo01NavigationEnvironment();

   private boolean showGUI = false;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private boolean hasSimulationBeenInitialized;

   private int statusStartedCounter = 0;
   private int statusCompletedCounter = 0;

   private int leftStatusStartedCounter = 0;
   private int leftStatusCompletedCounter = 0;
   private int rightStatusStartedCounter = 0;
   private int rightStatusCompletedCounter = 0;

   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @Before
   public void setUp()
   {
      showMemoryUsageBeforeTest();
      drcSimulationTestHelper = new DRCSimulationTestHelper(demo01NavEnvironmant, networkObjectCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, false, showGUI, false, false, getRobotModel());

   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private void sendHandPosePacket(HandPosePacket handPosePacket)
   {
      networkObjectCommunicator.consumeObject(handPosePacket);
   }

   private HandPosePacket createRandomHandPosePacket()
   {

      RobotSide robotSide = RandomTools.generateRandomRobotSide(new Random());
      Point3d position = RandomTools.generateRandomPoint(new Random(), 0.1, -0.2, -0.3, 0.5, 0.2, 0.3);
      Quat4d orientation = RandomTools.generateRandomQuaternion(new Random(), Math.PI / 4);

      HandPosePacket handPosePacket = new HandPosePacket(robotSide, Frame.CHEST, position, orientation, 0.4);
      handPosePacket.setDestination(PacketDestination.CONTROLLER);
      return handPosePacket;
   }

   private HandPosePacket createRandomHandPosePacketWithRobotSide(RobotSide robotSide)
   {

      Point3d position = RandomTools.generateRandomPoint(new Random(), 0.1, -0.2, -0.3, 0.5, 0.2, 0.3);
      Quat4d orientation = RandomTools.generateRandomQuaternion(new Random(), Math.PI / 4);

      HandPosePacket handPosePacket = new HandPosePacket(robotSide, Frame.CHEST, position, orientation, 0.4);
      handPosePacket.setDestination(PacketDestination.CONTROLLER);
      return handPosePacket;
   }

   @Test
   public void testStartedAndCompletedStatusAreSentAndReceivedForOneHandPose() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      statusStartedCounter = 0;
      statusCompletedCounter = 0;

      hasSimulationBeenInitialized = false;

      networkObjectCommunicator.attachListener(HandPoseStatus.class, new ObjectConsumer<HandPoseStatus>()
      {
         @Override
         public void consumeObject(HandPoseStatus object)
         {
            if (object.getStatus() == HandPoseStatus.Status.STARTED && hasSimulationBeenInitialized)
               statusStartedCounter++;

            if (object.getStatus() == HandPoseStatus.Status.COMPLETED && hasSimulationBeenInitialized)
               statusCompletedCounter++;
         }
      });

      HandPosePacket outgoingHandPosePacket = createRandomHandPosePacket();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.1);
      hasSimulationBeenInitialized = true;

      sendHandPosePacket(outgoingHandPosePacket);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      assertTrue((statusStartedCounter == 1) && (statusCompletedCounter == 1));
      BambooTools.reportTestFinishedMessage();
   }

   @Test
   public void testWhenTwoHandPosesAreSentInARow() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      statusStartedCounter = 0;
      statusCompletedCounter = 0;

      hasSimulationBeenInitialized = false;

      networkObjectCommunicator.attachListener(HandPoseStatus.class, new ObjectConsumer<HandPoseStatus>()
      {
         @Override
         public void consumeObject(HandPoseStatus object)
         {
            if (object.getStatus() == HandPoseStatus.Status.STARTED && hasSimulationBeenInitialized)
               statusStartedCounter++;

            if (object.getStatus() == HandPoseStatus.Status.COMPLETED && hasSimulationBeenInitialized)
               statusCompletedCounter++;
         }
      });

      RobotSide robotSide = RandomTools.generateRandomRobotSide(new Random());

      final HandPosePacket outgoingHandPosePacket_1 = createRandomHandPosePacketWithRobotSide(robotSide);
      final HandPosePacket outgoingHandPosePacket_2 = createRandomHandPosePacketWithRobotSide(robotSide);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.1);
      hasSimulationBeenInitialized = true;

      Runnable myRunnable = new Runnable()
      {
         @Override
         public void run()
         {
            sendHandPosePacket(outgoingHandPosePacket_1);
            ThreadTools.sleep(200);
            sendHandPosePacket(outgoingHandPosePacket_2);
         }
      };

      Thread handposeThread = new Thread(myRunnable);
      handposeThread.start();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue((statusStartedCounter == 2) && (statusCompletedCounter == 1));

      BambooTools.reportTestFinishedMessage();
   }

   @Test
   public void testEachArmReceiveOneHandPoseAtTheSameTime() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      leftStatusStartedCounter = 0;
      leftStatusCompletedCounter = 0;
      rightStatusStartedCounter = 0;
      rightStatusCompletedCounter = 0;

      hasSimulationBeenInitialized = false;

      networkObjectCommunicator.attachListener(HandPoseStatus.class, new ObjectConsumer<HandPoseStatus>()
      {
         @Override
         public void consumeObject(HandPoseStatus object)
         {
            if (object.getStatus() == HandPoseStatus.Status.STARTED && hasSimulationBeenInitialized)
            {
               if (object.getRobotSide() == RobotSide.LEFT)
                  leftStatusStartedCounter++;
               else
                  rightStatusStartedCounter++;
            }
            if (object.getStatus() == HandPoseStatus.Status.COMPLETED && hasSimulationBeenInitialized)
            {
               if (object.getRobotSide() == RobotSide.LEFT)
                  leftStatusCompletedCounter++;
               else
                  rightStatusCompletedCounter++;
            }
         }
      });

      final HandPosePacket outgoingHandPosePacket_1 = createRandomHandPosePacketWithRobotSide(RobotSide.LEFT);
      final HandPosePacket outgoingHandPosePacket_2 = createRandomHandPosePacketWithRobotSide(RobotSide.RIGHT);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.1);
      hasSimulationBeenInitialized = true;

      Runnable myRunnable = new Runnable()
      {
         @Override
         public void run()
         {
            sendHandPosePacket(outgoingHandPosePacket_1);
            ThreadTools.sleep(200);
            sendHandPosePacket(outgoingHandPosePacket_2);
         }
      };

      Thread handposeThread = new Thread(myRunnable);
      handposeThread.start();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue((leftStatusStartedCounter == 1) && (rightStatusStartedCounter == 1) && (leftStatusCompletedCounter == 1) && (rightStatusCompletedCounter == 1));
      BambooTools.reportTestFinishedMessage();
   }
}
