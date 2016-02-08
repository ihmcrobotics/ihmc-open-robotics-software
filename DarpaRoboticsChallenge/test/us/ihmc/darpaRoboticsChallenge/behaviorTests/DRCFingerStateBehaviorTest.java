package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCFingerStateBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();
      
      

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCFingerStateBehaviorTest.class + " after class.");
   }

   private final boolean DEBUG = false;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @Before
   public void setUp()
   {
      DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();


      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
   }

   @DeployableTestMethod(estimatedDuration = 27.7)
   @Test(timeout = 83115)
   public void testCloseHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 2.0;

      double fingerJointQInitial = getTotalFingerJointQ(robotSide);
      FingerStateBehavior fingerStateBehavior = testFingerStateBehavior(new HandDesiredConfigurationMessage(robotSide, HandConfiguration.CLOSE), trajectoryTime);
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(fingerStateBehavior);
      assertTrue(success);
      double fingerJointQFinal = getTotalFingerJointQ(robotSide);

      PrintTools.debug(this, "fingerJointQInitial: " + fingerJointQInitial);
      PrintTools.debug(this, "fingerJointQFinal : " + fingerJointQFinal);


      assertTrue(fingerJointQFinal > fingerJointQInitial);
      assertTrue(fingerStateBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @DeployableTestMethod(estimatedDuration = 27.7)
   @Test(timeout = 83115)
   public void testStopCloseHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      PrintTools.debug(this, "Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 0.3; // [0.3] Hand closes quickly!
      double stopTime = trajectoryTime / 2.0;

      PrintTools.debug(this, "Initializing Behavior");
      FingerStateBehavior fingerStateBehavior = testFingerStateBehavior(new HandDesiredConfigurationMessage(robotSide, HandConfiguration.CLOSE), trajectoryTime);

      PrintTools.debug(this, "Starting Behavior");
      double fingerJointQInitial = getTotalFingerJointQ(robotSide);
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(fingerStateBehavior, stopTime);
      assertTrue(success);
      PrintTools.debug(this, "Stopping Behavior");
      double fingerJointQAtStop = getTotalFingerJointQ(robotSide);
      fingerStateBehavior.stop();
      assertTrue(!fingerStateBehavior.isDone());

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(fingerStateBehavior, 1.0);
      assertTrue(success);
      double fingerJointQFinal = getTotalFingerJointQ(robotSide);

      PrintTools.debug(this, "fingerJointQInitial: " + fingerJointQInitial);
      PrintTools.debug(this, "fingerJointQAtStop : " + fingerJointQAtStop);
      PrintTools.debug(this, "fingerJointQFinal : " + fingerJointQFinal);

      assertTrue(Math.abs(fingerJointQFinal - fingerJointQAtStop) < 3.0);
      assertTrue(!fingerStateBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }
   
   @DeployableTestMethod(estimatedDuration = 27.7)
   @Test(timeout = 83115)
   public void testPauseAndResumeCloseHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      PrintTools.debug(this, "Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 0.3; // [0.3] Hand closes quickly!
      double stopTime = trajectoryTime / 2.0;

      PrintTools.debug(this, "Initializing Behavior");
      FingerStateBehavior fingerStateBehavior = testFingerStateBehavior(new HandDesiredConfigurationMessage(robotSide, HandConfiguration.CLOSE), trajectoryTime);

      PrintTools.debug(this, "Starting Behavior");
      double fingerJointQInitial = getTotalFingerJointQ(robotSide);
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(fingerStateBehavior, stopTime);
      assertTrue(success);
      PrintTools.debug(this, "Pausing Behavior");
      double fingerJointQAtPause = getTotalFingerJointQ(robotSide);
      fingerStateBehavior.pause();

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(fingerStateBehavior, 1.0);
      assertTrue(success);
      PrintTools.debug(this, "Resuming Behavior");
      double fingerJointQAtResume = getTotalFingerJointQ(robotSide);
      fingerStateBehavior.resume();
      assertTrue(!fingerStateBehavior.isDone());

      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(fingerStateBehavior, 1.0);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should Be Done");
      double fingerJointQFinal = getTotalFingerJointQ(robotSide);
      fingerStateBehavior.resume();

      PrintTools.debug(this, "fingerJointQInitial: " + fingerJointQInitial);
      PrintTools.debug(this, "fingerJointQAtPause : " + fingerJointQAtPause);
      PrintTools.debug(this, "fingerJointQAtResume : " + fingerJointQAtResume);
      PrintTools.debug(this, "fingerJointQFinal : " + fingerJointQFinal);

      assertTrue(Math.abs(fingerJointQAtResume - fingerJointQAtPause) < 3.0);
      assertTrue(fingerJointQFinal > fingerJointQAtResume);
//      assertTrue(fingerStateBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   private HandDesiredConfigurationMessage getRandomClosedTypeFingerStatePacket(RobotSide robotSide)
   {
      ArrayList<HandConfiguration> closedFingerConfigs = new ArrayList<HandConfiguration>();
      closedFingerConfigs.add(HandConfiguration.CLOSE);
      closedFingerConfigs.add(HandConfiguration.CLOSE_FINGERS);
      closedFingerConfigs.add(HandConfiguration.CLOSE_THUMB);
      closedFingerConfigs.add(HandConfiguration.CRUSH);
      closedFingerConfigs.add(HandConfiguration.CRUSH_INDEX);
      closedFingerConfigs.add(HandConfiguration.CRUSH_MIDDLE);
      closedFingerConfigs.add(HandConfiguration.CRUSH_THUMB);

      HandConfiguration fingerState = closedFingerConfigs.get(RandomTools.generateRandomInt(new Random(), 0, closedFingerConfigs.size() - 1));
      if (DEBUG)
      {
         PrintTools.debug(this, fingerState.name());
      }

      return new HandDesiredConfigurationMessage(robotSide, fingerState);
   }

   private double getTotalFingerJointQ(RobotSide robotSide)
   {
      double ret = 0.0;

      ArrayList<OneDegreeOfFreedomJoint> fingerJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      SDFJointNameMap jointNameMap = (SDFJointNameMap) drcBehaviorTestHelper.getSDFFullRobotModel().getRobotSpecificJointNames();
      Joint wristJoint = drcBehaviorTestHelper.getRobot().getJoint(jointNameMap.getJointBeforeHandName(robotSide));
      wristJoint.recursiveGetOneDegreeOfFreedomJoints(fingerJoints);
      fingerJoints.remove(0);

      for (OneDegreeOfFreedomJoint fingerJoint : fingerJoints)
      {
         double q = fingerJoint.getQ().getDoubleValue();
         ret += q;
         if (DEBUG)
         {
            PrintTools.debug(this, fingerJoint.getName() + " q : " + q);
         }
      }

      return ret;
   }

   private BoundingBox3d getDistalFingerJointBoundingBox(RobotSide robotSide)
   {
      ArrayList<OneDegreeOfFreedomJoint> fingerJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      SDFJointNameMap jointNameMap = (SDFJointNameMap) drcBehaviorTestHelper.getSDFFullRobotModel().getRobotSpecificJointNames();
      Joint wristJoint = drcBehaviorTestHelper.getRobot().getJoint(jointNameMap.getJointBeforeHandName(robotSide));
      wristJoint.recursiveGetOneDegreeOfFreedomJoints(fingerJoints);
      fingerJoints.remove(0);

      ArrayList<OneDegreeOfFreedomJoint> mostDistalJoints = new ArrayList<OneDegreeOfFreedomJoint>();

      for (OneDegreeOfFreedomJoint fingerJoint : fingerJoints)
      {
         if (fingerJoint.childrenJoints.size() == 0)
            mostDistalJoints.add(fingerJoint);
      }
      
      PrintTools.debug(this, "mostDistalJoints: " + mostDistalJoints);
      
      BoundingBox3d boundingBoxOld = null;

      while(mostDistalJoints.size() >= 2)
      {
         Vector3d translationToWorldA = new Vector3d();
         mostDistalJoints.remove(0).getTranslationToWorld(translationToWorldA);
         Point3d positionInWorldA = new Point3d(translationToWorldA);
         
         Vector3d translationToWorldB = new Vector3d();
         mostDistalJoints.remove(0).getTranslationToWorld(translationToWorldB);
         Point3d positionInWorldB = new Point3d(translationToWorldB);

         if(boundingBoxOld != null)
         {
            double xMin = Math.min(positionInWorldA.getX(), positionInWorldB.getX());
            double yMin = Math.min(positionInWorldA.getY(), positionInWorldB.getY());
            double zMin = Math.min(positionInWorldA.getZ(), positionInWorldB.getZ());

            double xMax = Math.max(positionInWorldA.getX(), positionInWorldB.getX());
            double yMax = Math.max(positionInWorldA.getY(), positionInWorldB.getY());
            double zMax = Math.max(positionInWorldA.getZ(), positionInWorldB.getZ());

            BoundingBox3d boundingBoxNew = new BoundingBox3d(xMin, yMin, zMin, xMax, yMax, zMax);
            
            boundingBoxOld = BoundingBox3d.union(boundingBoxOld, boundingBoxNew);
         }
         else
         {
            double xMin = Math.min(positionInWorldA.getX(), positionInWorldB.getX());
            double yMin = Math.min(positionInWorldA.getY(), positionInWorldB.getY());
            double zMin = Math.min(positionInWorldA.getZ(), positionInWorldB.getZ());

            double xMax = Math.max(positionInWorldA.getX(), positionInWorldB.getX());
            double yMax = Math.max(positionInWorldA.getY(), positionInWorldB.getY());
            double zMax = Math.max(positionInWorldA.getZ(), positionInWorldB.getZ());

            boundingBoxOld = new BoundingBox3d(xMin, yMin, zMin, xMax, yMax, zMax);
         }
      }
      
      return boundingBoxOld;
   }
   
   private double getDistalFingerJointBoundingBoxSize(RobotSide robotSide)
   {
      BoundingBox3d boundingBox = getDistalFingerJointBoundingBox(robotSide);
      
      Point3d minPoint = new Point3d();
      Point3d maxPoint = new Point3d();
      
      boundingBox.getMinPoint(minPoint);
      boundingBox.getMaxPoint(maxPoint);
      
      double size = minPoint.distance(maxPoint);
      
      return size;
   }

   private FingerStateBehavior testFingerStateBehavior(HandDesiredConfigurationMessage fingerStatePacket, double trajectoryTime)
         throws SimulationExceededMaximumTimeException
   {
      final FingerStateBehavior fingerStateBehavior = new FingerStateBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getYoTime());

      fingerStateBehavior.initialize();
      fingerStateBehavior.setInput(fingerStatePacket);
      assertTrue(fingerStateBehavior.hasInputBeenSet());

      return fingerStateBehavior;
   }
}
