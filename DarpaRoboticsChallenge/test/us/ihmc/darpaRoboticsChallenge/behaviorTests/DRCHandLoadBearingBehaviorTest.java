package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Handstep;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.communication.packets.manipulation.HandLoadBearingPacket;
import us.ihmc.communication.packets.manipulation.HandstepPacket;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCWallWorldEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.darpaRoboticsChallenge.testTools.ScriptedHandstepGenerator;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandLoadBearingBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.GlobalTimer;

public abstract class DRCHandLoadBearingBehaviorTest implements MultiRobotTestInterface
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
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCHandLoadBearingBehaviorTest.class + " after class.");
   }

   private static final boolean DEBUG = false;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @Before
   public void setUp()
   {
      double wallMaxY = 3.5;
      DRCWallWorldEnvironment testEnvironment = new DRCWallWorldEnvironment(-1.0, wallMaxY);

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
   }

   @SuppressWarnings("unchecked")
   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 90137)
   public void testHandLoadBearingBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      ScriptedHandstepGenerator scriptedHandstepGenerator = drcBehaviorTestHelper.createScriptedHandstepGenerator();

      EnumYoVariable<HandControlState> leftHandControlModule = (EnumYoVariable<HandControlState>) drcBehaviorTestHelper.getSimulationConstructionSet().getVariable("leftHandControlModule");
      EnumYoVariable<HandControlState> rightHandControlModule = (EnumYoVariable<HandControlState>) drcBehaviorTestHelper.getSimulationConstructionSet().getVariable("leftHandControlModule");
      
      assertTrue(leftHandControlModule.getEnumValue().equals(HandControlState.JOINT_SPACE));
      assertTrue(rightHandControlModule.getEnumValue().equals(HandControlState.JOINT_SPACE));
      
      double bodyY = 0.0;

      double leftHandstepY = bodyY + 0.25;
      double rightHandstepY = bodyY - 0.25;
      ArrayList<Handstep> handsteps = createHandstepsForTesting(leftHandstepY, rightHandstepY, scriptedHandstepGenerator);

      for (Handstep handstep : handsteps)
      {
         Point3d location = new Point3d();
         Quat4d orientation = new Quat4d();
         Vector3d surfaceNormal = new Vector3d();
         handstep.getPose(location, orientation);
         handstep.getSurfaceNormal(surfaceNormal);

         HandstepPacket handstepPacket = new HandstepPacket(handstep.getRobotSide(), location, orientation, surfaceNormal, handstep.getSwingTrajectoryTime());
         PrintTools.debug(this, "Sending Handstep Packet to Controller");
         drcBehaviorTestHelper.getBehaviorCommunicationBridge().sendPacketToController(handstepPacket);
         success = success && drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.5);
      }

      double swingTrajectoryTime = 2.0;
      RobotSide robotSide = RobotSide.LEFT;

      final HandLoadBearingBehavior handLoadBearingBehavior = new HandLoadBearingBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge());

      PrintTools.debug(this, "Initializing Behavior");
      HandLoadBearingPacket handLoadBearingPacket = new HandLoadBearingPacket(robotSide, true);
      handLoadBearingBehavior.initialize();
      handLoadBearingBehavior.setInput(handLoadBearingPacket);
      assertTrue(handLoadBearingBehavior.hasInputBeenSet());

      PrintTools.debug(this, "Starting to Execute Behavior");
      success &= drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(handLoadBearingBehavior, swingTrajectoryTime
            + EXTRA_SIM_TIME_FOR_SETTLING);
      PrintTools.debug(this, "Behavior Should be done");

      assertTrue(leftHandControlModule.getEnumValue().equals(HandControlState.LOAD_BEARING));
      assertTrue(rightHandControlModule.getEnumValue().equals(HandControlState.LOAD_BEARING));
      
      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   private ArrayList<Handstep> createHandstepsForTesting(double leftHandstepY, double rightHandstepY, ScriptedHandstepGenerator scriptedHandstepGenerator)
   {
      ArrayList<Handstep> ret = new ArrayList<Handstep>();

      RobotSide robotSide = RobotSide.LEFT;
      Tuple3d position = new Point3d(0.6, leftHandstepY, 1.0);
      Vector3d surfaceNormal = new Vector3d(-1.0, 0.0, 0.0);
      double rotationAngleAboutNormal = 0.0;
      double swingTrajectoryTime = 1.0;

      Handstep handstep = scriptedHandstepGenerator.createHandstep(robotSide, position, surfaceNormal, rotationAngleAboutNormal, swingTrajectoryTime);
      ret.add(handstep);

      robotSide = RobotSide.RIGHT;
      position = new Point3d(0.6, rightHandstepY, 1.0);
      surfaceNormal = new Vector3d(-1.0, 0.0, 0.0);
      rotationAngleAboutNormal = 0.0;

      handstep = scriptedHandstepGenerator.createHandstep(robotSide, position, surfaceNormal, rotationAngleAboutNormal, swingTrajectoryTime);
      ret.add(handstep);

      return ret;
   }

}
