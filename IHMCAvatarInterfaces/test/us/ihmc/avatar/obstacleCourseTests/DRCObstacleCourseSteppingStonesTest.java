package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCObstacleCourseSteppingStonesTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   
   private DRCSimulationTestHelper drcSimulationTestHelper;

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
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


	@ContinuousIntegrationTest(estimatedDuration = 52.4)
	@Test(timeout = 260000)
   public void testWalkingOverEasySteppingStones() throws SimulationExceededMaximumTimeException
   {
      try
      {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.EASY_STEPPING_STONES;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingEasySteppingStonesTest", selectedLocation, simulationTestingParameters, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingOverEasySteppingStones(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingOverEasySteppingStones(scriptedFootstepGenerator);
      drcSimulationTestHelper.send(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(13.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);
      
      Point3D center = new Point3D(-10.241987629532595, -0.8330256660954483, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      }
      catch (Throwable throwable)
      {
         System.err.println("Caught throwable in testWalkingOverEasySteppingStones: " + throwable);
         System.err.flush();
         throw throwable;
      }
   }
   
 
   private void setupCameraForWalkingOverEasySteppingStones(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(-8.6, -0.1, 0.94);
      Point3D cameraPosition = new Point3D(-14.0, -5.0, 2.7);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private FootstepDataListMessage createFootstepsForWalkingOverEasySteppingStones(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {{{-7.72847174992541, -0.5619736174919732, 0.3839138258635628}, {-0.002564106649548222, 9.218543591576633E-4, 0.9999871158757672, 0.004282945726398341}},
         {{-8.233931300168681, -0.952122284180518, 0.3841921077973934}, {-2.649132161393031E-6, -0.00302400231893713, 0.999986265693845, 0.004280633905867881}},
         {{-8.711157422190857, -0.5634436272430561, 0.38340964898482055}, {-6.333967334144636E-4, -0.002689012266100874, 0.9999870292977306, 0.004278931865605645}},
         {{-9.246614388340875, -0.9823725639340232, 0.3838760717826556}, {4.990380502353344E-4, 0.002867206806117212, 0.9999866091454905, 0.00427920738681889}},
         {{-9.694460236661355, -0.5363354293129117, 0.3828438933446154}, {0.0043663633816866795, 6.575433167622114E-4, 0.9999811020260976, 0.004277627645902338}},
         {{-10.204483462540168, -1.0007498263499959, 0.3841142603691748}, {3.379337850421112E-4, 0.0013510800402890615, 0.9999898702179759, 0.004280168795429233}},
         {{-10.20677294790819, -0.6741336761434962, 0.3829201197142793}, {0.004772284224629501, 0.005592011887113724, 0.9999639290557834, 0.004253856327364576}}
         };
      
      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

}
