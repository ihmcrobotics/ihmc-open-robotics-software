package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCObstacleCourseRocksTest implements MultiRobotTestInterface
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


	@ContinuousIntegrationTest(estimatedDuration = 51.6)
	@Test(timeout = 260000)
   public void testWalkingOntoRocks() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ROCKS;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingOntoRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingOntoRocks(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingToTheRocks(scriptedFootstepGenerator);
      drcSimulationTestHelper.send(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.5);

      if (success)
      {
         footstepDataList = createFootstepsForSteppingOntoTheRocks(scriptedFootstepGenerator);
         drcSimulationTestHelper.send(footstepDataList);

         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue("Caught Exception: " + drcSimulationTestHelper.getCaughtException(), success);

      Point3D center = new Point3D(0.6853965087476173, 4.5173529666394305, 0.8898586980716016);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);


      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


   private void setupCameraForWalkingOntoRocks(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(0.1, 3.2, 0.5);
      Point3D cameraPosition = new Point3D(-5.6, 9.6, 3.0);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }


   private FootstepDataListMessage createFootstepsForWalkingToTheRocks(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{-0.1332474847934586, 2.17798187482979, 0.08940742138400676}, {-3.403089128803084E-18, -2.675953624308355E-18, 0.6245692503477591, 0.7809694305925413}},
            {{0.20298560218119716, 2.4713021792322385, 0.08104440170899034}, {-2.417350678894318E-18, -6.032683755089822E-18, 0.6245692503477591, 0.7809694305925413}},
            {{0.08702817020723454, 2.866518422349047, 0.09494061415642502}, {-2.949179890966763E-17, -2.818034069811306E-17, 0.6245692503477591, 0.7809694305925413}},
            {{0.3668262356162857, 3.175450945925054, 0.07612517468837078}, {0.012931347168178755, 0.03190096955164589, 0.6238040168753828, 0.7808224234307172}},
            {{0.16792368362988752, 3.2186278940060267, 0.07936405785551498}, {0.0024963384952205, 0.011305199060860416, 0.6244762285129228, 0.7809580019377401}}
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataListMessage createFootstepsForSteppingOntoTheRocks(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {
            {{0.5529662090543602, 3.6286983881838646, 0.21590956237843234}, {-0.022876958432406867, -1.6243179975814606E-4, 0.6241266770053053, 0.7809881621632351}},
            {{0.23889485139604242, 3.7707451973897086, 0.23208827774938992}, {0.15338009067097566, 0.022598335690374175, 0.6247807222043084, 0.7652534953671569}},
            {{0.49923112290573035, 4.061638018194549, 0.2279816197448486}, {0.016006125164714134, 7.02388640921526E-4, 0.6320497992718929, 0.7747621324301857}},
            {{0.25039979297513204, 4.182455867683593, 0.23109072043393983}, {-0.011488423864199179, 0.006006205458266799, 0.6320296588353294, 0.7748357580581879}},
            {{0.7403166726686088, 4.45052552107267, 0.2700214618943709}, {-0.024017719675227735, -0.032205010198184086, 0.6337204274025326, 0.7725182239614081}},
            {{0.40338011262136547, 4.565, 0.17174456010388917}, {-0.05084225980296802, 0.01004423054718689, 0.6346904846256795, 0.7710266965394017}}
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

}
