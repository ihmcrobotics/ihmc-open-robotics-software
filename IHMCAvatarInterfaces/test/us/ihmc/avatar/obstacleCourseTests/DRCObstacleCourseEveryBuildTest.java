package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.*;

import java.io.InputStream;

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
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCObstacleCourseEveryBuildTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters;
   
   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
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
      
      simulationTestingParameters = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


	@ContinuousIntegrationTest(estimatedDuration = 52.7)
	@Test(timeout = 260000)
   public void testSimpleFlatGroundScript() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      String scriptName = "scripts/ExerciseAndJUnitScripts/SimpleFlatGroundScript.xml";

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCSimpleFlatGroundScriptTest", selectedLocation, simulationTestingParameters, getRobotModel());
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.001);
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      drcSimulationTestHelper.loadScriptFile(scriptInputStream, ReferenceFrame.getWorldFrame());
      
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      setupCameraForWalkingUpToRamp(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.121, -0.092, 0.7102);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);




      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@ContinuousIntegrationTest(estimatedDuration = 53.8)
	@Test(timeout = 270000)
   public void testWalkingUpToRampWithLongSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCWalkingUpToRampLongStepsTest", selectedLocation, simulationTestingParameters, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingOnFlatLongSteps(scriptedFootstepGenerator);

      // FootstepDataList footstepDataList = createFootstepsForTwoLongFlatSteps(scriptedFootstepGenerator);
      drcSimulationTestHelper.send(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(3.106182296217929, 0.019198891341144136, 0.7894546312193843);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);



      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }




   private void setupCameraForWalkingUpToRamp(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }



   private FootstepDataListMessage createFootstepsForWalkingOnFlatLongSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {0.5909646234016005, 0.10243127081250579, 0.08400000000000002},
            {3.5805394102331502E-22, -1.0841962601668662E-19, 0.003302464707320093, 0.99999454684856}
         },
         {
            {1.212701966120992, -0.09394691394679651, 0.084}, {1.0806157207566333E-19, 1.0877767995770995E-19, 0.0033024647073200924, 0.99999454684856}
         },
         {
            {1.8317941784239657, 0.11014657591704705, 0.08619322927296164},
            {8.190550851520344E-19, 1.5693991726842814E-18, 0.003302464707320093, 0.99999454684856}
         },
         {
            {2.4535283480857237, -0.08575120920059497, 0.08069788195751608},
            {-2.202407644730947E-19, -8.117149793610565E-19, 0.0033024647073200924, 0.99999454684856}
         },
         {
            {3.073148474156348, 0.11833676240086898, 0.08590468550531082},
            {4.322378465953267E-5, 0.003142233766871708, 0.0033022799833692306, 0.9999896096688056}
         },
         {
            {3.0729346702590505, -0.0816428320664241, 0.0812390388356}, {-8.243740658642556E-5, -0.005993134849034999, 0.003301792738040525, 0.999976586577641}
         }
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }


}
