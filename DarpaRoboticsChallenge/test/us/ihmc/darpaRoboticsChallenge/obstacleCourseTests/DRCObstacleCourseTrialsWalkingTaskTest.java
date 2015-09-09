package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.simulationconstructionset.DataProcessingFunction;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCObstacleCourseTrialsWalkingTaskTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private static final boolean MOVE_ROBOT_FOR_VIZ = false;
   
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


	@DeployableTestMethod(estimatedDuration = 39.7)
	@Test(timeout = 200000)
   public void testStepOnCinderBlocks() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      String scriptName = "scripts/ExerciseAndJUnitScripts/TwoCinderBlocksStepOn_LeftFootTest.xml";

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCObstacleCourseTrialsCinderBlocksTest", scriptName, selectedLocation, simulationTestingParameters, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();

      setupCameraForWalkingOverCinderBlocks(simulationConstructionSet);

      ThreadTools.sleep(0);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(9.5);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3d center = new Point3d(13.10268850797296, 14.090724695197087, 1.146368436759061);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage();
   }
	
   
	@DeployableTestMethod(estimatedDuration = 50.0)
   @Test(timeout = 250000)
   public void testStepOnCinderBlocksSlowlyWithDisturbance() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      String scriptName = "scripts/ExerciseAndJUnitScripts/TwoCinderBlocksStepOn_LeftFootTest.xml";

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCObstacleCourseTrialsCinderBlocksTest", scriptName, selectedLocation, simulationTestingParameters, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      DoubleYoVariable swingTime = (DoubleYoVariable) simulationConstructionSet.getVariable("swingTime");
      DoubleYoVariable transferTime = (DoubleYoVariable) simulationConstructionSet.getVariable("transferTime");
      
      setupCameraForWalkingOverCinderBlocks(simulationConstructionSet);

      ThreadTools.sleep(0);

      swingTime.set(2.0);
      transferTime.set(2.0);
      
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.5);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(14.5);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3d center = new Point3d(13.10268850797296, 14.090724695197087, 1.146368436759061);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      if (MOVE_ROBOT_FOR_VIZ) moveRobotOutOfWayForViz();
      
      BambooTools.reportTestFinishedMessage();
   }

   private void moveRobotOutOfWayForViz()
   {
      final SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      DataProcessingFunction dataProcessingFunction = new DataProcessingFunction()
      {
         private final DoubleYoVariable q_y = (DoubleYoVariable) scs.getVariable("q_y");
         
         @Override
         public void initializeProcessing()
         {   
         }

         @Override
         public void processData()
         {
            q_y.sub(4.0);
         }};
      
      scs.applyDataProcessingFunction(dataProcessingFunction);
   }

   // @Test(timeout=300000), we don't need step on/off two layer CinderBlocks anymore
   //Note: this test will fail because of bounding box that needs to be "tuned"
   public void testStepOnAndOffCinderBlocks() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      String scriptName = "scripts/ExerciseAndJUnitScripts/TwoCinderBlocksStepOver_LeftFootTest.xml";

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCObstacleCourseTrialsCinderBlocksTest", scriptName, selectedLocation, simulationTestingParameters, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();

      setupCameraForWalkingOverCinderBlocks(simulationConstructionSet);

      ThreadTools.sleep(0);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);

      BooleanYoVariable doToeTouchdownIfPossible = (BooleanYoVariable) simulationConstructionSet.getVariable("doToeTouchdownIfPossible");
      doToeTouchdownIfPossible.set(true);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(13.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3d center = new Point3d();
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage();
   }

   private void setupCameraForWalkingOverCinderBlocks(SimulationConstructionSet scs)
   {
      Point3d cameraFix = new Point3d(13.2664, 13.03, 0.75);
      Point3d cameraPosition = new Point3d(9.50, 15.59, 1.87);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
}
