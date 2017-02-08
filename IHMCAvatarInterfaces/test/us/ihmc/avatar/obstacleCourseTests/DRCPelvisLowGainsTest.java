package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationDoneCriterion;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

/**
 * This end to end test is to make sure the pelvis doesn't flip out when it has low gains. In March, 2015 we started investigating this bug.
 * It seems to have something to do with either some sort of problem in the MomentumBasedController or InverseDynamicsCalculator or State Estimator.
 *
 */
public abstract class DRCPelvisLowGainsTest implements MultiRobotTestInterface
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

   public abstract InverseDynamicsCalculatorListener getInverseDynamicsCalculatorListener(FullRobotModel fullRobotModel, FloatingRootJointRobot sdfRobot);

   // 150313: This test currently fails, seemingly due to some sort of problem in the MomentumBasedController or InverseDynamicsCalculator. Trying to fix it...
	@ContinuousIntegrationTest(estimatedDuration = 38.0)
   @Test(timeout = 190000)
   public void testStandingWithLowPelvisOrientationGains() throws SimulationExceededMaximumTimeException
   {
      // March 2015: Low pelvis orientation gains cause the pelvis to flip out. Trying to track down why this happens.

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // Use perfect sensors and run single threaded to make sure state estimation isn't what's causing the problem.
      simulationTestingParameters.setUsePefectSensors(true);
      simulationTestingParameters.setRunMultiThreaded(false);

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      DRCRobotModel robotModel = getRobotModel();

      // Disable joint damping to make sure that damping isn't causing the problem.
      robotModel.setEnableJointDamping(false);
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCPelvisFlippingOutBugTest", selectedLocation, simulationTestingParameters,
              robotModel);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();

      // This is used for doing extra crazy debugging by spawing a whole new visualizer or analyzer to spy on the inverse dynamics calculator.
      // But is not needed for the test itself. So return null if you aren't debugging.
      InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener = getInverseDynamicsCalculatorListener(drcSimulationTestHelper.getControllerFullRobotModel(), drcSimulationTestHelper.getRobot());
      if (inverseDynamicsCalculatorListener != null) drcSimulationTestHelper.setInverseDynamicsCalculatorListener(inverseDynamicsCalculatorListener);

      setupCameraForElvisPelvis();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      final DoubleYoVariable pelvisOrientationError = getPelvisOrientationErrorVariableName(simulationConstructionSet);

      SimulationDoneCriterion checkPelvisOrientationError = new SimulationDoneCriterion()
      {
         @Override
         public boolean isSimulationDone()
         {
            return (Math.abs(pelvisOrientationError.getDoubleValue()) > 0.22);
         }
      };

      simulationConstructionSet.setSimulateDoneCriterion(checkPelvisOrientationError);

      DoubleYoVariable kpPelvisOrientation = (DoubleYoVariable) simulationConstructionSet.getVariable(getKpPelvisOrientationName());
      DoubleYoVariable zetaPelvisOrientation = (DoubleYoVariable) simulationConstructionSet.getVariable(getZetaPelvisOrientationName());

      // kp = 20.0, zeta = 0.7 causes problems when running multithreaded. kp = 1.0, zeta = 0.7 causes problems when running single threaded.
      kpPelvisOrientation.set(1.0);
      zetaPelvisOrientation.set(0.7);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(12.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3d center = new Point3d(-0.09807959403314585, 0.002501752329158081, 0.7867972043876718);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public abstract String getZetaPelvisOrientationName();
   public abstract String getKpPelvisOrientationName();

   private void setupCameraForElvisPelvis()
   {
      Point3d cameraFix = new Point3d(0.0, 0.0, 0.9);
      Point3d cameraPosition = new Point3d(0.0, -1.8, 0.9);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   protected abstract DoubleYoVariable getPelvisOrientationErrorVariableName(SimulationConstructionSet scs);

}
