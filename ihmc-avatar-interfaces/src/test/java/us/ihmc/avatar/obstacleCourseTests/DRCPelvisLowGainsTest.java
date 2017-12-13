package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationDoneCriterion;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This end to end test is to make sure the pelvis doesn't flip out when it has low gains. In March, 2015 we started investigating this bug.
 * It seems to have something to do with either some sort of problem in the HighLevelHumanoidControllerToolbox or InverseDynamicsCalculator or State Estimator.
 *
 */
public abstract class DRCPelvisLowGainsTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

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

   // 150313: This test currently fails, seemingly due to some sort of problem in the HighLevelHumanoidControllerToolbox or InverseDynamicsCalculator. Trying to fix it...
   public void testStandingWithLowPelvisOrientationGains() throws SimulationExceededMaximumTimeException
   {
      // March 2015: Low pelvis orientation gains cause the pelvis to flip out. Trying to track down why this happens.

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // Use perfect sensors and run single threaded to make sure state estimation isn't what's causing the problem.
      simulationTestingParameters.setUsePefectSensors(true);
      simulationTestingParameters.setRunMultiThreaded(false);

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, flatGround);
      drcSimulationTestHelper.createSimulation("DRCPelvisFlippingOutBugTest");

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();

      // This is used for doing extra crazy debugging by spawing a whole new visualizer or analyzer to spy on the inverse dynamics calculator.
      // But is not needed for the test itself. So return null if you aren't debugging.
      InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener = getInverseDynamicsCalculatorListener(drcSimulationTestHelper.getControllerFullRobotModel(), drcSimulationTestHelper.getRobot());
      if (inverseDynamicsCalculatorListener != null) drcSimulationTestHelper.setInverseDynamicsCalculatorListener(inverseDynamicsCalculatorListener);

      setupCameraForElvisPelvis();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      final YoDouble pelvisOrientationError = getPelvisOrientationErrorVariableName(simulationConstructionSet);

      SimulationDoneCriterion checkPelvisOrientationError = new SimulationDoneCriterion()
      {
         @Override
         public boolean isSimulationDone()
         {
            return (Math.abs(pelvisOrientationError.getDoubleValue()) > 0.22);
         }
      };

      simulationConstructionSet.setSimulateDoneCriterion(checkPelvisOrientationError);

      String namespace = HighLevelControlManagerFactory.class.getSimpleName();
      YoDouble kpPelvisOrientation = (YoDouble) simulationConstructionSet.getVariable(namespace, "kpXYPelvisOrientation");
      YoDouble zetaPelvisOrientation = (YoDouble) simulationConstructionSet.getVariable(namespace, "zetaXYPelvisOrientation");

      // kp = 20.0, zeta = 0.7 causes problems when running multithreaded. kp = 1.0, zeta = 0.7 causes problems when running single threaded.
      kpPelvisOrientation.set(1.0);
      zetaPelvisOrientation.set(0.7);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(12.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-0.09807959403314585, 0.002501752329158081, 0.7867972043876718);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForElvisPelvis()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 0.9);
      Point3D cameraPosition = new Point3D(0.0, -1.8, 0.9);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   protected abstract YoDouble getPelvisOrientationErrorVariableName(SimulationConstructionSet scs);

}
