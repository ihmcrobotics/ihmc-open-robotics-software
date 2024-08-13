package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.function.BooleanSupplier;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This end to end test is to make sure the pelvis doesn't flip out when it has low gains. In March,
 * 2015 we started investigating this bug. It seems to have something to do with either some sort of
 * problem in the HighLevelHumanoidControllerToolbox or InverseDynamicsCalculator or State
 * Estimator.
 */
public abstract class DRCPelvisLowGainsTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   // 150313: This test currently fails, seemingly due to some sort of problem in the HighLevelHumanoidControllerToolbox or InverseDynamicsCalculator. Trying to fix it...
   @Test
   public void testStandingWithLowPelvisOrientationGains()
   {
      // March 2015: Low pelvis orientation gains cause the pelvis to flip out. Trying to track down why this happens.

      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // Use perfect sensors and run single threaded to make sure state estimation isn't what's causing the problem.
      simulationTestingParameters.setUsePefectSensors(true);
      simulationTestingParameters.setRunMultiThreaded(false);

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();

      DRCRobotModel robotModel = getRobotModel();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel, flatGround, simulationTestingParameters);
      simulationTestHelper.start();

      // This is used for doing extra crazy debugging by spawing a whole new visualizer or analyzer to spy on the inverse dynamics calculator.
      // But is not needed for the test itself. So return null if you aren't debugging.

      setupCameraForElvisPelvis();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(3.0);

      final YoDouble pelvisOrientationError = getPelvisOrientationErrorVariableName(simulationTestHelper);

      BooleanSupplier checkPelvisOrientationError = new BooleanSupplier()
      {
         @Override
         public boolean getAsBoolean()
         {
            return (Math.abs(pelvisOrientationError.getDoubleValue()) > 0.22);
         }
      };

      simulationTestHelper.addSimulationTerminalCondition(checkPelvisOrientationError);

      String namespace = HighLevelControlManagerFactory.rigidBodyGainRegistryName;
      YoDouble kpPelvisOrientation = (YoDouble) simulationTestHelper.findVariable(namespace, "kpXYPelvisOrientation");
      YoDouble zetaPelvisOrientation = (YoDouble) simulationTestHelper.findVariable(namespace, "zetaXYPelvisOrientation");

      // kp = 20.0, zeta = 0.7 causes problems when running multithreaded. kp = 1.0, zeta = 0.7 causes problems when running single threaded.
      kpPelvisOrientation.set(1.0);
      zetaPelvisOrientation.set(0.7);

      success = success && simulationTestHelper.simulateNow(12.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-0.09807959403314585, 0.002501752329158081, 0.7867972043876718);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForElvisPelvis()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 0.9);
      Point3D cameraPosition = new Point3D(0.0, -1.8, 0.9);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   protected abstract YoDouble getPelvisOrientationErrorVariableName(YoVariableHolder yoVariableHolder);

}
