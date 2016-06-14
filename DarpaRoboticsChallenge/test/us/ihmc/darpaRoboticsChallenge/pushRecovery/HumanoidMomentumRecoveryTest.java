package us.ihmc.darpaRoboticsChallenge.pushRecovery;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.DRCStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.FlatGroundEnvironment;
import us.ihmc.darpaRoboticsChallenge.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;

public abstract class HumanoidMomentumRecoveryTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private OffsetAndYawRobotInitialSetup location = new OffsetAndYawRobotInitialSetup(new Vector3d(0.0, 0.0, 0.0), 0.0);
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private BooleanYoVariable allowUpperBodyMomentumInSingleSupport;
   private BooleanYoVariable allowUpperBodyMomentumInDoubleSupport;
   private BooleanYoVariable allowUsingHighMomentumWeight;

   private DoubleYoVariable swingTime;

   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   /**
    * End to end test that makes sure the robot can recover from a push using upper body momentum
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testPushDuringDoubleSupport() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      Point3d cameraFix = new Point3d(0.0, 0.0, 1.0);
      Point3d cameraPosition = new Point3d(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);

      allowUpperBodyMomentumInSingleSupport.set(true);
      allowUpperBodyMomentumInDoubleSupport.set(true);
      allowUsingHighMomentumWeight.set(true);

      assertTrue(standAndPush());
   }

   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   /**
    * End to end test that makes sure the robot falls during test if momentum is disabled
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testPushDuringDoubleSupportExpectFall() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      Point3d cameraFix = new Point3d(0.0, 0.0, 1.0);
      Point3d cameraPosition = new Point3d(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);

      allowUpperBodyMomentumInSingleSupport.set(false);
      allowUpperBodyMomentumInDoubleSupport.set(false);
      allowUsingHighMomentumWeight.set(false);

      assertFalse(standAndPush());
   }

   private boolean standAndPush() throws SimulationExceededMaximumTimeException
   {
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push the robot
      Vector3d rootVelocity = new Vector3d();
      FloatingJoint rootJoint = drcSimulationTestHelper.getRobot().getRootJoint();
      rootJoint.getVelocity(rootVelocity);
      double push = 0.26;
      rootVelocity.x = rootVelocity.x + push;
      rootJoint.setVelocity(rootVelocity);

      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);
   }

   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   /**
    * End to end test that makes sure the robot can recover from a push using upper body momentum
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testPushDuringSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      Point3d cameraFix = new Point3d(0.0, 0.0, 1.0);
      Point3d cameraPosition = new Point3d(-10.0, 0.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);

      allowUpperBodyMomentumInSingleSupport.set(true);
      allowUpperBodyMomentumInDoubleSupport.set(true);
      allowUsingHighMomentumWeight.set(true);

      assertTrue(stepAndPush());
   }

   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   /**
    * End to end test that makes sure the robot falls during test if momentum is disabled
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testPushDuringSwingExpectFall() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      Point3d cameraFix = new Point3d(0.0, 0.0, 1.0);
      Point3d cameraPosition = new Point3d(-10.0, 0.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);

      allowUpperBodyMomentumInSingleSupport.set(false);
      allowUpperBodyMomentumInDoubleSupport.set(false);
      allowUsingHighMomentumWeight.set(false);

      assertFalse(stepAndPush());
   }

   private boolean stepAndPush() throws SimulationExceededMaximumTimeException
   {
      swingTime.set(3.0);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage footstepData = new FootstepDataMessage();
      RobotSide stepSide = RobotSide.LEFT;

      ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(stepSide);
      FramePoint placeToStepInWorld = new FramePoint(soleFrame, 0.3, 0.0, 0.0);
      placeToStepInWorld.changeFrame(worldFrame);

      footstepData.setLocation(placeToStepInWorld.getPointCopy());
      footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(stepSide);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      message.add(footstepData);

      drcSimulationTestHelper.send(message);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0));

      // push the robot
      Vector3d rootVelocity = new Vector3d();
      FloatingJoint rootJoint = drcSimulationTestHelper.getRobot().getRootJoint();
      rootJoint.getVelocity(rootVelocity);
      double push = -0.15;
      rootVelocity.y = rootVelocity.y + push;
      rootJoint.setVelocity(rootVelocity);

      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);
   }

   private void setupTest()
   {
      BambooTools.reportTestStartedMessage();

      // create simulation test helper
      FlatGroundEnvironment emptyEnvironment = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();
      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            return location;
         }
      };
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(emptyEnvironment, className, startingLocation, simulationTestingParameters, robotModel);

      allowUpperBodyMomentumInSingleSupport = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("allowUpperBodyMomentumInSingleSupport");
      allowUpperBodyMomentumInDoubleSupport = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("allowUpperBodyMomentumInDoubleSupport");
      allowUsingHighMomentumWeight = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("allowUsingHighMomentumWeight");

      swingTime = (DoubleYoVariable) drcSimulationTestHelper.getYoVariable("swingTime");

      ThreadTools.sleep(1000);
   }

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

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
