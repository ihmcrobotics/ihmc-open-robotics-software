package us.ihmc.avatar.obstacleCourseTests;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

import static us.ihmc.robotics.Assert.assertTrue;

public abstract class AvatarBigStepDownTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
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


   @Test
   public void testWalkingOffOfLargePlatform() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ON_LARGE_PLATFORM;

      System.out.println(selectedLocation.getStartingLocationOffset().getAdditionalOffset());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCWalkingOffOfLargePlatformTest");

      Point3D cameraFix = new Point3D(-4.68, -7.8, 0.55);
      Point3D cameraPosition = new Point3D(-8.6, -4.47, 0.58);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      Quaternion footRotation = new Quaternion();
      footRotation.setToYawOrientation(selectedLocation.getStartingLocationOffset().getYaw());

      FootstepDataMessage firstStep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT,
                                                                                     new Point3D(-5.8 + 0.15, -7.471 - 0.15, 0.05),
                                                                                     footRotation);
      FootstepDataMessage secondStep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT,
                                                                                      new Point3D(-5.8 - 0.15, -7.471 + 0.15, 0.05),
                                                                                      footRotation);

      drcSimulationTestHelper.publishToController(HumanoidMessageTools.createFootstepDataListMessage(firstStep));

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      drcSimulationTestHelper.publishToController(HumanoidMessageTools.createFootstepDataListMessage(secondStep));

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);


      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-5.8, -7.5, 0.87);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.2);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


}
