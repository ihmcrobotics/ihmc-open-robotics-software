package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class DRCObstacleCourseSteppingStonesTest implements MultiRobotTestInterface
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

   @Test
   public void testWalkingOverEasySteppingStones()
   {
      try
      {
         CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

         DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.EASY_STEPPING_STONES;

         SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                                simulationTestingParameters);
         simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
         simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
         simulationTestHelper.start();

         setupCameraForWalkingOverEasySteppingStones();

         ThreadTools.sleep(1000);
         boolean success = simulationTestHelper.simulateNow(2.0);

         FootstepDataListMessage footstepDataList = createFootstepsForWalkingOverEasySteppingStones();
         simulationTestHelper.publishToController(footstepDataList);

         success = success && simulationTestHelper.simulateNow(13.0);

         // TODO GITHUB WORKFLOWS
//         simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
         //         simulationTestHelper.checkNothingChanged();

         assertTrue(success);

         Point3D center = new Point3D(-10.241987629532595, -0.8330256660954483, 1.0893768421917251);
         Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
         BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
         simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

         CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      }
      catch (Throwable throwable)
      {
         System.err.println("Caught throwable in testWalkingOverEasySteppingStones: " + throwable);
         System.err.flush();
         throw throwable;
      }
   }

   private void setupCameraForWalkingOverEasySteppingStones()
   {
      Point3D cameraFix = new Point3D(-8.6, -0.1, 0.94);
      Point3D cameraPosition = new Point3D(-14.0, -5.0, 2.7);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private FootstepDataListMessage createFootstepsForWalkingOverEasySteppingStones()
   {
      Pose3D[] footstepPoses = {
            new Pose3D(new Point3D(-7.72847174992541, -0.5619736174919732, 0.3839138258635628),
                       new Quaternion(-0.002564106649548222, 9.218543591576633E-4, 0.9999871158757672, 0.004282945726398341)),
            new Pose3D(new Point3D(-8.233931300168681, -0.952122284180518, 0.3841921077973934),
                       new Quaternion(-2.649132161393031E-6, -0.00302400231893713, 0.999986265693845, 0.004280633905867881)),
            new Pose3D(new Point3D(-8.711157422190857, -0.5634436272430561, 0.38340964898482055),
                       new Quaternion(-6.333967334144636E-4, -0.002689012266100874, 0.9999870292977306, 0.004278931865605645)),
            new Pose3D(new Point3D(-9.246614388340875, -0.9823725639340232, 0.3838760717826556),
                       new Quaternion(4.990380502353344E-4, 0.002867206806117212, 0.9999866091454905, 0.00427920738681889)),
            new Pose3D(new Point3D(-9.694460236661355, -0.5363354293129117, 0.3828438933446154),
                       new Quaternion(0.0043663633816866795, 6.575433167622114E-4, 0.9999811020260976, 0.004277627645902338)),
            new Pose3D(new Point3D(-10.204483462540168, -1.0007498263499959, 0.3841142603691748),
                       new Quaternion(3.379337850421112E-4, 0.0013510800402890615, 0.9999898702179759, 0.004280168795429233)),
            new Pose3D(new Point3D(-10.20677294790819, -0.6741336761434962, 0.3829201197142793),
                       new Quaternion(0.004772284224629501, 0.005592011887113724, 0.9999639290557834, 0.004253856327364576))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.RIGHT, footstepPoses);
   }

}
