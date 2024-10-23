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
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class DRCObstacleCourseRocksTest implements MultiRobotTestInterface
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

   public void setupTestingParameters(SimulationTestingParameters parameters)
   {

   }

   @Test
   public void testWalkingOntoRocks()
   {
      setupTestingParameters(simulationTestingParameters);
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ROCKS;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOntoRocks();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingToTheRocks();
      simulationTestHelper.publishToController(footstepDataList);

      success = success && simulationTestHelper.simulateNow(6.5);

      if (success)
      {
         footstepDataList = createFootstepsForSteppingOntoTheRocks();
         simulationTestHelper.publishToController(footstepDataList);

         success = success && simulationTestHelper.simulateNow(6.0);
      }

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      assertTrue("Caught Exception: " + simulationTestHelper.getLastThrownException(), success);

      Point3D center = new Point3D(0.6853965087476173, 4.5173529666394305, 0.8898586980716016);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForWalkingOntoRocks()
   {
      Point3D cameraFix = new Point3D(0.1, 3.2, 0.5);
      Point3D cameraPosition = new Point3D(-5.6, 9.6, 3.0);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private FootstepDataListMessage createFootstepsForWalkingToTheRocks()
   {
      Pose3D[] footstepPoses = {
            new Pose3D(new Point3D(-0.1332474847934586, 2.17798187482979, 0.08940742138400676),
                       new Quaternion(-3.403089128803084E-18, -2.675953624308355E-18, 0.6245692503477591, 0.7809694305925413)),
            new Pose3D(new Point3D(0.20298560218119716, 2.4713021792322385, 0.08104440170899034),
                       new Quaternion(-2.417350678894318E-18, -6.032683755089822E-18, 0.6245692503477591, 0.7809694305925413)),
            new Pose3D(new Point3D(0.08702817020723454, 2.866518422349047, 0.09494061415642502),
                       new Quaternion(-2.949179890966763E-17, -2.818034069811306E-17, 0.6245692503477591, 0.7809694305925413)),
            new Pose3D(new Point3D(0.3668262356162857, 3.175450945925054, 0.07612517468837078),
                       new Quaternion(0.012931347168178755, 0.03190096955164589, 0.6238040168753828, 0.7808224234307172)),
            new Pose3D(new Point3D(0.16792368362988752, 3.2186278940060267, 0.07936405785551498),
                       new Quaternion(0.0024963384952205, 0.011305199060860416, 0.6244762285129228, 0.7809580019377401))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.LEFT, footstepPoses);
   }

   private FootstepDataListMessage createFootstepsForSteppingOntoTheRocks()
   {
      Pose3D[] footstepPoses = {
            new Pose3D(new Point3D(0.5529662090543602, 3.6286983881838646, 0.21590956237843234),
                       new Quaternion(-0.022876958432406867, -1.6243179975814606E-4, 0.6241266770053053, 0.7809881621632351)),
            new Pose3D(new Point3D(0.23889485139604242, 3.7707451973897086, 0.23208827774938992),
                       new Quaternion(0.15338009067097566, 0.022598335690374175, 0.6247807222043084, 0.7652534953671569)),
            new Pose3D(new Point3D(0.49923112290573035, 4.061638018194549, 0.2279816197448486),
                       new Quaternion(0.016006125164714134, 7.02388640921526E-4, 0.6320497992718929, 0.7747621324301857)),
            new Pose3D(new Point3D(0.25039979297513204, 4.182455867683593, 0.23109072043393983),
                       new Quaternion(-0.011488423864199179, 0.006006205458266799, 0.6320296588353294, 0.7748357580581879)),
            new Pose3D(new Point3D(0.7403166726686088, 4.45052552107267, 0.2700214618943709),
                       new Quaternion(-0.024017719675227735, -0.032205010198184086, 0.6337204274025326, 0.7725182239614081)),
            new Pose3D(new Point3D(0.40338011262136547, 4.565, 0.17174456010388917),
                       new Quaternion(-0.05084225980296802, 0.01004423054718689, 0.6346904846256795, 0.7710266965394017))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.RIGHT, footstepPoses);
   }

}
