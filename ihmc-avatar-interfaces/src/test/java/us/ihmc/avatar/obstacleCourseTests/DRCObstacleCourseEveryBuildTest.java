package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.io.InputStream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class DRCObstacleCourseEveryBuildTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters;

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
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

      simulationTestingParameters = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testSimpleFlatGroundScript()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      String scriptName = "scripts/ExerciseAndJUnitScripts/SimpleFlatGroundScript.xml";

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatGround, simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.simulateNow(0.001);
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      simulationTestHelper.loadScriptFile(scriptInputStream, ReferenceFrame.getWorldFrame());

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(20.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
//      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.121, -0.092, 0.7102);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingUpToRampWithLongSteps()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatGround, simulationTestingParameters);
      simulationTestHelper.start();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingOnFlatLongSteps();

      // FootstepDataList footstepDataList = createFootstepsForTwoLongFlatSteps();
      simulationTestHelper.publishToController(footstepDataList);

      success = success && simulationTestHelper.simulateNow(10.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
//      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(3.106182296217929, 0.019198891341144136, 0.7894546312193843);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForWalkingUpToRamp()
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private FootstepDataListMessage createFootstepsForWalkingOnFlatLongSteps()
   {
      Pose3D[] footstepPoses = {
            new Pose3D(new Point3D(0.5909646234016005, 0.10243127081250579, 0.08400000000000002),
                       new Quaternion(3.5805394102331502E-22, -1.0841962601668662E-19, 0.003302464707320093, 0.99999454684856)),
            new Pose3D(new Point3D(1.212701966120992, -0.09394691394679651, 0.084),
                       new Quaternion(1.0806157207566333E-19, 1.0877767995770995E-19, 0.0033024647073200924, 0.99999454684856)),
            new Pose3D(new Point3D(1.8317941784239657, 0.11014657591704705, 0.08619322927296164),
                       new Quaternion(8.190550851520344E-19, 1.5693991726842814E-18, 0.003302464707320093, 0.99999454684856)),
            new Pose3D(new Point3D(2.4535283480857237, -0.08575120920059497, 0.08069788195751608),
                       new Quaternion(-2.202407644730947E-19, -8.117149793610565E-19, 0.0033024647073200924, 0.99999454684856)),
            new Pose3D(new Point3D(3.073148474156348, 0.11833676240086898, 0.08590468550531082),
                       new Quaternion(4.322378465953267E-5, 0.003142233766871708, 0.0033022799833692306, 0.9999896096688056)),
            new Pose3D(new Point3D(3.0729346702590505, -0.0816428320664241, 0.0812390388356),
                       new Quaternion(-8.243740658642556E-5, -0.005993134849034999, 0.003301792738040525, 0.999976586577641))};

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.LEFT, footstepPoses);
   }

}
