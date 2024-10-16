package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.io.InputStream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;

public abstract class DRCObstacleCourseTrialsWalkingTaskTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
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
   public void testStepOnCinderBlocks()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      String scriptName = "scripts/ExerciseAndJUnitScripts/TwoCinderBlocksStepOn_LeftFootTest.xml";

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS;

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      simulationTestHelper.simulateNow(0.01);

      FramePoint3D pelvisHeight = new FramePoint3D(fullRobotModel.getRootJoint().getFrameAfterJoint());
      pelvisHeight.changeFrame(ReferenceFrame.getWorldFrame());
      PelvisHeightTrajectoryMessage message = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, pelvisHeight.getZ() + 0.05);
      simulationTestHelper.publishToController(message);

      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      simulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));

      setupCameraForWalkingOverCinderBlocks();

      ThreadTools.sleep(0);
      boolean success = simulationTestHelper.simulateNow(0.5);

      success = success && simulationTestHelper.simulateNow(9.5);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(13.10268850797296, 14.090724695197087, 1.146368436759061);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   // We don't need step on/off two layer CinderBlocks anymore
   //Note: this test will fail because of bounding box that needs to be "tuned"
   @Test
   public void testStepOnAndOffCinderBlocks()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS;

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      setupCameraForWalkingOverCinderBlocks();

      simulationTestHelper.simulateNow(0.01);

      //      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      //      simulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));

      ThreadTools.sleep(0);
      boolean success = simulationTestHelper.simulateNow(0.5);

      //      YoBoolean rightDoToeTouchdownIfPossible = (YoBoolean) simulationTestHelper.findVariable("rightFootSwingDoToeTouchdownIfPossible");
      YoBoolean doToeOffIfPossibleInSingleSupport = (YoBoolean) simulationTestHelper.findVariable("doToeOffIfPossibleInSingleSupport");
      //      rightDoToeTouchdownIfPossible.set(true);
      doToeOffIfPossibleInSingleSupport.set(true);

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      FootstepDataListMessage footstepDataListMessage2 = new FootstepDataListMessage();

      ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(RobotSide.LEFT);
      FramePoint3D step1Location = new FramePoint3D(soleFrame, 0.271343, -0.15565, 0.0);
      FramePoint3D step2Location = new FramePoint3D(soleFrame, 0.565873, 0.09695, 0.0);
      FramePoint3D step3Location = new FramePoint3D(soleFrame, 0.875617, -0.18381, 0.3);
      FramePoint3D step4Location = new FramePoint3D(soleFrame, 1.154142, 0.24624, 0.3);
      FramePoint3D step5Location = new FramePoint3D(soleFrame, 1.356672, -0.05836, 0.0);
      FramePoint3D step6Location = new FramePoint3D(soleFrame, 1.741226, 0.14575, 0.0);
      FramePoint3D step7Location = new FramePoint3D(soleFrame, 1.748862, -0.12301, 0.0);

      step1Location.changeFrame(worldFrame);
      step2Location.changeFrame(worldFrame);
      step3Location.changeFrame(worldFrame);
      step4Location.changeFrame(worldFrame);
      step5Location.changeFrame(worldFrame);
      step6Location.changeFrame(worldFrame);
      step7Location.changeFrame(worldFrame);

      FrameQuaternion zeroRotation = new FrameQuaternion(soleFrame);
      zeroRotation.changeFrame(worldFrame);

      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, step1Location, zeroRotation));
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, step2Location, zeroRotation));
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, step3Location, zeroRotation));
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, step4Location, zeroRotation));

      footstepDataListMessage2.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, step5Location, zeroRotation));
      footstepDataListMessage2.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, step6Location, zeroRotation));
      footstepDataListMessage2.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, step7Location, zeroRotation));

      simulationTestHelper.publishToController(footstepDataListMessage);

      success = success && simulationTestHelper.simulateNow(6.0);

      simulationTestHelper.publishToController(footstepDataListMessage2);
      success = success && simulationTestHelper.simulateNow(4.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D();
      center.interpolate(step6Location, step7Location, 0.5);
      center.addZ(0.9);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testStepOnCinderBlocksSlowlyWithDisturbance()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      String scriptName = "scripts/ExerciseAndJUnitScripts/TwoCinderBlocksStepOn_LeftFootTest_slow.xml";

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS;

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      setupCameraForWalkingOverCinderBlocks();

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      simulationTestHelper.simulateNow(1.0);
      FramePoint3D pelvisHeight = new FramePoint3D(fullRobotModel.getRootJoint().getFrameAfterJoint());
      pelvisHeight.changeFrame(ReferenceFrame.getWorldFrame());
      PelvisHeightTrajectoryMessage message = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, pelvisHeight.getZ() + 0.1);
      simulationTestHelper.publishToController(message);
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      simulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));

      ThreadTools.sleep(0);
      assertTrue(simulationTestHelper.simulateNow(6.0));

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      Point3D center = new Point3D(13.10268850797296, 14.090724695197087, 1.146368436759061);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForWalkingOverCinderBlocks()
   {
      Point3D cameraFix = new Point3D(13.5, 13.0, 0.75);
      Point3D cameraPosition = new Point3D(7.0, 17.0, 2.0);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }
}
