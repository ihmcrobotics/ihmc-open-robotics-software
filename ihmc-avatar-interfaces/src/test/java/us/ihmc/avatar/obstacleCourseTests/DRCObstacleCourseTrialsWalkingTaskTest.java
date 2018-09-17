package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import java.io.InputStream;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.junit.After;
import org.junit.Before;

import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import org.junit.Test;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.dataBuffer.DataProcessingFunction;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class DRCObstacleCourseTrialsWalkingTaskTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private static final boolean MOVE_ROBOT_FOR_VIZ = false;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
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

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 64.7)
   @Test(timeout = 320000)
   public void testStepOnCinderBlocks() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      String scriptName = "scripts/ExerciseAndJUnitScripts/TwoCinderBlocksStepOn_LeftFootTest.xml";

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCObstacleCourseTrialsCinderBlocksTest");
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.01);

      FramePoint3D pelvisHeight = new FramePoint3D(fullRobotModel.getRootJoint().getFrameAfterJoint());
      pelvisHeight.changeFrame(ReferenceFrame.getWorldFrame());
      PelvisHeightTrajectoryMessage message = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, pelvisHeight.getZ() + 0.05);
      drcSimulationTestHelper.publishToController(message);

      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      drcSimulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));

      setupCameraForWalkingOverCinderBlocks();

      ThreadTools.sleep(0);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(9.5);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(13.10268850797296, 14.090724695197087, 1.146368436759061);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   // We don't need step on/off two layer CinderBlocks anymore
   //Note: this test will fail because of bounding box that needs to be "tuned"
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 100.6)
   @Test(timeout = 500000)
   public void testStepOnAndOffCinderBlocks() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      String scriptName = "scripts/ExerciseAndJUnitScripts/TwoCinderBlocksStepOver_LeftFootTest.xml";


      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCObstacleCourseTrialsCinderBlocksTest");
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      setupCameraForWalkingOverCinderBlocks();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.01);

      //      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      //      drcSimulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));


      ThreadTools.sleep(0);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      YoBoolean leftDoToeTouchdownIfPossible = (YoBoolean) simulationConstructionSet.getVariable("leftFootSwingDoToeTouchdownIfPossible");
      YoBoolean rightDoToeTouchdownIfPossible = (YoBoolean) simulationConstructionSet.getVariable("rightFootSwingDoToeTouchdownIfPossible");
      YoBoolean doToeOffIfPossibleInSingleSupport = (YoBoolean) simulationConstructionSet.getVariable("doToeOffIfPossibleInSingleSupport");
      leftDoToeTouchdownIfPossible.set(true);
      rightDoToeTouchdownIfPossible.set(true);
      doToeOffIfPossibleInSingleSupport.set(true);


      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      FootstepDataListMessage footstepDataListMessage2 = new FootstepDataListMessage();

      ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(RobotSide.LEFT);
      FramePoint3D step1Location = new FramePoint3D(soleFrame, 0.271343,-0.15565, 0.0);
      FramePoint3D step2Location = new FramePoint3D(soleFrame, 0.565873, 0.09695, 0.0);
      FramePoint3D step3Location = new FramePoint3D(soleFrame, 0.875617,-0.18381, 0.3);
      FramePoint3D step4Location = new FramePoint3D(soleFrame, 1.154142, 0.24624, 0.3);
      FramePoint3D step5Location = new FramePoint3D(soleFrame, 1.356672,-0.05836, 0.0);
      FramePoint3D step6Location = new FramePoint3D(soleFrame, 1.741226, 0.14575, 0.0);
      FramePoint3D step7Location = new FramePoint3D(soleFrame, 1.748862,-0.12301, 0.0);

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

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);

      drcSimulationTestHelper.publishToController(footstepDataListMessage2);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D();
      center.interpolate(step6Location, step7Location, 0.5);
      center.addZ(0.9);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 100.6)
   @Test(timeout = 500000)
   public void testStepOnCinderBlocksSlowlyWithDisturbance() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      String scriptName = "scripts/ExerciseAndJUnitScripts/TwoCinderBlocksStepOn_LeftFootTest_slow.xml";

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCObstacleCourseTrialsCinderBlocksTest");
      setupCameraForWalkingOverCinderBlocks();

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      FramePoint3D pelvisHeight = new FramePoint3D(fullRobotModel.getRootJoint().getFrameAfterJoint());
      pelvisHeight.changeFrame(ReferenceFrame.getWorldFrame());
      PelvisHeightTrajectoryMessage message = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, pelvisHeight.getZ() + 0.1);
      drcSimulationTestHelper.publishToController(message);
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      drcSimulationTestHelper.loadScriptFile(scriptInputStream, fullRobotModel.getSoleFrame(RobotSide.LEFT));

      ThreadTools.sleep(0);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.5);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(14.5);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(13.10268850797296, 14.090724695197087, 1.146368436759061);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      if (MOVE_ROBOT_FOR_VIZ)
         moveRobotOutOfWayForViz();

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void moveRobotOutOfWayForViz()
   {
      final SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      DataProcessingFunction dataProcessingFunction = new DataProcessingFunction()
      {
         private final YoDouble q_y = (YoDouble) scs.getVariable("q_y");

         @Override
         public void initializeProcessing()
         {
         }

         @Override
         public void processData()
         {
            q_y.sub(4.0);
         }
      };

      scs.applyDataProcessingFunction(dataProcessingFunction);
   }

   private void setupCameraForWalkingOverCinderBlocks()
   {
      Point3D cameraFix = new Point3D(13.5, 13.0, 0.75);
      Point3D cameraPosition = new Point3D(7.0, 17.0, 2.0);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
}
