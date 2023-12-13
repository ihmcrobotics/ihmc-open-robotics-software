package us.ihmc.avatar.controllerAPI;

import controller_msgs.msg.dds.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.*;

public abstract class EndToEndFootstepQueueStatusMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();
   private static final DRCStartingLocation location = DRCObstacleCourseStartingLocation.DEFAULT;

   private SCS2AvatarTestingSimulation simulationTestHelper;

   private void createSimulationTestHelper(CommonAvatarEnvironmentInterface environment, DRCStartingLocation location)
   {
      SCS2AvatarTestingSimulationFactory factory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                         environment,
                                                                                                                         simulationTestingParameters);
      if (location != null)
         factory.setStartingLocationOffset(location.getStartingLocationOffset());
      simulationTestHelper = factory.createAvatarTestingSimulation();
   }

   @Test
   public void testTheFootstepQueueStatus() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = getRobotModel();
      createSimulationTestHelper(environment, location);
      simulationTestHelper.start();
      ThreadTools.sleep(1000);

      // do the initial simulation
      assertTrue(simulationTestHelper.simulateNow(0.25));

      CommonHumanoidReferenceFrames referenceFrames = simulationTestHelper.getControllerReferenceFrames();
      referenceFrames.updateFrames();
      MovingReferenceFrame midFeetFrame = referenceFrames.getMidFootZUpGroundFrame();

      double stepWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getInPlaceWidth();
      double halfStepWidth = stepWidth / 2.0;
      double stepLength = robotModel.getWalkingControllerParameters().getSteppingParameters().getDefaultStepLength() * 0.5;
      double nominalSwingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      double nominalTransferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      Random random = new Random(24384523737236643L);

      List<FootstepDataMessage> footstepList = new ArrayList<>();
      RobotSide stepSide = RobotSide.LEFT;
      int steps = 8;
      double totalTime = robotModel.getWalkingControllerParameters().getDefaultFinalTransferTime();

      for (int foostepIdx = 0; foostepIdx < steps; foostepIdx++)
      {
         // walk forward
         FramePoint3D frameLocation = new FramePoint3D(midFeetFrame);
         frameLocation.setX(stepLength * (foostepIdx + 1));
         frameLocation.setY(stepSide.negateIfRightSide(halfStepWidth));
         frameLocation.changeFrame(ReferenceFrame.getWorldFrame());
         FrameQuaternion frameOrientation = new FrameQuaternion(midFeetFrame);
         frameOrientation.changeFrame(ReferenceFrame.getWorldFrame());
         int id = RandomNumbers.nextInt(random, 1, 5000);
         FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(stepSide, frameLocation, frameOrientation);
         footstep.setSequenceId(id);

         // between 0.75 and 1.25 times the nominal time:
         double swingTime = (1.0 + 0.5 * (random.nextDouble() + 0.5)) * nominalSwingTime;
         double transferTime = (1.0 + 0.5 * (random.nextDouble() + 0.5)) * nominalTransferTime;
         footstep.setSwingDuration(swingTime);
         footstep.setTransferDuration(transferTime);

         footstepList.add(footstep);
         totalTime += swingTime + transferTime;
         stepSide = stepSide.getOppositeSide();
      }

      // set up the listener architecture.
      AtomicBoolean isInSwing = new AtomicBoolean(false);
      List<FootstepDataMessage> currentFootstepQueue = new ArrayList<>();
      FootstepQueueStatusMessage currentFootstepQueueStatus = new FootstepQueueStatusMessage();
      YoInteger numberOfStepsInController = ((YoInteger) simulationTestHelper.findVariable(WalkingMessageHandler.class.getSimpleName(), "currentNumberOfFootsteps"));

      StatusMessageOutputManager statusMessageOutputManager = simulationTestHelper.getHighLevelHumanoidControllerFactory().getStatusOutputManager();

      statusMessageOutputManager.attachStatusMessageListener(FootstepQueueStatusMessage.class, currentFootstepQueueStatus::set);
      statusMessageOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, (m) ->
      {
         if (m.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
         {
            isInSwing.set(false);
            currentFootstepQueue.remove(0);
         }
         else
         {
            isInSwing.set(true);
         }
      });

      double simDt = 0.05;

      // run the sim for a little bit, and make sure the published queue is empty
      for (int i = 0; i < 100; i++)
      {
         simulationTestHelper.simulateNow(simDt);
         assertEquals(0, currentFootstepQueueStatus.getQueuedFootstepList().size());
         assertEquals(0, numberOfStepsInController.getIntegerValue());
      }

      // publish some footsteps to the controller, and check that the queue is correct during execution.
      FootstepDataListMessage footstepListMessage = HumanoidMessageTools.createFootstepDataListMessage(footstepList.toArray(new FootstepDataMessage[0]));
      simulationTestHelper.publishToController(footstepListMessage);

      currentFootstepQueue.addAll(footstepList);

      simulationTestHelper.simulateNow(50);

      double simTime = 0.0;
      while (simTime <= totalTime)
      {
         simulationTestHelper.simulateNow(simDt);
         ThreadTools.sleep(50);

         // check that the Queue size is accurate
         int controllerSideQueueSize = numberOfStepsInController.getIntegerValue() + (isInSwing.get() ? 1 : 0);
         assertEquals(isInSwing.get(), currentFootstepQueueStatus.getIsFirstStepInSwing());
         assertEquals(controllerSideQueueSize, currentFootstepQueue.size(), "The actual footstep queue is wrong.");
         assertEquals(controllerSideQueueSize, currentFootstepQueueStatus.getQueuedFootstepList().size());

         for (int i =- 0; i < currentFootstepQueue.size(); i++)
         {
            FootstepDataMessage expectedStep = currentFootstepQueue.get(i);
            QueuedFootstepStatusMessage queuedStep = currentFootstepQueueStatus.getQueuedFootstepList().get(i);
            assertFootstepsEqual(expectedStep, queuedStep, 1e-5);
         }

         simTime += simDt;
      }

      LogTools.info("Finished, now check to make sure it went forward and all the steps are executed.");

      assertTrue(simulationTestHelper.simulateNow(0.25));
      assertEquals(0, (int) numberOfStepsInController.getValueAsLongBits());
   }

   private static  void assertFootstepsEqual(FootstepDataMessage expected, QueuedFootstepStatusMessage queuedStep, double epsilon)
   {
      assertEquals(expected.getSequenceId(), queuedStep.getSequenceId());
      EuclidCoreTestTools.assertEquals(expected.getLocation(), queuedStep.getLocation(), epsilon);
      EuclidCoreTestTools.assertEquals(expected.getOrientation(), queuedStep.getOrientation(), epsilon);
      assertEquals(expected.getSwingDuration(), queuedStep.getSwingDuration(), epsilon);
      assertEquals(expected.getTransferDuration(), queuedStep.getTransferDuration(), epsilon);
   }


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
}
