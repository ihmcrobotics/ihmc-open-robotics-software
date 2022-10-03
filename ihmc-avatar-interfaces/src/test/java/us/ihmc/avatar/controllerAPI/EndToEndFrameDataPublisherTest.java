package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import ihmc_common_msgs.msg.dds.RobotFrameData;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class EndToEndFrameDataPublisherTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();
   private static final DRCStartingLocation location = DRCObstacleCourseStartingLocation.DEFAULT;

   private SCS2AvatarTestingSimulation simulationTestHelper;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

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
   public void testQueuing() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = getRobotModel();
      createSimulationTestHelper(environment, location);
      simulationTestHelper.start();
      ThreadTools.sleep(1000);
      assertTrue(simulationTestHelper.simulateNow(0.25));

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();
      MovingReferenceFrame midFeetFrame = referenceFrames.getMidFootZUpGroundFrame();

      double maxStepWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getMaxStepWidth();
      double minStepWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getMinStepWidth();
      double halfStepWidth = (maxStepWidth + minStepWidth) / 4.0;
      double stepLength = robotModel.getWalkingControllerParameters().getSteppingParameters().getDefaultStepLength() * 0.5;
      double nominalSwingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      double nominalTransferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      Random random = new Random(24384523737236643L);

      List<FootstepDataMessage> footstepList = new ArrayList<>();
      RobotSide stepSide = RobotSide.LEFT;
      int steps = 8;
      int stepsPerMessage = 3;
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
         FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(stepSide, frameLocation, frameOrientation);

         // between 0.75 and 1.25 times the nominal time:
         double swingTime = (1.0 + 0.5 * (random.nextDouble() + 0.5)) * nominalSwingTime;
         double transferTime = (1.0 + 0.5 * (random.nextDouble() + 0.5)) * nominalTransferTime;
         footstep.setSwingDuration(swingTime);
         footstep.setTransferDuration(transferTime);

         footstepList.add(footstep);
         totalTime += swingTime + transferTime;
         stepSide = stepSide.getOppositeSide();
      }

      List<FootstepDataListMessage> messages = new ArrayList<>();
      int stepsPackedInMessage = 0;
      while (stepsPackedInMessage < footstepList.size())
      {
         FootstepDataListMessage message = new FootstepDataListMessage();
         for (int footstepIdx = 0; footstepIdx < stepsPerMessage; footstepIdx++)
         {
            if (stepsPackedInMessage == footstepList.size())
            {
               break;
            }

            message.getFootstepDataList().add().set(footstepList.get(stepsPackedInMessage));
            stepsPackedInMessage++;
         }
         message.getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
         message.getQueueingProperties().setPreviousMessageId(FootstepDataListMessage.VALID_MESSAGE_DEFAULT_ID);
         messages.add(message);
      }
      FootstepDataListMessage r = messages.get(0);
      r.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
      r.getQueueingProperties().setPreviousMessageId(FootstepDataListMessage.VALID_MESSAGE_DEFAULT_ID);

      YoVariable numberOfStepsInController = simulationTestHelper.findVariable(WalkingMessageHandler.class.getSimpleName(), "currentNumberOfFootsteps");
      int expectedNumberOfSteps = 0;

      double timeBetweenSendingMessages = nominalTransferTime / messages.size();
      double timeUntilDone = totalTime;
      for (int messageIdx = 0; messageIdx < messages.size(); messageIdx++)
      {
         simulationTestHelper.publishToController(messages.get(messageIdx));
         System.out.println("checking step list message: \n" + messages.get(messageIdx).toString());
         expectedNumberOfSteps += messages.get(messageIdx).getFootstepDataList().size();
         assertTrue(simulationTestHelper.simulateNow(timeBetweenSendingMessages));
         assertEquals(expectedNumberOfSteps, (int) numberOfStepsInController.getValueAsLongBits());
         timeUntilDone -= timeBetweenSendingMessages;
      }

      assertTrue(simulationTestHelper.simulateNow(timeUntilDone + 0.25));
      assertEquals(0, (int) numberOfStepsInController.getValueAsLongBits());
   }

   @Tag("controller-api")
   @Test
   //   @Disabled
   public void testWalking() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = getRobotModel();
      createSimulationTestHelper(environment, location);
      simulationTestHelper.start();
      ThreadTools.sleep(1000);
      assertTrue(simulationTestHelper.simulateNow(0.25));

      double maxStepWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getMaxStepWidth();
      double minStepWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getMinStepWidth();
      double halfStepWidth = (maxStepWidth + minStepWidth) / 4.0;
      double stepLength = robotModel.getWalkingControllerParameters().getSteppingParameters().getDefaultStepLength() * 0.5;
      double nominalSwingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      double nominalTransferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      Random random = new Random(24384523737236643L);
      double swingTime = (1.0 + 0.5 * (random.nextDouble() + 0.5)) * nominalSwingTime;
      double transferTime = (1.0 + 0.5 * (random.nextDouble() + 0.5)) * nominalTransferTime;

      //            FootstepDataListMessage footStepDataListMessage = EndToEndTestTools.generateForwardSteps(stepSide,numberOfSteps, stepLength, stepWidth, swingTime, transferTime, new Pose3D(), false);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      FramePose3D startPose = new FramePose3D(simulationTestHelper.getControllerReferenceFrames().getMidFootZUpGroundFrame());
      startPose.changeFrame(worldFrame);
      //      FootstepDataListMessage steps = EndToEndTestTools.generateForwardSteps(RobotSide.LEFT,
      //                                                                             6,
      //                                                                             a -> steppingParameters.getDefaultStepLength(),
      //                                                                             steppingParameters.getInPlaceWidth(),
      //                                                                             swingTime,
      //                                                                             transferTime,
      //                                                                             startPose,
      //                                                                             true);

      //      FootstepDataListMessage steps = EndToEndTestTools.generateForwardSteps(RobotSide.LEFT, 6, 0.5, 0.25, swingTime, transferTime, startPose, true);

      FootstepDataListMessage steps = EndToEndTestTools.generateCircleSteps(RobotSide.LEFT,
                                                                            4,
                                                                            a -> steppingParameters.getDefaultStepLength(),
                                                                            steppingParameters.getInPlaceWidth(),
                                                                            0.75,
                                                                            0.25,
                                                                            startPose,
                                                                            true,
                                                                            RobotSide.RIGHT,
                                                                            1.0);
      

      
//      simulationTestHelper.createSubscriberFromController(RobotFrameData.class, robotFrameData ->  {
//         frameMessages.add(robotFrameData);
//         System.out.println("frame name in callback: " + robotFrameData.getFrameName());
//         System.out.println("frame pose in callback: " + robotFrameData.getFramePoseInWorld().toString());
//         
//      });
      //    simulationTestHelper.createSubscriber(RobotFrameData.class, topicName, frameMessages::add);
      

      simulationTestHelper.publishToController(steps);
      
      //      fullRobotModel.updateFrames();
      ROS2Topic<RobotFrameData> ros2Topic = ControllerAPIDefinition.getOutputTopic(robotModel.getSimpleRobotName()).withSuffix("afterL_arm_ely").withType(RobotFrameData.class);
      List<RobotFrameData> frameMessages = new ArrayList<>();
      
      
      
      assertTrue(simulationTestHelper.simulateNow(EndToEndTestTools.computeWalkingDuration(steps, robotModel.getWalkingControllerParameters())));
      
      simulationTestHelper.createSubscriber(RobotFrameData.class, ros2Topic, robotFrameData -> {
         frameMessages.add(robotFrameData);
         System.out.println("frame name in callback: " + robotFrameData.getFrameName());
         System.out.println("frame pose in callback: " + robotFrameData.getFramePoseInWorld().toString());
      });
      
      
      // yoGraphic referenceFrame . . . 
      // update 
      
      
      for (RobotFrameData rfd : frameMessages)
      {
         System.out.println("NAME : " + rfd.getFrameName().toString());
         System.out.println("POSE : " + rfd.getFramePoseInWorld().toString());
      }

      String topic = "framedata";

      String topicName = "frame data";

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      
//      fullRobotModel.
   }

   protected void testMessageIsHandled(FootstepDataListMessage messageInMidFeetZUp) throws SimulationExceededMaximumTimeException
   {
      CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();
      DRCStartingLocation location = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      createSimulationTestHelper(environment, location);
      simulationTestHelper.start();
      assertTrue(simulationTestHelper.simulateNow(0.25));

      MovingReferenceFrame messageFrame = simulationTestHelper.getControllerReferenceFrames().getMidFeetZUpFrame();
      transformMessageToWorld(messageFrame, messageInMidFeetZUp);

      simulationTestHelper.publishToController(messageInMidFeetZUp);
      assertTrue(simulationTestHelper.simulateNow(0.25));

      int steps = (int) simulationTestHelper.findVariable("currentNumberOfFootsteps").getValueAsDouble();
      assertEquals(messageInMidFeetZUp.getFootstepDataList().size(), steps);
   }

   private static void transformMessageToWorld(ReferenceFrame messageFrame, FootstepDataListMessage message)
   {
      int steps = message.getFootstepDataList().size();
      FramePose3D stepPose = new FramePose3D();
      for (int i = 0; i < steps; i++)
      {
         FootstepDataMessage footstep = message.getFootstepDataList().get(i);
         stepPose.setIncludingFrame(messageFrame, footstep.getLocation(), footstep.getOrientation());
         stepPose.changeFrame(ReferenceFrame.getWorldFrame());
         footstep.getLocation().set(stepPose.getPosition());
         footstep.getOrientation().set(stepPose.getOrientation());
      }
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
