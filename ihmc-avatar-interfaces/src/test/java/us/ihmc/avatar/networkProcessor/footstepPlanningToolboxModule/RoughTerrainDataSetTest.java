package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import com.jme3.math.Transform;
import controller_msgs.msg.dds.*;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments;
import us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.*;
import us.ihmc.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.RemoteUIMessageConverter;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ContactSensorDefinition;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;

import java.io.IOException;
import java.io.InputStream;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.FootstepPlanTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlanningResultTopic;
import static us.ihmc.footstepPlanning.testTools.PlannerTestEnvironments.*;

public abstract class RoughTerrainDataSetTest
{
   protected static final double bambooTimeScaling = 4.0;

   // Whether to start the UI or not.
   protected static boolean VISUALIZE = false;
   // For enabling helpful prints.
   protected static boolean DEBUG = false;

   private boolean checkForBodyBoxCollision = true;
   private double bodyBoxDepth = 0.3;
   private double bodyBoxWidth = 0.7;
   private double bodyBoxOffsetX = 0.0;

   protected FootstepPlannerUI ui = null;
   protected Messager messager = null;

   private final AtomicReference<FootstepPlan> plannerPlanReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> plannerResultReference = new AtomicReference<>(null);
   protected final AtomicReference<Boolean> plannerReceivedPlan = new AtomicReference<>(false);
   protected final AtomicReference<Boolean> plannerReceivedResult = new AtomicReference<>(false);

   protected AtomicReference<FootstepPlan> uiFootstepPlanReference;
   protected AtomicReference<FootstepPlanningResult> uiPlanningResultReference;
   protected final AtomicReference<Boolean> uiReceivedPlan = new AtomicReference<>(false);
   protected final AtomicReference<Boolean> uiReceivedResult = new AtomicReference<>(false);

   protected final AtomicReference<FootstepPlan> expectedPlan = new AtomicReference<>(null);
   protected final AtomicReference<FootstepPlan> actualPlan = new AtomicReference<>(null);
   protected final AtomicReference<FootstepPlanningResult> expectedResult = new AtomicReference<>(null);
   protected final AtomicReference<FootstepPlanningResult> actualResult = new AtomicReference<>(null);

   private static final String robotName = "testBot";
   private MultiStageFootstepPlanningModule toolboxModule;
   private IHMCRealtimeROS2Publisher<FootstepPlannerParametersPacket> footstepPlannerParametersPublisher;
   private IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;

   private RealtimeRos2Node ros2Node;

   private RemoteUIMessageConverter messageConverter = null;

   protected DomainFactory.PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;

   public abstract FootstepPlannerType getPlannerType();

   @BeforeEach
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (VISUALIZE)
         messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      else
         messager = new SharedMemoryMessager(FootstepPlannerMessagerAPI.API);

      messageConverter = RemoteUIMessageConverter.createConverter(messager, robotName, pubSubImplementation);

      tryToStartModule(() -> setupFootstepPlanningToolboxModule());

      messager.registerTopicListener(FootstepPlanTopic, request -> uiReceivedPlan.set(true));
      messager.registerTopicListener(PlanningResultTopic, request -> uiReceivedResult.set(true));

      uiFootstepPlanReference = messager.createInput(FootstepPlanTopic);
      uiPlanningResultReference = messager.createInput(PlanningResultTopic);

      ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_footstep_planner_test");

      footstepPlanningRequestPublisher = ROS2Tools
            .createPublisher(ros2Node, FootstepPlanningRequestPacket.class, FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      footstepPlannerParametersPublisher = ROS2Tools
            .createPublisher(ros2Node, FootstepPlannerParametersPacket.class, FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      toolboxStatePublisher = ROS2Tools
            .createPublisher(ros2Node, ToolboxStateMessage.class, FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));

      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningToolboxOutputStatus.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));

      ros2Node.spin();

      try
      {
         messager.startMessager();
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to start messager.");
      }

      if (VISUALIZE)
      {
         createUI(messager);
      }

      for (int i = 0; i < 100; i++)
         ThreadTools.sleep(10);
   }

   public void setCheckForBodyBoxCollision(boolean checkForBodyBoxCollision)
   {
      this.checkForBodyBoxCollision = checkForBodyBoxCollision;
   }

   public void setBodyBoxDepth(double bodyBoxDepth)
   {
      this.bodyBoxDepth = bodyBoxDepth;
   }

   public void setBodyBoxWidth(double bodyBoxWidth)
   {
      this.bodyBoxWidth = bodyBoxWidth;
   }

   public void setBodyBoxOffsetX(double bodyBoxOffsetX)
   {
      this.bodyBoxOffsetX = bodyBoxOffsetX;
   }


   @AfterEach
   public void tearDown() throws Exception
   {
      ros2Node.destroy();

      messager.closeMessager();
      messageConverter.destroy();
      toolboxModule.destroy();

      if (ui != null)
         ui.stop();
      ui = null;

      pubSubImplementation = null;

      uiFootstepPlanReference = null;
      uiPlanningResultReference = null;

      ros2Node = null;
      footstepPlanningRequestPublisher = null;
      toolboxModule = null;

      messageConverter = null;
      messager = null;
   }

   private void resetAllAtomics()
   {
      plannerPlanReference.set(null);
      plannerResultReference.set(null);
      plannerReceivedPlan.set(false);
      plannerReceivedResult.set(false);

      if (uiFootstepPlanReference != null)
         uiFootstepPlanReference.set(null);
      if (uiPlanningResultReference != null)
         uiPlanningResultReference.set(null);
      uiReceivedPlan.set(false);
      uiReceivedResult.set(false);

      expectedPlan.set(null);
      actualPlan.set(null);
      expectedResult.set(null);
      actualResult.set(null);
   }

   private void tryToStartModule(ModuleStarter runnable)
   {
      try
      {
         runnable.startModule();
      }
      catch (RuntimeException | IOException e)
      {
         PrintTools.error(this, "Failed to start a module in the network processor, stack trace:");
         e.printStackTrace();
      }
   }

   private interface ModuleStarter
   {
      void startModule() throws IOException;
   }

   private void setupFootstepPlanningToolboxModule() throws IOException
   {
      toolboxModule = new MultiStageFootstepPlanningModule(getRobotModel(), null, true, pubSubImplementation);
   }

   private DRCRobotModel getRobotModel()
   {
      return new TestRobotModel();
   }

   public void testDownCorridor()
   {
      runAssertions(getTestData(corridor));
   }

   public void testBetweenTwoBollards()
   {
      setCheckForBodyBoxCollision(true);
      setBodyBoxDepth(0.45);
      setBodyBoxWidth(0.9);
      setBodyBoxOffsetX(0.1);
      runAssertions(getTestData(bollards));
   }

   public String runAssertions(PlannerTestData dataset)
   {
      submitDataSet(dataset);

      return findPlanAndAssertGoodResult(dataset);
   }

   protected void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      if (DEBUG)
         PrintTools.info("Processed an output from a remote planner.");

      plannerResultReference.set(FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult()));
      plannerPlanReference.set(convertToFootstepPlan(packet.getFootstepDataList()));
      plannerReceivedPlan.set(true);
      plannerReceivedResult.set(true);
   }

   private static FootstepPlan convertToFootstepPlan(FootstepDataListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      for (FootstepDataMessage footstepMessage : footstepDataListMessage.getFootstepDataList())
      {
         FramePose3D stepPose = new FramePose3D();
         stepPose.setPosition(footstepMessage.getLocation());
         stepPose.setOrientation(footstepMessage.getOrientation());
         SimpleFootstep footstep = footstepPlan.addFootstep(RobotSide.fromByte(footstepMessage.getRobotSide()), stepPose);

         ConvexPolygon2D foothold = new ConvexPolygon2D();
         for (int i = 0; i < footstepMessage.getPredictedContactPoints2d().size(); i++)
            foothold.addVertex(footstepMessage.getPredictedContactPoints2d().get(i));
         foothold.update();
         footstep.setFoothold(foothold);
      }

      return footstepPlan;
   }

   protected String assertPlansAreValid(String datasetName, FootstepPlanningResult expectedResult, FootstepPlanningResult actualResult,
                                        FootstepPlan expectedPlan, FootstepPlan actualPlan, Point3D goal)
   {
      String errorMessage = "";

      errorMessage += assertTrue(datasetName, "Planning results for " + datasetName + " are not equal: " + expectedResult + " and " + actualResult + ".\n",
                                 expectedResult.equals(actualResult));

      errorMessage += assertPlanIsValid(datasetName, expectedResult, expectedPlan, goal);
      errorMessage += assertPlanIsValid(datasetName, actualResult, actualPlan, goal);

      if (actualResult.validForExecution())
      {
         errorMessage += areFootstepPlansEqual(actualPlan, expectedPlan);
      }

      return errorMessage;
   }

   protected String assertPlanIsValid(String datasetName, FootstepPlanningResult result, FootstepPlan plan, Point3D goal)
   {
      String errorMessage = "";

      errorMessage += assertTrue(datasetName, "Planning result for " + datasetName + " is invalid, result was " + result, result.validForExecution());

      if (result.validForExecution())
      {
         errorMessage += assertTrue(datasetName,
                                    datasetName + " did not reach goal. Made it to " + PlannerTools.getEndPosition(plan) + ", trying to get to " + goal,
                                    PlannerTools.isGoalNextToLastStep(goal, plan));
      }

      return errorMessage;
   }

   private String assertTrue(String datasetName, String message, boolean condition)
   {
      if (VISUALIZE || DEBUG)
      {
         if (!condition)
            PrintTools.error(datasetName + ": " + message);
      }
      return !condition ? "\n" + message : "";
   }

   private String areFootstepPlansEqual(FootstepPlan actualPlan, FootstepPlan expectedPlan)
   {
      String errorMessage = "";

      if (actualPlan.getNumberOfSteps() != expectedPlan.getNumberOfSteps())
      {
         errorMessage += "Actual Plan has " + actualPlan.getNumberOfSteps() + ", while Expected Plan has " + expectedPlan.getNumberOfSteps() + ".\n";
      }

      for (int i = 0; i < Math.min(actualPlan.getNumberOfSteps(), expectedPlan.getNumberOfSteps()); i++)
      {
         errorMessage += areFootstepsEqual(i, actualPlan.getFootstep(i), expectedPlan.getFootstep(i));
      }

      return errorMessage;
   }

   private String areFootstepsEqual(int footstepNumber, SimpleFootstep actual, SimpleFootstep expected)
   {
      String errorMessage = "";

      if (!actual.getRobotSide().equals(expected.getRobotSide()))
      {
         errorMessage += "Footsteps " + footstepNumber + " are different robot sides: " + actual.getRobotSide() + " and " + expected.getRobotSide() + ".\n";
      }

      FramePose3D poseA = new FramePose3D();
      FramePose3D poseB = new FramePose3D();

      actual.getSoleFramePose(poseA);
      expected.getSoleFramePose(poseB);

      if (!poseA.epsilonEquals(poseB, 1e-5))
      {
         errorMessage += "Footsteps " + footstepNumber + " have different poses: \n \t" + poseA.toString() + "\n and \n\t " + poseB.toString() + ".\n";
      }

      if (!actual.epsilonEquals(expected, 1e-5))

      {
         errorMessage += "Footsteps " + footstepNumber + " are not equal: \n \t" + actual.toString() + "\n and \n\t " + expected.toString() + ".\n";
      }

      return errorMessage;
   }

   protected void createUI(Messager messager)
   {
      ApplicationRunner.runApplication(new Application()
      {
         @Override
         public void start(Stage stage) throws Exception
         {
            ui = FootstepPlannerUI.createMessagerUI(stage, (SharedMemoryJavaFXMessager) messager);
            ui.show();
         }

         @Override
         public void stop() throws Exception
         {
            ui.stop();
            Platform.exit();
         }
      });

      double maxWaitTime = 5.0;
      double totalTime = 0.0;
      long sleepDuration = 100;

      while (ui == null)
      {
         if (totalTime > maxWaitTime)
            throw new RuntimeException("Timed out waiting for the UI to start.");
         ThreadTools.sleep(sleepDuration);
         totalTime += Conversions.millisecondsToSeconds(sleepDuration);
      }
   }

   protected double totalTimeTaken;

   protected String waitForResult(ConditionChecker conditionChecker, double maxTimeToWait, String prefix)
   {
      String errorMessage = "";
      long waitTime = 10;
      while (conditionChecker.checkCondition())
      {
         if (totalTimeTaken > maxTimeToWait)
         {
            errorMessage += prefix + " timed out waiting for a result.\n";
            return errorMessage;
         }

         ThreadTools.sleep(waitTime);
         totalTimeTaken += Conversions.millisecondsToSeconds(waitTime);
         queryUIResults();
         queryPlannerResults();
      }

      return errorMessage;
   }

   protected String validateResult(ConditionChecker conditionChecker, FootstepPlanningResult result, String prefix)
   {
      String errorMessage = "";

      if (!conditionChecker.checkCondition())
      {
         errorMessage += prefix + " failed to find a valid result. Result : " + result + "\n";
      }

      return errorMessage;
   }

   protected String waitForPlan(ConditionChecker conditionChecker, double maxTimeToWait, String prefix)
   {
      String errorMessage = "";

      while (conditionChecker.checkCondition())
      {
         long waitTime = 10;

         if (totalTimeTaken > maxTimeToWait)
         {
            errorMessage += prefix + " timed out waiting on plan.\n";
            return errorMessage;
         }

         ThreadTools.sleep(waitTime);
         totalTimeTaken += Conversions.millisecondsToSeconds(waitTime);
         queryUIResults();
         queryPlannerResults();
      }

      return errorMessage;
   }

   protected void queryUIResults()
   {
      if (uiReceivedPlan.get() && uiFootstepPlanReference.get() != null && actualPlan.get() == null)
      {
         if (DEBUG)
            PrintTools.info("Received a plan from the UI.");
         actualPlan.set(uiFootstepPlanReference.getAndSet(null));
         uiReceivedPlan.set(false);
      }

      if (uiReceivedResult.get() && uiPlanningResultReference.get() != null)
      {
         if (DEBUG)
            PrintTools.info("Received a result " + uiPlanningResultReference.get() + " from the UI.");
         actualResult.set(uiPlanningResultReference.getAndSet(null));
         uiReceivedResult.set(false);
      }
   }

   protected void queryPlannerResults()
   {
      if (plannerReceivedPlan.get() && plannerPlanReference.get() != null && expectedPlan.get() == null)
      {
         if (DEBUG)
            PrintTools.info("Received a plan from the planner.");
         expectedPlan.set(plannerPlanReference.getAndSet(null));
         plannerReceivedPlan.set(false);
      }

      if (plannerReceivedResult.get() && plannerResultReference.get() != null)
      {
         if (DEBUG)
            PrintTools.info("Received a result " + plannerResultReference.get() + " from the planner.");
         expectedResult.set(plannerResultReference.getAndSet(null));
         plannerReceivedResult.set(false);
      }
   }

   public void submitDataSet(PlannerTestEnvironments.PlannerTestData testData)
   {
      for (int i = 0; i < 100; i++)
         ThreadTools.sleep(10);

      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

      for (int i = 0; i < 100; i++)
         ThreadTools.sleep(10);

      FootstepPlannerParametersPacket parametersPacket = new FootstepPlannerParametersPacket();
      FootstepPlannerParameters parameters = getRobotModel().getFootstepPlannerParameters();

      if (messager != null)
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParametersTopic, parameters);
         messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionDataTopic, testData.getPlanarRegionsList());
      }

      FootstepPlannerMessageTools.copyParametersToPacket(parametersPacket, parameters);

      footstepPlannerParametersPublisher.publish(parametersPacket);

      FootstepPlanningRequestPacket packet = new FootstepPlanningRequestPacket();

      byte plannerType = getPlannerType().toByte();
      PlanarRegionsListMessage planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(testData.getPlanarRegionsList());

      packet.getStanceFootPositionInWorld().set(testData.getStartPosition());
      packet.getGoalPositionInWorld().set(testData.getGoalPosition());
      packet.setRequestedFootstepPlannerType(plannerType);
      packet.getPlanarRegionsListMessage().set(planarRegions);

      double timeout = 60.0;
      double timeoutMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      packet.setTimeout(timeoutMultiplier * timeout);

      packet.setHorizonLength(Double.MAX_VALUE);

      packet.getGoalOrientationInWorld().set(testData.getGoalOrientation());
      packet.getStanceFootOrientationInWorld().set(testData.getStartOrientation());

      if (DEBUG)
         PrintTools.info("Sending out planning request packet.");

      footstepPlanningRequestPublisher.publish(packet);
   }

   public String findPlanAndAssertGoodResult(PlannerTestData dataset)
   {
      totalTimeTaken = 0;
      double timeoutMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double maxTimeToWait = 2.0 * timeoutMultiplier * 60.0;
      String datasetName = "";

      queryUIResults();
      queryPlannerResults();

      String errorMessage = "";

      if (DEBUG)
         PrintTools.info("Waiting for result.");

      errorMessage += waitForResult(() -> actualResult.get() == null || expectedResult.get() == null, maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      if (DEBUG)
         PrintTools.info("Received a result (actual = " + actualResult.get() + " expected = " + expectedResult.get() + ", checking it's validity.");

      errorMessage += validateResult(() -> actualResult.get().validForExecution() && expectedResult.get().validForExecution(), actualResult.get(), datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      if (DEBUG)
         PrintTools.info("Results are valid, waiting for plan.");

      errorMessage += waitForPlan(() -> expectedPlan.get() == null || actualPlan.get() == null, maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      if (DEBUG)
         PrintTools.info("Received a plan, checking it's validity.");

      FootstepPlanningResult expectedResult = this.expectedResult.getAndSet(null);
      FootstepPlanningResult actualResult = this.actualResult.getAndSet(null);
      FootstepPlan expectedPlan = this.expectedPlan.getAndSet(null);
      FootstepPlan actualPlan = this.actualPlan.getAndSet(null);

      uiReceivedResult.set(false);
      uiReceivedPlan.set(false);
      plannerReceivedPlan.set(false);
      plannerReceivedResult.set(false);

      errorMessage += assertPlansAreValid(datasetName, expectedResult, actualResult, expectedPlan, actualPlan, dataset.getGoalPosition());

      for (int i = 0; i < 100; i++)
         ThreadTools.sleep(10);

      return errorMessage;
   }

   protected static interface DatasetTestRunner
   {
      String testDataset(FootstepPlannerUnitTestDataset dataset);
   }

   protected static interface ConditionChecker
   {
      boolean checkCondition();
   }

   private class TestRobotModel implements DRCRobotModel
   {
      @Override
      public DRCRobotJointMap getJointMap()
      {
         return null;
      }

      @Override
      public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
      {
         return null;
      }

      @Override
      public HandModel getHandModel()
      {
         return null;
      }

      @Override
      public Transform getJmeTransformWristToHand(RobotSide side)
      {
         return null;
      }

      @Override
      public double getSimulateDT()
      {
         return 0;
      }

      @Override
      public double getEstimatorDT()
      {
         return 0;
      }

      @Override
      public double getStandPrepAngle(String jointName)
      {
         return 0;
      }

      @Override
      public DRCROSPPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
      {
         return null;
      }

      @Override
      public DRCSensorSuiteManager getSensorSuiteManager()
      {
         return null;
      }

      @Override
      public MultiThreadedRobotControlElement createSimulatedHandController(FloatingRootJointRobot simulatedRobot,
                                                                            ThreadDataSynchronizerInterface threadDataSynchronizer,
                                                                            RealtimeRos2Node realtimeRos2Node,
                                                                            CloseableAndDisposableRegistry closeableAndDisposableRegistry)
      {
         return null;
      }

      @Override
      public LogSettings getLogSettings()
      {
         return null;
      }

      @Override
      public LogModelProvider getLogModelProvider()
      {
         return null;
      }

      @Override
      public String getSimpleRobotName()
      {
         return robotName;
      }

      @Override
      public CollisionBoxProvider getCollisionBoxProvider()
      {
         return null;
      }

      @Override
      public HighLevelControllerParameters getHighLevelControllerParameters()
      {
         return null;
      }

      @Override
      public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
      {
         return null;
      }

      @Override
      public RobotDescription getRobotDescription()
      {
         return null;
      }

      @Override
      public FullHumanoidRobotModel createFullRobotModel()
      {
         return new TestFullRobotModel();
      }

      @Override
      public double getControllerDT()
      {
         return 0;
      }

      @Override
      public StateEstimatorParameters getStateEstimatorParameters()
      {
         return null;
      }

      @Override
      public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
      {
         return null;
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return null;
      }

      @Override
      public RobotContactPointParameters<RobotSide> getContactPointParameters()
      {
         return null;
      }

      @Override
      public DRCRobotSensorInformation getSensorInformation()
      {
         return null;
      }

      @Override
      public InputStream getWholeBodyControllerParametersFile()
      {
         return null;
      }

      @Override
      public FootstepPlannerParameters getFootstepPlannerParameters()
      {
         return new DefaultFootstepPlanningParameters()

         {
            @Override
            public double getBodyBoxBaseX()
            {
               return bodyBoxOffsetX;
            }

            @Override
            public double getBodyBoxDepth()
            {
               return bodyBoxDepth;
            }

            @Override
            public double getBodyBoxWidth()
            {
               return bodyBoxWidth;
            }

            @Override
            public boolean checkForBodyBoxCollisions()
            {
               return checkForBodyBoxCollision;
            }
         };
      }

      @Override
      public VisibilityGraphsParameters getVisibilityGraphsParameters()
      {
         return new DefaultVisibilityGraphParameters();
      }
   }

   public class TestFullRobotModel implements FullHumanoidRobotModel
   {

      @Override
      public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return null;
      }

      @Override
      public void updateFrames()
      {

      }

      @Override
      public MovingReferenceFrame getElevatorFrame()
      {
         return null;
      }

      @Override
      public FloatingJointBasics getRootJoint()
      {
         return null;
      }

      @Override
      public RigidBody getElevator()
      {
         return null;
      }

      @Override
      public OneDoFJoint getSpineJoint(SpineJointName spineJointName)
      {
         return null;
      }

      @Override
      public RigidBody getEndEffector(Enum<?> segmentEnum)
      {
         return null;
      }

      @Override
      public OneDoFJoint getNeckJoint(NeckJointName neckJointName)
      {
         return null;
      }

      @Override
      public JointBasics getLidarJoint(String lidarName)
      {
         return null;
      }

      @Override
      public ReferenceFrame getLidarBaseFrame(String name)
      {
         return null;
      }

      @Override
      public RigidBodyTransform getLidarBaseToSensorTransform(String name)
      {
         return null;
      }

      @Override
      public ReferenceFrame getCameraFrame(String name)
      {
         return null;
      }

      @Override
      public RigidBody getRootBody()
      {
         return null;
      }

      @Override
      public RigidBody getHead()
      {
         return null;
      }

      @Override
      public ReferenceFrame getHeadBaseFrame()
      {
         return null;
      }

      @Override
      public OneDoFJoint[] getOneDoFJoints()
      {
         return new OneDoFJoint[0];
      }

      @Override
      public Map<String, OneDoFJointBasics> getOneDoFJointsAsMap()
      {
         return null;
      }

      @Override
      public void getOneDoFJointsFromRootToHere(OneDoFJointBasics oneDoFJointAtEndOfChain, List<OneDoFJointBasics> oneDoFJointsToPack)
      {

      }

      @Override
      public OneDoFJoint[] getControllableOneDoFJoints()
      {
         return new OneDoFJoint[0];
      }

      @Override
      public void getOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {

      }

      @Override
      public OneDoFJoint getOneDoFJointByName(String name)
      {
         return null;
      }

      @Override
      public void getControllableOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {

      }

      @Override
      public IMUDefinition[] getIMUDefinitions()
      {
         return new IMUDefinition[0];
      }

      @Override
      public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return new ForceSensorDefinition[0];
      }

      @Override
      public ContactSensorDefinition[] getContactSensorDefinitions()
      {
         return new ContactSensorDefinition[0];
      }

      @Override
      public double getTotalMass()
      {
         return 0;
      }

      @Override
      public RigidBody getChest()
      {
         return null;
      }

      @Override
      public RigidBody getPelvis()
      {
         return null;
      }

      @Override
      public OneDoFJoint getArmJoint(RobotSide robotSide, ArmJointName armJointName)
      {
         return null;
      }

      @Override
      public RigidBody getHand(RobotSide robotSide)
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getHandControlFrame(RobotSide robotSide)
      {
         return null;
      }

      @Override
      public void setJointAngles(RobotSide side, LimbName limb, double[] q)
      {

      }

      @Override
      public MovingReferenceFrame getFrameAfterLegJoint(RobotSide robotSegment, LegJointName legJointName)
      {
         return null;
      }

      @Override
      public OneDoFJoint getLegJoint(RobotSide robotSegment, LegJointName legJointName)
      {
         return null;
      }

      @Override
      public RigidBody getFoot(RobotSide robotSegment)
      {
         return null;
      }

      @Override
      public RigidBody getEndEffector(RobotSide robotSegment, LimbName limbName)
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getEndEffectorFrame(RobotSide robotSegment, LimbName limbName)
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getSoleFrame(RobotSide robotSegment)
      {
         return null;
      }

      @Override
      public SideDependentList<MovingReferenceFrame> getSoleFrames()
      {
         return null;
      }
   }
}
