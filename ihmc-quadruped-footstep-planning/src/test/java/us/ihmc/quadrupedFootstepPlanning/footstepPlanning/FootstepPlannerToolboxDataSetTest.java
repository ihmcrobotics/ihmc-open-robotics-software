package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import controller_msgs.msg.dds.*;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.junit.Test;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedCommunication.networkProcessing.footstepPlanning.QuadrupedFootstepPlanningModule;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.tools.FootstepPlannerIOTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.tools.FootstepPlannerIOTools.FootstepPlannerUnitTestDataset;
import us.ihmc.quadrupedFootstepPlanning.ui.ApplicationRunner;
import us.ihmc.quadrupedFootstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.quadrupedFootstepPlanning.ui.RemoteUIMessageConverter;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.DefaultPointFootSnapperParameters;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public abstract class FootstepPlannerToolboxDataSetTest
{
   protected static final double bambooTimeScaling = 4.0;
   private static final double epsilon = 1e-3;


   // Whether to start the UI or not.
   protected static boolean VISUALIZE = false;
   // For enabling helpful prints.
   protected static boolean DEBUG = true;
   protected static boolean VERBOSE = true;

   private FootstepPlannerUI ui = null;
   protected Messager messager = null;

   private QuadrupedXGaitSettingsReadOnly xGaitSettings = null;
   private QuadrupedFootstepPlanningModule footstepPlanningModule = null;

   private RealtimeRos2Node ros2Node;
   private RemoteUIMessageConverter converter;
   private IHMCRealtimeROS2Publisher<QuadrupedFootstepPlannerParametersPacket> footstepPlannerParametersPublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedFootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;

   private final AtomicReference<FootstepPlan> plannerPlanReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> plannerResultReference = new AtomicReference<>(null);
   private final AtomicReference<Boolean> plannerReceivedPlan = new AtomicReference<>(false);
   private final AtomicReference<Boolean> plannerReceivedResult = new AtomicReference<>(false);

   private final AtomicReference<FootstepPlan> planReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningResult> resultReference = new AtomicReference<>(null);

   private static final String robotName = "testBot";
   public static final PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;


   protected abstract FootstepPlannerType getPlannerType();

   protected abstract QuadrupedXGaitSettingsReadOnly getXGaitSettings();

   @BeforeEach
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (VISUALIZE)
         messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      else
         messager = new SharedMemoryMessager(FootstepPlannerMessagerAPI.API);

      if (xGaitSettings == null)
         xGaitSettings = getXGaitSettings();

      footstepPlanningModule = new QuadrupedFootstepPlanningModule(robotName, null, new DefaultFootstepPlannerParameters(), xGaitSettings,
                                                                   new DefaultPointFootSnapperParameters(), null, false, pubSubImplementation);


      ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_footstep_planner_test");

      footstepPlanningRequestPublisher = ROS2Tools
            .createPublisher(ros2Node, QuadrupedFootstepPlanningRequestPacket.class, FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      footstepPlannerParametersPublisher = ROS2Tools
            .createPublisher(ros2Node, QuadrupedFootstepPlannerParametersPacket.class, FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));
      toolboxStatePublisher = ROS2Tools
            .createPublisher(ros2Node, ToolboxStateMessage.class, FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotName));

      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedFootstepPlanningToolboxOutputStatus.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));

      converter = RemoteUIMessageConverter.createConverter(messager, robotName, pubSubImplementation);

      ros2Node.spin();

      try
      {
         messager.startMessager();
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to start messager.");
      }

      messager.submitMessage(FootstepPlannerMessagerAPI.XGaitSettingsTopic, xGaitSettings);

      if (VISUALIZE)
      {
         createUI(messager);
      }

      ThreadTools.sleep(1000);
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      messager.closeMessager();
      footstepPlanningModule.destroy();
      converter.destroy();
      if (ui != null)
         ui.stop();

      converter = null;
      footstepPlanningModule = null;
      ui = null;
      messager = null;
   }

   private void resetAllAtomics()
   {
      plannerPlanReference.set(null);
      plannerResultReference.set(null);
      plannerReceivedPlan.set(false);
      plannerReceivedResult.set(false);


      planReference.set(null);
      resultReference.set(null);
   }

   private void createUI(Messager messager)
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

   @Test
   public void testDatasetsWithoutOcclusion()
   {
      List<FootstepPlannerUnitTestDataset> allDatasets = FootstepPlannerIOTools.loadAllFootstepPlannerDatasetsWithoutOcclusions(FootstepPlannerIOTools.class);
      runAssertionsOnAllDatasets(this::runAssertions, allDatasets);
   }

   @Disabled
   @Test
   public void testDatasetsWithoutOcclusionInDevelopment()
   {
      List<FootstepPlannerUnitTestDataset> allDatasets = FootstepPlannerIOTools
            .loadAllFootstepPlannerDatasetsWithoutOcclusionsInDevelopment(FootstepPlannerIOTools.class);
      runAssertionsOnAllDatasets(this::runAssertions, allDatasets);
   }

   protected void runAssertionsOnDataset(DatasetTestRunner datasetTestRunner, String datasetName)
   {
      FootstepPlannerUnitTestDataset dataset = FootstepPlannerIOTools.loadDataset(FootstepPlannerIOTools.class, datasetName);

      String errorMessages = datasetTestRunner.testDataset(dataset);
      //      Assert.assertTrue("Errors:" + errorMessages, errorMessages.isEmpty());
   }

   private void runAssertionsOnAllDatasets(DatasetTestRunner datasetTestRunner, List<FootstepPlannerUnitTestDataset> allDatasets)
   {
      if (VERBOSE || DEBUG)
         LogTools.info("Unit test files found: " + allDatasets.size());

      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      int numberOfFailingTests = 0;
      List<String> failingDatasets = new ArrayList<>();
      int numbberOfTestedSets = 0;
      for (int i = 0; i < allDatasets.size(); i++)
      {
         FootstepPlannerUnitTestDataset dataset = allDatasets.get(i);
         if (DEBUG || VERBOSE)
            LogTools.info("Testing file: " + dataset.getDatasetName());

         if (!dataset.getTypes().contains(getPlannerType()))
         {
            if (DEBUG || VERBOSE)
               LogTools.info(dataset.getDatasetName() + " does not contain planner type " + getPlannerType() + ", skipping");
            continue;
         }

         numbberOfTestedSets++;
         String errorMessagesForCurrentFile = datasetTestRunner.testDataset(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
         {
            numberOfFailingTests++;
            failingDatasets.add(dataset.getDatasetName());
         }

         if (DEBUG || VERBOSE)
         {
            String result = errorMessagesForCurrentFile.isEmpty() ? "passed" : "failed";
            LogTools.info(dataset.getDatasetName() + " " + result);
         }

         ThreadTools.sleep(500); // Apparently need to give some time for the prints to appear in the right order.
      }

      String message = "Number of failing datasets: " + numberOfFailingTests + " out of " + numbberOfTestedSets;
      message += "\n Datasets failing: ";
      for (int i = 0; i < failingDatasets.size(); i++)
      {
         message += "\n" + failingDatasets.get(i);
      }
      if (VISUALIZE)
      {
         LogTools.info(message);
         ThreadTools.sleepForever();
      }
      else
      {
         Assert.assertEquals(message, 0, numberOfFailingTests);
      }
   }

   protected String runAssertions(FootstepPlannerUnitTestDataset dataset)
   {
      resetAllAtomics();
      ThreadTools.sleep(1000);
      packPlanningRequest(dataset);
      String errorMessage = findPlanAndAssertGoodResult(dataset);

      return errorMessage;
   }

   protected void packPlanningRequest(FootstepPlannerUnitTestDataset dataset)
   {
      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();
      startPose.setPosition(dataset.getStart());
      goalPose.setPosition(dataset.getGoal());
      if (dataset.hasStartOrientation())
         startPose.setOrientation(dataset.getStartOrientation());
      if (dataset.hasGoalOrientation())
         goalPose.setOrientation(dataset.getGoalOrientation());

      double timeMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double timeout = timeMultiplier * dataset.getTimeout(getPlannerType());

      QuadrupedFootstepPlannerStart start = new QuadrupedFootstepPlannerStart();
      QuadrupedFootstepPlannerGoal goal = new QuadrupedFootstepPlannerGoal();
      start.setStartPose(startPose);
      goal.setGoalPose(goalPose);

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTypeTopic, getPlannerType());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeoutTopic, timeout);

      messager.submitMessage(FootstepPlannerMessagerAPI.XGaitSettingsTopic, xGaitSettings);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionDataTopic, dataset.getPlanarRegionsList());
      messager.submitMessage(FootstepPlannerMessagerAPI.StartPositionTopic, new Point3D(startPose.getPosition()));
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalPositionTopic, new Point3D(goalPose.getPosition()));
      messager.submitMessage(FootstepPlannerMessagerAPI.StartOrientationTopic, new Quaternion(startPose.getOrientation()));
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalOrientationTopic, new Quaternion(goalPose.getOrientation()));

      messager.submitMessage(FootstepPlannerMessagerAPI.ComputePathTopic, true);

      //      planner.setHorizonLengthTopic(Double.MAX_VALUE);

      if (DEBUG)
         LogTools.info("Sending out planning request.");
   }

   private void processFootstepPlanningOutputStatus(QuadrupedFootstepPlanningToolboxOutputStatus packet)
   {
      if (DEBUG)
         PrintTools.info("Processed an output from a remote planner.");

      plannerResultReference.set(FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult()));
      plannerPlanReference.set(convertToFootstepPlan(packet.getFootstepDataList()));
      plannerReceivedPlan.set(true);
      plannerReceivedResult.set(true);
   }

   private static FootstepPlan convertToFootstepPlan(QuadrupedTimedStepListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      for (QuadrupedTimedStepMessage footstepMessage : footstepDataListMessage.getQuadrupedStepList())
      {
         QuadrupedTimedOrientedStep step = new QuadrupedTimedOrientedStep();
         step.setGoalPosition(footstepMessage.getQuadrupedStepMessage().getGoalPosition());
         step.getTimeInterval().setInterval(footstepMessage.getTimeInterval().getStartTime(), footstepMessage.getTimeInterval().getEndTime());
         step.setGroundClearance(footstepMessage.getQuadrupedStepMessage().getGroundClearance());
         step.setRobotQuadrant(RobotQuadrant.fromByte(footstepMessage.getQuadrupedStepMessage().getRobotQuadrant()));

         footstepPlan.addFootstep(step);
      }

      return footstepPlan;
   }

   private String findPlanAndAssertGoodResult(FootstepPlannerUnitTestDataset dataset)
   {
      totalTimeTaken = 0.0;
      double timeoutMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double maxTimeToWait = 2.0 * timeoutMultiplier * 60.0;
      String datasetName = "";

      queryPlannerResults();

      String errorMessage = "";
      if (DEBUG)
         PrintTools.info("Waiting for result.");

      errorMessage += waitForResult(() -> resultReference.get() == null, maxTimeToWait, datasetName);

      if (DEBUG)
         PrintTools.info("Received a result (actual = " + resultReference.get() + ", checking it's validity.");

      errorMessage += validateResult(() -> resultReference.get().validForExecution() , resultReference.get(), datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      if (DEBUG)
         PrintTools.info("Results are valid, waiting for plan.");

      errorMessage += waitForPlan(() -> planReference.get() == null, maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      if (DEBUG)
         PrintTools.info("Received a plan, checking it's validity.");

      FootstepPlanningResult result = this.resultReference.getAndSet(null);
      FootstepPlan plan = this.planReference.getAndSet(null);

      plannerReceivedPlan.set(false);
      plannerReceivedResult.set(false);

      errorMessage += assertPlanIsValid(datasetName, result, plan, dataset.getGoal(), dataset.getGoalOrientation());

      for (int i = 0; i < 100; i++)
         ThreadTools.sleep(10);

      return errorMessage;
   }


   private double totalTimeTaken;

   private String waitForResult(ConditionChecker conditionChecker, double maxTimeToWait, String prefix)
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
         queryPlannerResults();
      }

      return errorMessage;
   }

   private String validateResult(ConditionChecker conditionChecker, FootstepPlanningResult result, String prefix)
   {
      String errorMessage = "";

      if (!conditionChecker.checkCondition())
      {
         errorMessage += prefix + " failed to find a valid result. Result : " + result + "\n";
      }

      return errorMessage;
   }

   private String waitForPlan(ConditionChecker conditionChecker, double maxTimeToWait, String prefix)
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
         queryPlannerResults();
      }

      return errorMessage;
   }


   private void queryPlannerResults()
   {
      if (plannerReceivedPlan.get() && plannerPlanReference.get() != null && planReference.get() == null)
      {
         if (DEBUG)
            PrintTools.info("Received a plan from the planner.");
         planReference.set(plannerPlanReference.getAndSet(null));
         plannerReceivedPlan.set(false);
      }

      if (plannerReceivedResult.get() && plannerResultReference.get() != null)
      {
         if (DEBUG)
            PrintTools.info("Received a result " + plannerResultReference.get() + " from the planner.");
         resultReference.set(plannerResultReference.getAndSet(null));
         plannerReceivedResult.set(false);
      }
   }


   private static String assertPlanIsValid(String datasetName, FootstepPlanningResult result, FootstepPlan plannedSteps, Point3DReadOnly goalPosition, Quaternion goalOrientation)
   {
      QuadrantDependentList<Point3DBasics> finalSteps = getFinalStepPositions(plannedSteps);

      String errorMessage = "";
      if (!result.validForExecution())
         errorMessage = datasetName + " was not valid for execution " + result + ".\n";


      Point3D centerPoint = new Point3D();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         centerPoint.add(finalSteps.get(robotQuadrant));
      }

      double nominalYaw = FootstepNode.computeNominalYaw(finalSteps.get(RobotQuadrant.FRONT_LEFT).getX(), finalSteps.get(RobotQuadrant.FRONT_LEFT).getY(),
                                                         finalSteps.get(RobotQuadrant.FRONT_RIGHT).getX(), finalSteps.get(RobotQuadrant.FRONT_RIGHT).getY(),
                                                         finalSteps.get(RobotQuadrant.HIND_LEFT).getX(), finalSteps.get(RobotQuadrant.HIND_LEFT).getY(),
                                                         finalSteps.get(RobotQuadrant.HIND_RIGHT).getX(), finalSteps.get(RobotQuadrant.HIND_RIGHT).getY());

      centerPoint.scale(0.25);

      if (!goalPosition.epsilonEquals(centerPoint, FootstepNode.gridSizeXY))
         errorMessage += datasetName + " did not reach goal position. Made it to " + centerPoint + ", trying to get to " + goalPosition;
      if (goalOrientation != null)
      {
         double goalYaw = goalOrientation.getYaw();
         if (!MathTools.epsilonEquals(goalYaw, nominalYaw, FootstepNode.gridSizeYaw))
            errorMessage += datasetName + " did not reach goal yaw. Made it to " + nominalYaw + ", trying to get to " + goalYaw;
      }

      if ((VISUALIZE || DEBUG) && !errorMessage.isEmpty())
         LogTools.error(errorMessage);

      return errorMessage;
   }

   private static QuadrantDependentList<Point3DBasics> getFinalStepPositions(FootstepPlan plannedSteps)
   {
      QuadrantDependentList<Point3DBasics> finalSteps = new QuadrantDependentList<>();
      for (int i = plannedSteps.getNumberOfSteps() - 1; i >= 0; i--)
      {
         QuadrupedTimedStep step = plannedSteps.getFootstep(i);
         if (finalSteps.containsKey(step.getRobotQuadrant()))
            continue;
         else
            finalSteps.put(step.getRobotQuadrant(), step.getGoalPosition());
      }

      return finalSteps;
   }

   protected interface DatasetTestRunner
   {
      String testDataset(FootstepPlannerUnitTestDataset dataset);
   }

   private static interface ConditionChecker
   {
      boolean checkCondition();
   }
}
