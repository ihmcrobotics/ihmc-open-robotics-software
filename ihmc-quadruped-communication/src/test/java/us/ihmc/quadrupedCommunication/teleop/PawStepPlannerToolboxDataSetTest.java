package us.ihmc.quadrupedCommunication.teleop;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.PawStepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.starter.ApplicationRunner;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedCommunication.networkProcessing.pawPlanning.PawPlanningModule;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlan;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerType;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlanningResult;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerCommunicationProperties;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawStepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersBasics;
import us.ihmc.quadrupedFootstepPlanning.ui.PawStepPlannerUI;
import us.ihmc.quadrupedFootstepPlanning.ui.RemoteUIMessageConverter;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.DefaultPointFootSnapperParameters;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.ros2.RealtimeRos2Node;

public abstract class PawStepPlannerToolboxDataSetTest
{
   protected static final double bambooTimeScaling = 4.0;
   private static final double defaultBestEffortTimeout = 1.0;
//   private static final double defaultHorizonLength = 1.0;
   private static final double defaultHorizonLength = Double.POSITIVE_INFINITY;


   // Whether to start the UI or not.
   protected static boolean VISUALIZE = false;
   // For enabling helpful prints.
   protected static boolean DEBUG = false;
   protected static boolean VERBOSE = true;

   private PawStepPlannerUI ui = null;
   protected Messager messager = null;

   private QuadrupedXGaitSettingsReadOnly xGaitSettings = null;
   private PawPlanningModule footstepPlanningModule = null;

   private RealtimeRos2Node ros2Node;
   private RemoteUIMessageConverter converter;

   private final AtomicReference<PawStepPlan> plannerPlanReference = new AtomicReference<>(null);
   private final AtomicReference<PawStepPlanningResult> plannerResultReference = new AtomicReference<>(null);
   private final AtomicReference<Boolean> plannerReceivedPlan = new AtomicReference<>(false);
   private final AtomicReference<Boolean> plannerReceivedResult = new AtomicReference<>(false);

   private final AtomicReference<PawStepPlan> planReference = new AtomicReference<>(null);
   private final AtomicReference<PawStepPlanningResult> resultReference = new AtomicReference<>(null);

   private static final String robotName = "testBot";
   public static final PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;


   protected abstract PawStepPlannerType getPlannerType();

   protected abstract QuadrupedXGaitSettingsReadOnly getXGaitSettings();

   @BeforeEach
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (VISUALIZE)
         messager = new SharedMemoryJavaFXMessager(PawStepPlannerMessagerAPI.API);
      else
         messager = new SharedMemoryMessager(PawStepPlannerMessagerAPI.API);

      if (xGaitSettings == null)
         xGaitSettings = getXGaitSettings();

      VisibilityGraphsParametersBasics visibilityGraphsParameters = new DefaultVisibilityGraphParameters();
      visibilityGraphsParameters.setIncludePreferredExtrusions(false);
      visibilityGraphsParameters.setObstacleExtrusionDistance(0.6);
//      visibilityGraphsParameters.setPerformPostProcessingNodeShifting(true);
//      visibilityGraphsParameters.setComputeOrientationsToAvoidObstacles(true);

      PawStepPlannerParametersBasics parameters = new DefaultPawStepPlannerParameters();
      parameters.setXGaitWeight(0.0);
      parameters.setDesiredVelocityWeight(0.0);
      parameters.setYawWeight(2.0);
      parameters.setMaximumStepYawOutward(0.5);
      parameters.setMaximumStepYawInward(-0.5);
      parameters.setReturnBestEffortPlan(false);

      footstepPlanningModule = new PawPlanningModule(robotName, null, visibilityGraphsParameters, parameters, xGaitSettings,
                                                     new DefaultPointFootSnapperParameters(), null, false, false, pubSubImplementation);


      ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_footstep_planner_test");

      ROS2Tools.createCallbackSubscription(ros2Node, PawStepPlanningToolboxOutputStatus.class,
                                           PawStepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
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


      if (VISUALIZE)
      {
         createUI(messager);
      }

      messager.submitMessage(PawStepPlannerMessagerAPI.XGaitSettingsTopic, xGaitSettings);
      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerParametersTopic, parameters);
      messager.submitMessage(PawStepPlannerMessagerAPI.VisibilityGraphsParametersTopic, visibilityGraphsParameters);


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
            ui = PawStepPlannerUI.createMessagerUI(stage, (SharedMemoryJavaFXMessager) messager);
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
   public void testDataSets()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
                                                           {
                                                              if (!dataSet.hasPlannerInput())
                                                                 return false;
                                                              return dataSet.getPlannerInput().getQuadrupedPlannerIsTestable();
                                                           });
      runAssertionsOnAllDatasets(dataSets);
   }

   @Disabled
   @Test
   public void runInDevelopmentTests()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
                                                           {
                                                              if (!dataSet.hasPlannerInput())
                                                                 return false;
                                                              return dataSet.getPlannerInput().getQuadrupedPlannerIsInDevelopment();
                                                           });
      runAssertionsOnAllDatasets(dataSets);
   }

   private void runAssertionsOnAllDatasets(List<DataSet> allDatasets)
   {
      if (VERBOSE || DEBUG)
         LogTools.info("Unit test files found: " + allDatasets.size());

      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      int numberOfFailingTests = 0;
      List<String> failingDatasets = new ArrayList<>();
      List<String> failingMessages = new ArrayList<>();
      int numbberOfTestedSets = 0;
      for (int i = 0; i < allDatasets.size(); i++)
      {
         DataSet dataset = allDatasets.get(i);
         if (DEBUG || VERBOSE)
            LogTools.info("Testing file: " + dataset.getName());

         numbberOfTestedSets++;
         resetAllAtomics();
         String errorMessagesForCurrentFile = runAssertions(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
         {
            numberOfFailingTests++;
            failingDatasets.add(dataset.getName());
            failingMessages.add(errorMessagesForCurrentFile);
         }

         if (DEBUG || VERBOSE)
         {
            String result = errorMessagesForCurrentFile.isEmpty() ? "passed" : "failed";
            LogTools.info(dataset.getName() + " " + result);
         }

         ThreadTools.sleep(500); // Apparently need to give some time for the prints to appear in the right order.
      }

      String message = "Number of failing datasets: " + numberOfFailingTests + " out of " + numbberOfTestedSets;
      message += "\n Datasets failing: ";
      for (int i = 0; i < failingDatasets.size(); i++)
      {
         message += "\n" + failingDatasets.get(i) + " : " + failingMessages.get(i);
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

   protected String runAssertions(DataSetName dataSetName)
   {
      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);
      return runAssertions(dataSet);
   }

   protected String runAssertions(DataSet dataset)
   {
      resetAllAtomics();
      ThreadTools.sleep(1000);
      double timeout = packPlanningRequest(dataset);
      return findPlanAndAssertGoodResult(timeout);
   }

   protected double packPlanningRequest(DataSet dataset)
   {
      Quaternion startOrientation = new Quaternion();
      Quaternion goalOrientation = new Quaternion();
      if (dataset.getPlannerInput().getHasQuadrupedStartYaw())
         startOrientation.setYawPitchRoll(dataset.getPlannerInput().getQuadrupedStartYaw(), 0.0, 0.0);
      if (dataset.getPlannerInput().getHasQuadrupedGoalYaw())
         goalOrientation.setYawPitchRoll(dataset.getPlannerInput().getQuadrupedGoalYaw(), 0.0, 0.0);

      PlannerInput plannerInput = dataset.getPlannerInput();

      double timeMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double timeout = timeMultiplier * plannerInput.getQuadrupedTimeout();

      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerTypeTopic, getPlannerType());
      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerTimeoutTopic, timeout);
      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerBestEffortTimeoutTopic, defaultBestEffortTimeout);

      messager.submitMessage(PawStepPlannerMessagerAPI.XGaitSettingsTopic, xGaitSettings);
      messager.submitMessage(PawStepPlannerMessagerAPI.PlanarRegionDataTopic, dataset.getPlanarRegionsList());
      messager.submitMessage(PawStepPlannerMessagerAPI.StartPositionTopic, plannerInput.getQuadrupedStartPosition());
      messager.submitMessage(PawStepPlannerMessagerAPI.GoalPositionTopic, plannerInput.getQuadrupedGoalPosition());
      messager.submitMessage(PawStepPlannerMessagerAPI.StartOrientationTopic, startOrientation);
      messager.submitMessage(PawStepPlannerMessagerAPI.GoalOrientationTopic, goalOrientation);
      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerHorizonLengthTopic, defaultHorizonLength);

      ThreadTools.sleep(1000);

      messager.submitMessage(PawStepPlannerMessagerAPI.ComputePathTopic, true);


      //      planner.setHorizonLengthTopic(Double.MAX_VALUE);

      if (DEBUG)
         LogTools.info("Sending out planning request.");

      return timeout;
   }

   private void processFootstepPlanningOutputStatus(PawStepPlanningToolboxOutputStatus packet)
   {
      if (DEBUG)
         PrintTools.info("Processed an output from a remote planner.");

      plannerResultReference.set(PawStepPlanningResult.fromByte(packet.getFootstepPlanningResult()));
      plannerPlanReference.set(convertToFootstepPlan(packet));
      plannerReceivedPlan.set(true);
      plannerReceivedResult.set(true);
   }

   private static PawStepPlan convertToFootstepPlan(PawStepPlanningToolboxOutputStatus packet)
   {
      QuadrupedTimedStepListMessage footstepDataListMessage = packet.getFootstepDataList();
      PawStepPlan pawStepPlan = new PawStepPlan();

      for (QuadrupedTimedStepMessage footstepMessage : footstepDataListMessage.getQuadrupedStepList())
      {
         QuadrupedTimedOrientedStep step = new QuadrupedTimedOrientedStep();
         step.setGoalPosition(footstepMessage.getQuadrupedStepMessage().getGoalPosition());
         step.getTimeInterval().setInterval(footstepMessage.getTimeInterval().getStartTime(), footstepMessage.getTimeInterval().getEndTime());
         step.setGroundClearance(footstepMessage.getQuadrupedStepMessage().getGroundClearance());
         step.setRobotQuadrant(RobotQuadrant.fromByte(footstepMessage.getQuadrupedStepMessage().getRobotQuadrant()));

         pawStepPlan.addPawStep(step);
      }
      pawStepPlan.setLowLevelPlanGoal(new FramePose3D(ReferenceFrame.getWorldFrame(), packet.getLowLevelPlannerGoal()));

      return pawStepPlan;
   }

   private String findPlanAndAssertGoodResult(double timeout)
   {
      totalTimeTaken = 0.0;
      double maxTimeToWait = 2.0 * timeout;
      String datasetName = "";

      queryPlannerResults();

      String errorMessage = "";
      if (DEBUG)
         PrintTools.info("Waiting for result.");

      errorMessage += waitForResult(() -> resultReference.get() == null, maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

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

      PawStepPlanningResult result = this.resultReference.getAndSet(null);
      PawStepPlan plan = this.planReference.getAndSet(null);

      plannerReceivedPlan.set(false);
      plannerReceivedResult.set(false);

      errorMessage += assertPlanIsValid(datasetName, result, plan);
      for (int i = 0; i < 100; i++)
         ThreadTools.sleep(10);

      return errorMessage;
   }


   private double totalTimeTaken;

   private String waitForResult(BooleanSupplier waitCondition, double maxTimeToWait, String prefix)
   {
      String errorMessage = "";
      long waitTime = 10;
      while (waitCondition.getAsBoolean())
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

   private String validateResult(BooleanSupplier waitCondition, PawStepPlanningResult result, String prefix)
   {
      String errorMessage = "";

      if (!waitCondition.getAsBoolean())
      {
         errorMessage += prefix + " failed to find a valid result. Result : " + result + "\n";
      }

      return errorMessage;
   }

   private String waitForPlan(BooleanSupplier waitCondition, double maxTimeToWait, String prefix)
   {
      String errorMessage = "";

      while (waitCondition.getAsBoolean())
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


   private static String assertPlanIsValid(String datasetName, PawStepPlanningResult result, PawStepPlan plannedSteps)
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

      double nominalYaw = PawNode.computeNominalYaw(finalSteps.get(RobotQuadrant.FRONT_LEFT).getX(), finalSteps.get(RobotQuadrant.FRONT_LEFT).getY(),
                                                    finalSteps.get(RobotQuadrant.FRONT_RIGHT).getX(), finalSteps.get(RobotQuadrant.FRONT_RIGHT).getY(),
                                                    finalSteps.get(RobotQuadrant.HIND_LEFT).getX(), finalSteps.get(RobotQuadrant.HIND_LEFT).getY(),
                                                    finalSteps.get(RobotQuadrant.HIND_RIGHT).getX(), finalSteps.get(RobotQuadrant.HIND_RIGHT).getY());

      centerPoint.scale(0.25);

      Point3DReadOnly goalPosition = plannedSteps.getLowLevelPlanGoal().getPosition();
      double goalYaw = plannedSteps.getLowLevelPlanGoal().getYaw();
      if (goalPosition.distanceXY(centerPoint) > 4.0 * PawNode.gridSizeXY)
         errorMessage += datasetName + " did not reach goal position. Made it to " + centerPoint + ", trying to get to " + new Point3D(goalPosition);
      if (Double.isFinite(goalYaw))
      {
         if (AngleTools.computeAngleDifferenceMinusPiToPi(goalYaw, nominalYaw) > PawNode.gridSizeYaw)
            errorMessage += datasetName + " did not reach goal yaw. Made it to " + nominalYaw + ", trying to get to " + goalYaw;
      }

      errorMessage += checkStepOrder(datasetName, plannedSteps);

      if ((VISUALIZE || DEBUG) && !errorMessage.isEmpty())
         LogTools.error(errorMessage);

      return errorMessage;
   }

   private static QuadrantDependentList<Point3DBasics> getFinalStepPositions(PawStepPlan plannedSteps)
   {
      QuadrantDependentList<Point3DBasics> finalSteps = new QuadrantDependentList<>();
      for (int i = plannedSteps.getNumberOfSteps() - 1; i >= 0; i--)
      {
         QuadrupedTimedStep step = plannedSteps.getPawStep(i);
         if (finalSteps.containsKey(step.getRobotQuadrant()))
            continue;
         else
            finalSteps.put(step.getRobotQuadrant(), new Point3D(step.getGoalPosition()));
      }

      return finalSteps;
   }

   private static String checkStepOrder(String datasetName, PawStepPlan plannedSteps)
   {
      String errorMessage = "";
      RobotQuadrant previousMovingQuadrant = plannedSteps.getPawStep(0).getRobotQuadrant();
      for (int i = 1; i < plannedSteps.getNumberOfSteps(); i++)
      {
         RobotQuadrant movingQuadrant = plannedSteps.getPawStep(i).getRobotQuadrant();
         if (previousMovingQuadrant.getNextRegularGaitSwingQuadrant() != movingQuadrant)
            errorMessage += datasetName + " step " + i + " in the plan is out of order.\n";
         previousMovingQuadrant = movingQuadrant;
      }

      return errorMessage;
   }
}
