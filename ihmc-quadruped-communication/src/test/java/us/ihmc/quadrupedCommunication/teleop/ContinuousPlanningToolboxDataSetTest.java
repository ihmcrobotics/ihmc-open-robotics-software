package us.ihmc.quadrupedCommunication.teleop;

import controller_msgs.msg.dds.*;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedCommunication.networkProcessing.continuousPlanning.QuadrupedContinuousPlanningModule;
import us.ihmc.quadrupedCommunication.networkProcessing.footstepPlanning.QuadrupedFootstepPlanningModule;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlan;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerType;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlanningResult;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.ui.ApplicationRunner;
import us.ihmc.quadrupedFootstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.quadrupedFootstepPlanning.ui.RemoteUIMessageConverter;
import us.ihmc.quadrupedPlanning.QuadrupedGait;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.DefaultPointFootSnapperParameters;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.quadTree.QuadTreeForGround;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;
import static us.ihmc.robotics.Assert.assertTrue;

public class ContinuousPlanningToolboxDataSetTest
{
   private static final double defaultBestEffortTimeout = 1.0;
//   private static final double defaultHorizonLength = 1.0;
   private static final double defaultHorizonLength = Double.POSITIVE_INFINITY;

   private static final double dt = 0.01;


   // Whether to start the UI or not.
   protected static boolean VISUALIZE = false;
   // For enabling helpful prints.
   protected static boolean DEBUG = false;
   protected static boolean VERBOSE = true;

   private FootstepPlannerUI ui = null;
   protected Messager messager = null;

   private QuadrupedXGaitSettingsReadOnly xGaitSettings = null;
   private QuadrupedFootstepPlanningModule footstepPlanningModule = null;
   private QuadrupedContinuousPlanningModule continuousPlanningModule = null;

   private RealtimeRos2Node ros2Node;

   private final AtomicReference<List<QuadrupedTimedStep>> stepListReference = new AtomicReference<>(null);
   private final AtomicReference<Boolean> receivedStepList = new AtomicReference<>(false);

   private static final String robotName = "testBot";
   public static final PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;

   private IHMCRealtimeROS2Publisher<QuadrupedContinuousPlanningRequestPacket> requestPublisher;
   private IHMCRealtimeROS2Publisher<PlanarRegionsListMessage> planarRegionsPublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedFootstepStatusMessage> footstepStatusPublisher;


   public QuadrupedXGaitSettingsReadOnly getXGaitSettings()
   {
      QuadrupedXGaitSettings settings = new QuadrupedXGaitSettings();
      settings.setStanceLength(1.0);
      settings.setStanceWidth(0.5);

      settings.setQuadrupedSpeed(QuadrupedSpeed.MEDIUM);
      settings.setEndPhaseShift(QuadrupedGait.AMBLE.getEndPhaseShift());
      settings.getAmbleMediumTimings().setEndDoubleSupportDuration(0.25);
      settings.getAmbleMediumTimings().setStepDuration(0.5);
      settings.getAmbleMediumTimings().setMaxSpeed(0.3);
      return settings;
   }

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

      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters()
      {
         @Override
         public double getXGaitWeight()
         {
            return 0.0;
         }

         @Override
         public double getDesiredVelocityWeight()
         {
            return 0.0;
         }
      };

      footstepPlanningModule = new QuadrupedFootstepPlanningModule(robotName, null, parameters, xGaitSettings,
                                                                   new DefaultPointFootSnapperParameters(), null, false, false, pubSubImplementation);
      continuousPlanningModule = new QuadrupedContinuousPlanningModule(robotName, null, null, false, false, pubSubImplementation);


      ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_footstep_planner_test");

      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedFootstepPlanningToolboxOutputStatus.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedTimedStepListMessage.class,
                                           getTopicNameGenerator(robotName, ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX, ROS2TopicQualifier.OUTPUT),
                                           s -> processTimedStepListMessage(s.takeNextData()));

      requestPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedContinuousPlanningRequestPacket.class,
                                                   getTopicNameGenerator(robotName, ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT));
      planarRegionsPublisher = ROS2Tools.createPublisher(ros2Node, PlanarRegionsListMessage.class,
                                                   getTopicNameGenerator(robotName, ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT));

      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      footstepStatusPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedFootstepStatusMessage.class, controllerPubGenerator);

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

      messager.submitMessage(FootstepPlannerMessagerAPI.XGaitSettingsTopic, xGaitSettings);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParametersTopic, parameters);


      ThreadTools.sleep(1000);
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      messager.closeMessager();
      footstepPlanningModule.destroy();
      continuousPlanningModule.destroy();
      if (ui != null)
         ui.stop();

      footstepPlanningModule = null;
      continuousPlanningModule = null;
      requestPublisher = null;
      planarRegionsPublisher = null;
      footstepStatusPublisher = null;
      ui = null;
      messager = null;
   }

   private void resetAllAtomics()
   {
      stepListReference.set(null);
      receivedStepList.set(false);
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

   private String runAssertions(DataSet dataset)
   {
      resetAllAtomics();
      ThreadTools.sleep(1000);
      broadcastPlanningRequest(dataset);
      return simulateWalkingAlongThePathAndAssertGoodResults(dataset);
   }

   private void broadcastPlanningRequest(DataSet dataset)
   {
      Quaternion startOrientation = new Quaternion();
      Quaternion goalOrientation = new Quaternion();
      if (dataset.getPlannerInput().getHasQuadrupedStartYaw())
         startOrientation.setYawPitchRoll(dataset.getPlannerInput().getQuadrupedStartYaw(), 0.0, 0.0);
      if (dataset.getPlannerInput().getHasQuadrupedGoalYaw())
         goalOrientation.setYawPitchRoll(dataset.getPlannerInput().getQuadrupedGoalYaw(), 0.0, 0.0);

      PlannerInput plannerInput = dataset.getPlannerInput();

      QuadrupedContinuousPlanningRequestPacket requestPacket = new QuadrupedContinuousPlanningRequestPacket();
      requestPacket.setHorizonLength(defaultHorizonLength);
      requestPacket.setTimeout(defaultBestEffortTimeout);
      requestPacket.getGoalOrientationInWorld().set(goalOrientation);
      requestPacket.getGoalPositionInWorld().set(plannerInput.getQuadrupedGoalPosition());

      requestPublisher.publish(requestPacket);

      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(dataset.getPlanarRegionsList());

      planarRegionsPublisher.publish(planarRegionsListMessage);

      if (DEBUG)
         LogTools.info("Sending out planning request.");
   }

   private void processFootstepPlanningOutputStatus(QuadrupedFootstepPlanningToolboxOutputStatus packet)
   {
      if (DEBUG)
         PrintTools.info("Processed an output from a remote planner.");

      FootstepPlan footstepPlan = convertToFootstepPlan(packet);

      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanTopic, footstepPlan);
   }

   private void processTimedStepListMessage(QuadrupedTimedStepListMessage packet)
   {
      if (DEBUG)
         PrintTools.info("Processed an output from a remote planner.");

      QuadrupedTimedStepListCommand stepListCommand = new QuadrupedTimedStepListCommand();
      stepListCommand.setFromMessage(packet);

      List<QuadrupedTimedStep> stepList = new ArrayList<>();
      for (int i = 0; i < stepListCommand.getNumberOfSteps(); i++)
      {
         QuadrupedTimedStep step = new QuadrupedTimedStep();
         step.set(stepListCommand.getStepCommands().get(i));
         stepList.add(step);
      }

      stepListReference.set(stepList);
      receivedStepList.set(true);
   }

   private static FootstepPlan convertToFootstepPlan(QuadrupedFootstepPlanningToolboxOutputStatus packet)
   {
      QuadrupedTimedStepListMessage footstepDataListMessage = packet.getFootstepDataList();
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
      footstepPlan.setLowLevelPlanGoal(new FramePose3D(ReferenceFrame.getWorldFrame(), packet.getLowLevelPlannerGoal()));

      return footstepPlan;
   }

   private String simulateWalkingAlongThePathAndAssertGoodResults(DataSet dataSet)
   {
      QuadrantDependentList<Point3D> feetPositions = new QuadrantDependentList<>();
      Quaternion startOrientation = new Quaternion();
      if (dataSet.getPlannerInput().hasStartOrientation())
         startOrientation.setToYawQuaternion(dataSet.getPlannerInput().getQuadrupedStartYaw());
      PoseReferenceFrame startFrame = new PoseReferenceFrame("startFrame", ReferenceFrame.getWorldFrame());
      startFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), dataSet.getPlannerInput().getQuadrupedStartPosition()));
      startFrame.setOrientationAndUpdate(startOrientation);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D footPosition = new FramePoint3D(startFrame);
         footPosition.setX(robotQuadrant.getEnd().negateIfHindEnd(0.5 * xGaitSettings.getStanceLength()));
         footPosition.setY(robotQuadrant.getSide().negateIfRightSide(0.5 * xGaitSettings.getStanceWidth()));
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());

         feetPositions.put(robotQuadrant, new Point3D(footPosition));
      }

      List<QuadrupedTimedStep> stepsCurrentlyInProgress = new ArrayList<>();

      double tickStartTime = Conversions.nanosecondsToSeconds(System.nanoTime());
      boolean firstTick = true;
      double time = 0.0;
      while (!continuousPlanningModule.getToolboxController().isDone())
      {
         double currentTime = Conversions.nanosecondsToSeconds(System.nanoTime());

         if (!firstTick && (currentTime - tickStartTime < dt))
            ThreadTools.sleep(10);

         if (!receivedStepList.get())
            continue;


         List<QuadrupedTimedStep> stepList = stepListReference.get();
         List<QuadrupedTimedStep> stepsInProgress = new ArrayList<>();
         for (int i = 0; i < stepList.size(); i++)
         {
            if (stepList.get(i).getTimeInterval().intervalContains(time))
               stepsInProgress.add(stepList.get(i));
         }

         if (receivedStepList.get())
         {
            receivedStepList.set(false);

            String errorMessage = assertPlanIsValid(dataSet, stepList);
            if (!errorMessage.isEmpty())
               return errorMessage;
         }


         List<QuadrupedTimedStep> stepsJustStarted = stepsInProgress.stream().filter(step -> !stepsCurrentlyInProgress.contains(step)).collect(Collectors.toList());
         List<QuadrupedTimedStep> stepsJustFinished = stepsCurrentlyInProgress.stream().filter(step -> !stepsInProgress.contains(step)).collect(Collectors.toList());

         for (QuadrupedTimedStep stepJustStarted : stepsJustStarted)
         {
            QuadrupedFootstepStatusMessage statusMessage = new QuadrupedFootstepStatusMessage();
            statusMessage.setFootstepQuadrant(stepJustStarted.getRobotQuadrant().toByte());
            statusMessage.getDesiredStepInterval().setStartTime(stepJustStarted.getTimeInterval().getStartTime());
            statusMessage.getDesiredStepInterval().setEndTime(stepJustStarted.getTimeInterval().getEndTime());
            statusMessage.setFootstepStatus(QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_STARTED);
            statusMessage.getDesiredTouchdownPositionInWorld().set(stepJustStarted.getGoalPosition());

            footstepStatusPublisher.publish(statusMessage);

            feetPositions.remove(stepJustStarted.getRobotQuadrant());
         }

         for (QuadrupedTimedStep stepJustFinished : stepsJustFinished)
         {
            QuadrupedFootstepStatusMessage statusMessage = new QuadrupedFootstepStatusMessage();
            statusMessage.setFootstepQuadrant(stepJustFinished.getRobotQuadrant().toByte());
            statusMessage.getDesiredStepInterval().setStartTime(stepJustFinished.getTimeInterval().getStartTime());
            statusMessage.getDesiredStepInterval().setEndTime(stepJustFinished.getTimeInterval().getEndTime());
            statusMessage.setFootstepStatus(QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED);
            statusMessage.getDesiredTouchdownPositionInWorld().set(stepJustFinished.getGoalPosition());
            statusMessage.getActualStepInterval().setStartTime(stepJustFinished.getTimeInterval().getStartTime());
            statusMessage.getActualStepInterval().setEndTime(stepJustFinished.getTimeInterval().getEndTime());
            statusMessage.getActualTouchdownPositionInWorld().set(stepJustFinished.getGoalPosition());

            footstepStatusPublisher.publish(statusMessage);

            feetPositions.put(stepJustFinished.getRobotQuadrant(), new Point3D(stepJustFinished.getGoalPosition()));
         }

         messager.submitMessage(FootstepPlannerMessagerAPI.StartFeetPositionTopic, feetPositions);


         stepsCurrentlyInProgress.clear();
         stepsCurrentlyInProgress.addAll(stepsInProgress);

         firstTick = false;
         tickStartTime = currentTime;
         time += dt;
      }

      return "";
   }




   private static String assertPlanIsValid(DataSet dataSet, List<QuadrupedTimedStep> plannedSteps)
   {
      QuadrantDependentList<Point3DBasics> finalSteps = getFinalStepPositions(plannedSteps);

      String datasetName = dataSet.getName();
      String errorMessage = "";


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

      Point3DReadOnly goalPosition = dataSet.getPlannerInput().getGoalPosition();
      double goalYaw = dataSet.getPlannerInput().getQuadrupedGoalYaw();
      if (goalPosition.distanceXY(centerPoint) > 3.0 * FootstepNode.gridSizeXY)
         errorMessage += datasetName + " did not reach goal position. Made it to " + centerPoint + ", trying to get to " + new Point3D(goalPosition);
      if (Double.isFinite(goalYaw))
      {
         if (AngleTools.computeAngleDifferenceMinusPiToPi(goalYaw, nominalYaw) > FootstepNode.gridSizeYaw)
            errorMessage += datasetName + " did not reach goal yaw. Made it to " + nominalYaw + ", trying to get to " + goalYaw;
      }

      errorMessage += checkStepOrder(datasetName, plannedSteps);

      if ((VISUALIZE || DEBUG) && !errorMessage.isEmpty())
         LogTools.error(errorMessage);

      return errorMessage;
   }

   private static QuadrantDependentList<Point3DBasics> getFinalStepPositions(List<QuadrupedTimedStep> plannedSteps)
   {
      QuadrantDependentList<Point3DBasics> finalSteps = new QuadrantDependentList<>();
      for (int i = plannedSteps.size() - 1; i >= 0; i--)
      {
         QuadrupedTimedStep step = plannedSteps.get(i);
         if (finalSteps.containsKey(step.getRobotQuadrant()))
            continue;
         else
            finalSteps.put(step.getRobotQuadrant(), new Point3D(step.getGoalPosition()));
      }

      return finalSteps;
   }

   private static String checkStepOrder(String datasetName, List<QuadrupedTimedStep> plannedSteps)
   {
      String errorMessage = "";
      RobotQuadrant previousMovingQuadrant = plannedSteps.get(0).getRobotQuadrant();
      for (int i = 1; i < plannedSteps.size(); i++)
      {
         RobotQuadrant movingQuadrant = plannedSteps.get(i).getRobotQuadrant();
         if (previousMovingQuadrant.getNextRegularGaitSwingQuadrant() != movingQuadrant)
            errorMessage += datasetName + " step " + i + " in the plan is out of order.\n";
         previousMovingQuadrant = movingQuadrant;
      }

      return errorMessage;
   }

   public static void main(String[] args) throws Exception
   {
      ContinuousPlanningToolboxDataSetTest test = new ContinuousPlanningToolboxDataSetTest();
      VISUALIZE = true;
      test.setup();

      String errorMessage = test.runAssertions(DataSetName._20171218_204953_FlatGroundWithWall);
      assertTrue(errorMessage, errorMessage.isEmpty());

      ThreadTools.sleepForever();
      test.tearDown();
   }
}
