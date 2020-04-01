package us.ihmc.quadrupedCommunication.teleop;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;
import static us.ihmc.robotics.Assert.assertTrue;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import com.google.common.util.concurrent.AtomicDouble;

import controller_msgs.msg.dds.PawStepPlannerParametersPacket;
import controller_msgs.msg.dds.PawStepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.QuadrupedContinuousPlanningRequestPacket;
import controller_msgs.msg.dds.QuadrupedFootstepStatusMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
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
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedCommunication.networkProcessing.continuousPlanning.QuadrupedContinuousPlanningModule;
import us.ihmc.quadrupedCommunication.networkProcessing.pawPlanning.PawPlanningModule;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlan;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerTargetType;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerCommunicationProperties;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawStepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersBasics;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.quadrupedFootstepPlanning.ui.PawStepPlannerUI;
import us.ihmc.quadrupedPlanning.QuadrupedGait;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.DefaultPointFootSnapperParameters;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ContinuousPlanningToolboxDataSetTest
{
   private static final double defaultBestEffortTimeout = 0.25;
   //   private static final double defaultHorizonLength = 1.0;
   private static final double defaultHorizonLength = 1.0;

   private static final double dt = 0.05;
   private static double timeScaleFactor;

   private final DecimalFormat numberFormat = new DecimalFormat("#.00");

   /**
    * Sub-selection of all the datasets that are quick enough to run in the fast build.
    */
   private static final List<DataSetName> fastDatasets = Arrays.asList(DataSetName._20190514_163532_QuadrupedShortPlatformEnvironment,
                                                                       DataSetName._20190514_163532_QuadrupedPlatformEnvironment,
                                                                       DataSetName._20171114_135559_PartialShallowMaze,
                                                                       DataSetName._20171115_171243_SimplePlaneAndWall,
                                                                       DataSetName._20171215_201810_RampSteppingStones_Sim);

   private static final List<DataSetName> datasetsToIgnore = new ArrayList<>();
   static
   {
      datasetsToIgnore.add(DataSetName._20171216_111326_CrossoverPlatforms);
   }

   // Whether to start the UI or not.
   protected static boolean VISUALIZE = false;
   // For enabling helpful prints.
   protected static boolean DEBUG = false;
   protected static boolean VERBOSE = false;

   private PawStepPlannerUI ui = null;
   protected Messager messager = null;

   private QuadrupedXGaitSettingsReadOnly xGaitSettings = null;
   private PawPlanningModule footstepPlanningModule = null;
   private VisibilityGraphsParametersBasics visibilityGraphsParameters = null;
   private PawStepPlannerParametersBasics footstepPlannerParameters = null;
   private QuadrupedContinuousPlanningModule continuousPlanningModule = null;

   private RealtimeRos2Node ros2Node;

   private final AtomicReference<PawStepPlan> planReference = new AtomicReference<>(null);
   private final AtomicReference<List<Pose3D>> pathReference = new AtomicReference<>(null);
   private final AtomicReference<List<QuadrupedTimedStep>> stepListReference = new AtomicReference<>(null);
   private final AtomicReference<Boolean> receivedStepList = new AtomicReference<>(false);
   private final AtomicDouble timeReference = new AtomicDouble();

   private static final String robotName = "testBot";
   public static final PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;

   private IHMCRealtimeROS2Publisher<QuadrupedContinuousPlanningRequestPacket> requestPublisher;
   private IHMCRealtimeROS2Publisher<PlanarRegionsListMessage> planarRegionsPublisher;
   private IHMCRealtimeROS2Publisher<QuadrupedFootstepStatusMessage> footstepStatusPublisher;
   private IHMCRealtimeROS2Publisher<PawStepPlannerParametersPacket> plannerParametersPublisher;
   private YoBoolean planningFailed;

   public QuadrupedXGaitSettingsReadOnly getXGaitSettings()
   {
      QuadrupedXGaitSettings settings = new QuadrupedXGaitSettings();
      settings.setStanceLength(1.0);
      settings.setStanceWidth(0.5);

      settings.setQuadrupedSpeed(QuadrupedSpeed.MEDIUM);
      settings.setEndPhaseShift(QuadrupedGait.TROT.getEndPhaseShift());
      settings.getAmbleMediumTimings().setEndDoubleSupportDuration(0.25);
      settings.getAmbleMediumTimings().setStepDuration(0.5);
      settings.getAmbleMediumTimings().setMaxSpeed(0.3);
      settings.getTrotMediumTimings().setEndDoubleSupportDuration(0.25);
      settings.getTrotMediumTimings().setStepDuration(0.5);
      settings.getTrotMediumTimings().setMaxSpeed(0.3);
      return settings;
   }

   public VisibilityGraphsParametersBasics getVisibilityGraphsParameters()
   {
      VisibilityGraphsParametersBasics parameters = new DefaultVisibilityGraphParameters();
//      parameters.setPerformPostProcessingNodeShifting(true);
      parameters.setComputeOrientationsToAvoidObstacles(false);
      parameters.setReturnBestEffortSolution(true);
      return parameters;
   }

   public PawStepPlannerParametersBasics getFootstepPlannerParameters()
   {
      PawStepPlannerParametersBasics parameters = new DefaultPawStepPlannerParameters();
      parameters.setXGaitWeight(10.0);
      parameters.setDesiredVelocityWeight(0.0);
      parameters.setReturnBestEffortPlan(true);
      parameters.setPerformGraphRepairingStep(true);
      parameters.setMinimumStepsForBestEffortPlan(6);

      return parameters;
   }

   @BeforeEach
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      timeScaleFactor = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? timeScaleFactor = 10.0 : 10.0;

      if (VISUALIZE)
         messager = new SharedMemoryJavaFXMessager(PawStepPlannerMessagerAPI.API);
      else
         messager = new SharedMemoryMessager(PawStepPlannerMessagerAPI.API);

      if (xGaitSettings == null)
         xGaitSettings = getXGaitSettings();

      if (visibilityGraphsParameters == null)
         visibilityGraphsParameters = getVisibilityGraphsParameters();

      if (footstepPlannerParameters == null)
         footstepPlannerParameters = getFootstepPlannerParameters();

      footstepPlanningModule = new PawPlanningModule(robotName,
                                                     null,
                                                     visibilityGraphsParameters,
                                                     footstepPlannerParameters,
                                                     xGaitSettings,
                                                     new DefaultPointFootSnapperParameters(),
                                                     null,
                                                     false,
                                                     false,
                                                     pubSubImplementation);
      YoVariableRegistry testRegistry = new YoVariableRegistry("testRegistry");
      continuousPlanningModule = new QuadrupedContinuousPlanningModule(robotName, null, xGaitSettings, null, false, false, pubSubImplementation);
      continuousPlanningModule.setRootRegistry(testRegistry, null);
      planningFailed = ((YoBoolean) testRegistry.getVariable("planningFailed"));

      ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_footstep_planner_test");

      ROS2Tools.createCallbackSubscription(ros2Node,
                                           PawStepPlanningToolboxOutputStatus.class,
                                           PawStepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           QuadrupedTimedStepListMessage.class,
                                           getTopicNameGenerator(robotName, ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX, ROS2TopicQualifier.OUTPUT),
                                           s -> processTimedStepListMessage(s.takeNextData()));

      requestPublisher = ROS2Tools.createPublisher(ros2Node,
                                                   QuadrupedContinuousPlanningRequestPacket.class,
                                                   getTopicNameGenerator(robotName, ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT));
      planarRegionsPublisher = ROS2Tools.createPublisher(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator);
      plannerParametersPublisher = ROS2Tools.createPublisher(ros2Node,
                                                             PawStepPlannerParametersPacket.class,
                                                             getTopicNameGenerator(robotName,
                                                                                   ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX,
                                                                                   ROS2Tools.ROS2TopicQualifier.INPUT));

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

      messager.submitMessage(PawStepPlannerMessagerAPI.XGaitSettingsTopic, xGaitSettings);
      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerParametersTopic, footstepPlannerParameters);

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
      plannerParametersPublisher = null;
      ui = null;
      messager = null;
   }

   private void resetAllAtomics()
   {
      stepListReference.set(null);
      pathReference.set(null);
      planReference.set(null);
      receivedStepList.set(false);
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

   @Disabled
   @Test
   @Tag("fast")
   public void testFewDataSets()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
      {
         if (!dataSet.hasPlannerInput() || !dataSet.getPlannerInput().getQuadrupedPlannerIsTestable())
            return false;
         for (DataSetName nameToIgnore : fastDatasets)
         {
            if (dataSet.getName().equals(nameToIgnore.name().substring(1)))
               return true;
         }

         return false;
      });
      runAssertionsOnAllDatasets(dataSets);
   }

   @Disabled
   @Test
   @Tag("quad-comm-slow")
   public void testAllDataSets()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
      {
         if (!dataSet.hasPlannerInput())
            return false;
         for (DataSetName nameToIgnore : datasetsToIgnore)
         {
            if (dataSet.getName().equals(nameToIgnore.name().substring(1)))
               return false;
         }
         for (DataSetName nameToIgnore : fastDatasets)
         {
            if (dataSet.getName().equals(nameToIgnore.name().substring(1)))
               return false;
         }

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
      int numberOfTestedSets = 0;
      for (int i = 0; i < allDatasets.size(); i++)
      {
         DataSet dataset = allDatasets.get(i);
         long startTime = System.nanoTime();
         if (DEBUG || VERBOSE)
            LogTools.info("Testing file: " + dataset.getName());

         numberOfTestedSets++;
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
            long endTime = System.nanoTime();
            String result = errorMessagesForCurrentFile.isEmpty() ? "passed" : "failed";
            LogTools.info(dataset.getName() + " " + result + ", took: " + Conversions.nanosecondsToSeconds(endTime - startTime) + "sec.");
            if (!errorMessagesForCurrentFile.isEmpty())
               LogTools.info(errorMessagesForCurrentFile);
         }

         ThreadTools.sleep(100); // Apparently need to give some time for the prints to appear in the right order.
      }

      String message = "Number of failing datasets: " + numberOfFailingTests + " out of " + numberOfTestedSets;
      message += "\n Datasets failing: ";
      for (int i = 0; i < failingDatasets.size(); i++)
      {
         message += "\n" + failingDatasets.get(i) + " : " + failingMessages.get(i);
      }
      if (VISUALIZE)
      {
         LogTools.info(message);
         //         ThreadTools.sleepForever();
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
      ThreadTools.sleep(100);
      broadcastPlanningRequest(dataset);
      ThreadTools.sleep(100);

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

      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(dataset.getPlanarRegionsList());

      planarRegionsPublisher.publish(planarRegionsListMessage);
      plannerParametersPublisher.publish(footstepPlannerParameters.getAsPacket());

      messager.submitMessage(PawStepPlannerMessagerAPI.PlanarRegionDataTopic,
                             PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerParametersTopic, footstepPlannerParameters);

      ThreadTools.sleep(100);

      QuadrupedContinuousPlanningRequestPacket requestPacket = new QuadrupedContinuousPlanningRequestPacket();
      requestPacket.setHorizonLength(defaultHorizonLength);
      System.out.println(dataset.getPlannerInput().getQuadrupedTimeout());
      requestPacket.setTimeout(dataset.getPlannerInput().getQuadrupedTimeout());
      requestPacket.setBestEffortTimeout(defaultBestEffortTimeout);
      requestPacket.getGoalOrientationInWorld().set(goalOrientation);
      requestPacket.getGoalPositionInWorld().set(plannerInput.getQuadrupedGoalPosition());

      QuadrantDependentList<Point3D> feetPositions = new QuadrantDependentList<>();
      PoseReferenceFrame startFrame = new PoseReferenceFrame("startFrame", ReferenceFrame.getWorldFrame());
      startFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), dataset.getPlannerInput().getQuadrupedStartPosition()));
      startFrame.setOrientationAndUpdate(startOrientation);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D footPosition = new FramePoint3D(startFrame);
         footPosition.setX(robotQuadrant.getEnd().negateIfHindEnd(0.5 * xGaitSettings.getStanceLength()));
         footPosition.setY(robotQuadrant.getSide().negateIfRightSide(0.5 * xGaitSettings.getStanceWidth()));
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());

         feetPositions.put(robotQuadrant, new Point3D(footPosition));
      }

      requestPacket.getFrontLeftStartPositionInWorld().set(feetPositions.get(RobotQuadrant.FRONT_LEFT));
      requestPacket.getFrontRightStartPositionInWorld().set(feetPositions.get(RobotQuadrant.FRONT_RIGHT));
      requestPacket.getHindLeftStartPositionInWorld().set(feetPositions.get(RobotQuadrant.HIND_LEFT));
      requestPacket.getHindRightStartPositionInWorld().set(feetPositions.get(RobotQuadrant.HIND_RIGHT));

      requestPublisher.publish(requestPacket);

      messager.submitMessage(PawStepPlannerMessagerAPI.GoalPositionTopic, dataset.getPlannerInput().getQuadrupedGoalPosition());
      messager.submitMessage(PawStepPlannerMessagerAPI.GoalOrientationTopic, goalOrientation);
      messager.submitMessage(PawStepPlannerMessagerAPI.StartPositionTopic, dataset.getPlannerInput().getQuadrupedStartPosition());
      messager.submitMessage(PawStepPlannerMessagerAPI.StartOrientationTopic, startOrientation);
      messager.submitMessage(PawStepPlannerMessagerAPI.StartFeetPositionTopic, feetPositions);
      messager.submitMessage(PawStepPlannerMessagerAPI.StartTargetTypeTopic, PawStepPlannerTargetType.FOOTSTEPS);

      if (DEBUG)
         LogTools.info("Sending out planning request.");
   }

   private void processFootstepPlanningOutputStatus(PawStepPlanningToolboxOutputStatus packet)
   {
      if (DEBUG)
         PrintTools.info("Processed an output from a remote planner.");

      PawStepPlan footstepPlan = convertToFootstepPlan(packet);
      pathReference.set(packet.getBodyPath());
      planReference.set(footstepPlan);

      messager.submitMessage(PawStepPlannerMessagerAPI.LowLevelGoalPositionTopic, packet.getLowLevelPlannerGoal().getPosition());
      messager.submitMessage(PawStepPlannerMessagerAPI.LowLevelGoalOrientationTopic, packet.getLowLevelPlannerGoal().getOrientation());
      messager.submitMessage(PawStepPlannerMessagerAPI.BodyPathDataTopic, packet.getBodyPath());
      messager.submitMessage(PawStepPlannerMessagerAPI.FootstepPlanTopic, footstepPlan);
      messager.submitMessage(PawStepPlannerMessagerAPI.PlanarRegionDataTopic,
                             PlanarRegionMessageConverter.convertToPlanarRegionsList(packet.getPlanarRegionsList()));
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
         if (!packet.getIsExpressedInAbsoluteTime())
            step.getTimeInterval().shiftInterval(timeReference.get());
         stepList.add(step);
      }

      stepListReference.set(stepList);
      receivedStepList.set(true);
   }

   private static PawStepPlan convertToFootstepPlan(PawStepPlanningToolboxOutputStatus packet)
   {
      QuadrupedTimedStepListMessage footstepDataListMessage = packet.getFootstepDataList();
      PawStepPlan footstepPlan = new PawStepPlan();

      for (QuadrupedTimedStepMessage footstepMessage : footstepDataListMessage.getQuadrupedStepList())
      {
         QuadrupedTimedOrientedStep step = new QuadrupedTimedOrientedStep();
         step.setGoalPosition(footstepMessage.getQuadrupedStepMessage().getGoalPosition());
         step.getTimeInterval().setInterval(footstepMessage.getTimeInterval().getStartTime(), footstepMessage.getTimeInterval().getEndTime());
         step.setGroundClearance(footstepMessage.getQuadrupedStepMessage().getGroundClearance());
         step.setRobotQuadrant(RobotQuadrant.fromByte(footstepMessage.getQuadrupedStepMessage().getRobotQuadrant()));

         footstepPlan.addPawStep(step);
      }
      footstepPlan.setLowLevelPlanGoal(new FramePose3D(ReferenceFrame.getWorldFrame(), packet.getLowLevelPlannerGoal()));

      return footstepPlan;
   }

   private String simulateWalkingAlongThePathAndAssertGoodResults(DataSet dataSet)
   {
      QuadrantDependentList<FramePoint3D> feetPositions = new QuadrantDependentList<>();
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

         feetPositions.put(robotQuadrant, footPosition);
      }

      List<QuadrupedTimedStep> stepsCurrentlyInProgress = new ArrayList<>();

      double tickStartTime = Conversions.nanosecondsToSeconds(System.nanoTime());
      boolean firstTick = true;
      timeReference.set(0.0);

      double expectedDuration = (dataSet.getPlannerInput().getStartPosition().distanceXY(dataSet.getPlannerInput().getGoalPosition()))
            / xGaitSettings.getMaxSpeed();
      double maxDuration = 4.0 * expectedDuration;
      double absoluteMaxDuration = 10.0 * expectedDuration;
      boolean timedOut = false;
      String message = "";
      double startTime = Conversions.nanosecondsToSeconds(System.nanoTime());
      while (!continuousPlanningModule.getToolboxController().isDone() && !planningFailed.getBooleanValue() && !timedOut)
      {
         double currentTime = Conversions.nanosecondsToSeconds(System.nanoTime());
         if (currentTime - startTime > absoluteMaxDuration)
         {
            message += "Hit an absolute timeout.";
            break;
         }

         if (!firstTick && ((currentTime - tickStartTime) < (dt / timeScaleFactor)))
         {
            ThreadTools.sleep(100);
            continue;
         }

         if (!receivedStepList.get() || pathReference.get() == null)
            continue;

         List<QuadrupedTimedStep> stepList = stepListReference.get();
         List<QuadrupedTimedStep> stepsInProgress = new ArrayList<>();
         for (int i = 0; i < stepList.size(); i++)
         {
            if (stepList.get(i).getTimeInterval().intervalContains(timeReference.get()))
               stepsInProgress.add(stepList.get(i));
         }

         String newMessage = assertPlanIsValid(dataSet, pathReference.get(), stepList, xGaitSettings, footstepPlannerParameters);
         if (!newMessage.isEmpty())
         {
            DecimalFormat numberFormat = new DecimalFormat("#.00");
            message += "\nAt time " + numberFormat.format(timeReference.get()) + " " + newMessage;
         }

         List<QuadrupedTimedStep> stepsJustStarted = stepsInProgress.stream()
                                                                    .filter(step -> !hasQuadrantInProgress(step.getRobotQuadrant(), stepsCurrentlyInProgress))
                                                                    .collect(Collectors.toList());
         List<QuadrupedTimedStep> stepsJustFinished = stepsCurrentlyInProgress.stream()
                                                                              .filter(step -> !hasQuadrantInProgress(step.getRobotQuadrant(), stepsInProgress))
                                                                              .collect(Collectors.toList());

         for (QuadrupedTimedStep stepJustStarted : stepsJustStarted)
         {
            QuadrupedFootstepStatusMessage statusMessage = new QuadrupedFootstepStatusMessage();
            statusMessage.setRobotQuadrant(stepJustStarted.getRobotQuadrant().toByte());
            statusMessage.getDesiredStepInterval().setStartTime(stepJustStarted.getTimeInterval().getStartTime());
            statusMessage.getDesiredStepInterval().setEndTime(stepJustStarted.getTimeInterval().getEndTime());
            statusMessage.setFootstepStatus(QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_STARTED);
            statusMessage.getDesiredTouchdownPositionInWorld().set(stepJustStarted.getGoalPosition());

            if (DEBUG)
            {
               LogTools.info("Just started at t = " + timeReference.get() + " step " + stepJustStarted.getRobotQuadrant() + " : "
                     + stepJustStarted.getGoalPosition() + " Time: " + stepJustStarted.getTimeInterval());
            }

            footstepStatusPublisher.publish(statusMessage);

            feetPositions.remove(stepJustStarted.getRobotQuadrant());
         }

         for (QuadrupedTimedStep stepJustFinished : stepsJustFinished)
         {
            QuadrupedFootstepStatusMessage statusMessage = new QuadrupedFootstepStatusMessage();
            statusMessage.setRobotQuadrant(stepJustFinished.getRobotQuadrant().toByte());
            statusMessage.getDesiredStepInterval().setStartTime(stepJustFinished.getTimeInterval().getStartTime());
            statusMessage.getDesiredStepInterval().setEndTime(stepJustFinished.getTimeInterval().getEndTime());
            statusMessage.setFootstepStatus(QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED);
            statusMessage.getDesiredTouchdownPositionInWorld().set(stepJustFinished.getGoalPosition());
            statusMessage.getActualStepInterval().setStartTime(stepJustFinished.getTimeInterval().getStartTime());
            statusMessage.getActualStepInterval().setEndTime(stepJustFinished.getTimeInterval().getEndTime());
            statusMessage.getActualTouchdownPositionInWorld().set(stepJustFinished.getGoalPosition());

            if (DEBUG)
            {
               LogTools.info("Just finished at t = " + timeReference.get() + " step " + stepJustFinished.getRobotQuadrant() + " : "
                     + stepJustFinished.getGoalPosition() + " Time: " + stepJustFinished.getTimeInterval());
            }

            footstepStatusPublisher.publish(statusMessage);

            feetPositions.put(stepJustFinished.getRobotQuadrant(), new FramePoint3D(ReferenceFrame.getWorldFrame(), stepJustFinished.getGoalPosition()));
         }

         Point3D centerPoint = new Point3D();
         QuadrantDependentList<Point3D> positions = new QuadrantDependentList<>();

         int number = 0;
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (feetPositions.get(robotQuadrant) != null)
            {
               centerPoint.add(feetPositions.get(robotQuadrant));
               positions.put(robotQuadrant, new Point3D(feetPositions.get(robotQuadrant)));
               number++;
            }
         }
         centerPoint.scale(1.0 / number);

         messager.submitMessage(PawStepPlannerMessagerAPI.StartPositionTopic, centerPoint);
         messager.submitMessage(PawStepPlannerMessagerAPI.StartFeetPositionTopic, positions);

         stepsCurrentlyInProgress.clear();
         stepsCurrentlyInProgress.addAll(stepsInProgress);

         firstTick = false;
         tickStartTime = currentTime;
         timeReference.addAndGet(dt);

         timedOut = timeReference.get() > maxDuration;
         if (timeReference.get() > maxDuration)
         {
            message += "\nFailed. Took too long. Maybe stalled?";
         }
      }

      if (planReference.get() != null)
      {
         Point3DReadOnly pointReached = planReference.get().getLowLevelPlanGoal().getPosition();
         Point3DReadOnly goalPosition = dataSet.getPlannerInput().getQuadrupedGoalPosition();
         if (pointReached.distanceXY(goalPosition) > PawNode.gridSizeXY)
         {
            message += "Final goal pose was not correct, meaning it did not reach the goal.\n";
            message += "Reached ( " + numberFormat.format(pointReached.getX()) + ", " + numberFormat.format(pointReached.getY()) + ", "
                  + numberFormat.format(pointReached.getZ()) + " ), but was trying to get to " + goalPosition + ".\n";
         }
      }
      else if (!planningFailed.getBooleanValue())
      {
         message += "Never got a footstep plan result, which is a problem.\n";
      }

      if (planningFailed.getBooleanValue())
      {
         return "Failed to reach the goal.\n" + message;
      }
      else
      {
         LogTools.info("Done!");

         return message;
      }
   }

   private static boolean hasQuadrantInProgress(RobotQuadrant robotQuadrant, List<QuadrupedTimedStep> stepList)
   {
      for (QuadrupedTimedStep step : stepList)
      {
         if (step.getRobotQuadrant() == robotQuadrant)
            return true;
      }

      return false;
   }

   private static String assertPlanIsValid(DataSet dataSet, List<Pose3D> pathPlan, List<QuadrupedTimedStep> plannedSteps,
                                           QuadrupedXGaitSettingsReadOnly xGaitSettings, PawStepPlannerParametersReadOnly parameters)
   {
      String datasetName = dataSet.getName();
      String errorMessage = "";

      Pose3DReadOnly actualGoal = pathPlan.get(pathPlan.size() - 1);
      Point3DReadOnly goalPosition = dataSet.getPlannerInput().getQuadrupedGoalPosition();
      double goalYaw = dataSet.getPlannerInput().getQuadrupedGoalYaw();

      if (goalPosition.distanceXY(actualGoal.getPosition()) > 3.0 * PawNode.gridSizeXY)
         errorMessage += datasetName + " did not reach goal position. Made it to " + actualGoal.getPosition() + ", trying to get to "
               + new Point3D(goalPosition);
      if (Double.isFinite(goalYaw))
      {
         if (AngleTools.computeAngleDifferenceMinusPiToPi(goalYaw, actualGoal.getOrientation().getYaw()) > PawNode.gridSizeYaw)
            errorMessage += datasetName + " did not reach goal yaw. Made it to " + actualGoal.getOrientation().getYaw() + ", trying to get to " + goalYaw;
      }

      errorMessage += checkStepTiming(datasetName, plannedSteps, xGaitSettings);
      errorMessage += checkStepOrder(datasetName, plannedSteps);
      errorMessage += checkStepPositions(datasetName, plannedSteps, xGaitSettings, parameters);

      //      if ((VISUALIZE || DEBUG) && !errorMessage.isEmpty())
      //         LogTools.error(errorMessage);

      return errorMessage;
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

   private static String checkStepTiming(String datasetName, List<QuadrupedTimedStep> plannedSteps, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      String errorMessage = "";
      RobotQuadrant previousMovingQuadrant = plannedSteps.get(0).getRobotQuadrant();
      double previousStartTime = plannedSteps.get(0).getTimeInterval().getStartTime();
      for (int i = 1; i < plannedSteps.size(); i++)
      {
         RobotQuadrant movingQuadrant = plannedSteps.get(i).getRobotQuadrant();
         double startTime = plannedSteps.get(i).getTimeInterval().getStartTime();
         double timeBetweenSteps = startTime - previousStartTime;
         double expectedTimeBetweenSteps = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(previousMovingQuadrant, xGaitSettings);
         if (!MathTools.epsilonEquals(startTime - previousStartTime, expectedTimeBetweenSteps, 1e-3))
            errorMessage += datasetName + " step " + i + " is improperly timed. Should start at " + (previousStartTime + expectedTimeBetweenSteps)
                  + " but starts at " + startTime + ". This leads to a time difference of " + timeBetweenSteps + ", when it should be "
                  + expectedTimeBetweenSteps + "\n";
         previousMovingQuadrant = movingQuadrant;
         previousStartTime = startTime;
      }

      return errorMessage;
   }

   private static String checkStepPositions(String datasetName, List<QuadrupedTimedStep> plannedSteps, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                            PawStepPlannerParametersReadOnly parameters)
   {
      String errorMessage = "";
      QuadrantDependentList<QuadrupedTimedStep> previousSteps = new QuadrantDependentList<>();
      for (int i = 0; i < plannedSteps.size(); i++)
      {
         QuadrupedTimedStep step = plannedSteps.get(i);
         RobotQuadrant stepQuadrant = step.getRobotQuadrant();
         QuadrupedTimedStep previousStep = previousSteps.get(stepQuadrant);

         if (previousStep != null)
         {
            double heightChange = Math.abs(step.getGoalPosition().getZ() - previousStep.getGoalPosition().getZ());
            if (heightChange > parameters.getMaximumStepChangeZ())
               errorMessage += datasetName + "\n Step " + i + " height changed " + heightChange + ", which was too much. Max is "
                     + parameters.getMaximumStepChangeZ();
         }

         previousSteps.put(stepQuadrant, step);
      }

      return errorMessage;
   }

   public static void main(String[] args) throws Exception
   {
      ContinuousPlanningToolboxDataSetTest test = new ContinuousPlanningToolboxDataSetTest();
      VISUALIZE = true;
      test.setup();

      String errorMessage = test.runAssertions(DataSetName._20171115_171243_SimplePlaneAndWall);
      assertTrue(errorMessage, errorMessage.isEmpty());
      LogTools.info("Done!");

      ThreadTools.sleepForever();
      test.tearDown();
   }
}
