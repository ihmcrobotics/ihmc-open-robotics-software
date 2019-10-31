package us.ihmc.avatar.networkProcessor.continuousPlanningToolboxModule;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;
import static us.ihmc.robotics.Assert.assertTrue;

import java.io.InputStream;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import com.google.common.util.concurrent.AtomicDouble;
import com.jme3.math.Transform;

import controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
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
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class BipedContinuousPlanningToolboxDataSetTest
{
   private static final double defaultNominalWidth = 0.3;
   private static final double defaultNominalLength = 0.0;

   private static final double defaultBestEffortTimeout = 0.5;
   //   private static final double defaultHorizonLength = 1.0;
   private static final double defaultHorizonLength = 2.0;

   private static final double dt = 0.01;
   private static double timeScaleFactor;

   private final DecimalFormat numberFormat = new DecimalFormat("#.00");

   private static final List<DataSetName> datasetsToIgnore = new ArrayList<>();
   static
   {
//      datasetsToIgnore.add(DataSetName._20171216_111326_CrossoverPlatforms);
   }

   // Whether to start the UI or not.
   protected static boolean VISUALIZE = true;
   // For enabling helpful prints.
   protected static boolean DEBUG = false;
   protected static boolean VERBOSE = true;

   private FootstepPlannerUI ui = null;
   protected Messager messager = null;

   private MultiStageFootstepPlanningModule footstepPlanningModule = null;
   private VisibilityGraphsParametersBasics visibilityGraphsParameters = null;
   private FootstepPlannerParametersBasics footstepPlannerParameters = null;
   private BipedContinuousPlanningToolboxModule continuousPlanningModule = null;

   private RealtimeRos2Node ros2Node;

   private final AtomicReference<FootstepPlanningToolboxOutputStatus> outputFromPlannerReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepDataListMessage> fullStepListFromContinuousToolbox = new AtomicReference<>(null);
   private final AtomicReference<Boolean> receivedFullStepList = new AtomicReference<>(false);
   private final AtomicDouble timeReference = new AtomicDouble();

   private static final String robotName = "testBot";
   public static final PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;

   private IHMCRealtimeROS2Publisher<BipedContinuousPlanningRequestPacket> requestPublisher;
   private IHMCRealtimeROS2Publisher<PlanarRegionsListMessage> planarRegionsPublisher;
   private IHMCRealtimeROS2Publisher<FootstepStatusMessage> footstepStatusPublisher;
   private IHMCRealtimeROS2Publisher<FootstepPlannerParametersPacket> plannerParametersPublisher;
   private YoBoolean planningFailed;


   // For the occlusion test
   private static final int maxPointsInRegion = 25000;
   private static final double lidarObserverHeight = 1.25;
   private static final int rays = 7500;
   private static final double rayLengthSquared = MathTools.square(5.0);

   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();


   public VisibilityGraphsParametersBasics getTestVisibilityGraphsParameters()
   {
      VisibilityGraphsParametersBasics parameters = new DefaultVisibilityGraphParameters();
//      parameters.setPerformPostProcessingNodeShifting(true);
      parameters.setComputeOrientationsToAvoidObstacles(false);
      parameters.setReturnBestEffortSolution(true);
      return parameters;
   }

   public FootstepPlannerParametersBasics getTestFootstepPlannerParameters()
   {
      FootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters();
      parameters.setReturnBestEffortPlan(true);
      parameters.setMinimumStepsForBestEffortPlan(3);

      return parameters;
   }

   @BeforeEach
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      timeScaleFactor = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? timeScaleFactor = 10.0 : 10.0;

      if (VISUALIZE)
         messager = new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API);
      else
         messager = new SharedMemoryMessager(FootstepPlannerMessagerAPI.API);


      if (visibilityGraphsParameters == null)
         visibilityGraphsParameters = getTestVisibilityGraphsParameters();

      if (footstepPlannerParameters == null)
         footstepPlannerParameters = getTestFootstepPlannerParameters();

      DRCRobotModel robotModel = getRobotModel();
      footstepPlanningModule = new MultiStageFootstepPlanningModule(robotModel, null, true, pubSubImplementation);

      YoVariableRegistry testRegistry = new YoVariableRegistry("testRegistry");
      continuousPlanningModule = new BipedContinuousPlanningToolboxModule(robotModel, null, false, pubSubImplementation);
      continuousPlanningModule.setRootRegistry(testRegistry, null);
      planningFailed = ((YoBoolean) testRegistry.getVariable("planningFailed"));


      ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_footstep_planner_test");

      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPlanningToolboxOutputStatus.class,
                                           FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotName),
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepDataListMessage.class,
                                           getTopicNameGenerator(robotName, ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX, ROS2TopicQualifier.OUTPUT),
                                           s -> processFootstepDataListMessage(s.takeNextData()));


      requestPublisher = ROS2Tools.createPublisher(ros2Node, BipedContinuousPlanningRequestPacket.class,
                                                   getTopicNameGenerator(robotName, ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX, ROS2TopicQualifier.INPUT));
      planarRegionsPublisher = ROS2Tools.createPublisher(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator);
      plannerParametersPublisher = ROS2Tools.createPublisher(ros2Node, FootstepPlannerParametersPacket.class,
                                                         getTopicNameGenerator(robotName, ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX, ROS2TopicQualifier.INPUT));

      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      footstepStatusPublisher = ROS2Tools.createPublisher(ros2Node, FootstepStatusMessage.class, controllerPubGenerator);

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

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParameters, footstepPlannerParameters);


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
      fullStepListFromContinuousToolbox.set(null);
      outputFromPlannerReference.set(null);
      receivedFullStepList.set(false);
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
                                                              for (DataSetName nameToIgnore : datasetsToIgnore)
                                                              {
                                                                 if (dataSet.getName().equals(nameToIgnore.name().substring(1)))
                                                                    return false;
                                                              }

                                                              return dataSet.getPlannerInput().getStepPlannerIsTestable() && dataSet.getPlannerInput()
                                                                                                                                    .containsFlag(getTimeoutFlag());
                                                           });
      runAssertionsOnAllDatasets(dataSets, false);
   }

   @Disabled
   @Test
   public void testDataSetsWithOcclusions()
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

                                                              return dataSet.getPlannerInput().getStepPlannerIsTestable() && dataSet.getPlannerInput()
                                                                                                                                    .containsFlag(getTimeoutFlag());
                                                           });
      runAssertionsOnAllDatasets(dataSets, true);
   }

   @Disabled
   @Test
   public void runInDevelopmentTests()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
                                                           {
                                                              if (!dataSet.hasPlannerInput())
                                                                 return false;
                                                              return dataSet.getPlannerInput().getStepPlannerIsInDevelopment() && dataSet.getPlannerInput().containsFlag(getTimeoutFlag());
                                                           });
      runAssertionsOnAllDatasets(dataSets, false);
   }

   protected String getTimeoutFlag()
   {
      return FootstepPlannerType.VIS_GRAPH_WITH_A_STAR.toString().toLowerCase() + "_timeout";
   }

   private void runAssertionsOnAllDatasets(List<DataSet> allDatasets, boolean simulateOcclusions)
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
         if (DEBUG || VERBOSE)
            LogTools.info("Testing file: " + dataset.getName());

         numberOfTestedSets++;
         resetAllAtomics();
         String errorMessagesForCurrentFile = runAssertions(dataset, simulateOcclusions);
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
            if (!errorMessagesForCurrentFile.isEmpty())
               LogTools.info(errorMessagesForCurrentFile);
         }

         ThreadTools.sleep(2000); // Apparently need to give some time for the prints to appear in the right order.
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

   protected String runAssertions(DataSetName dataSetName, boolean simulateOcclusions)
   {
      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);
      return runAssertions(dataSet, simulateOcclusions);
   }

   private String runAssertions(DataSet dataset, boolean simulateOcclusions)
   {
      resetAllAtomics();
      ThreadTools.sleep(1000);

      PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();
      if (simulateOcclusions)
      {
         HashMap<PlanarRegion, HashMap<ConvexPolygon2D, List<Point3D>>> pointsInRegions = new HashMap<>();
         for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
         {
            pointsInRegions.put(planarRegion, new HashMap<>());
            for (ConvexPolygon2D convexPolygon2D : planarRegion.getConvexPolygons())
               pointsInRegions.get(planarRegion).put(convexPolygon2D, new ArrayList<>());
         }

         Point3D observer = new Point3D();
         observer.set(dataset.getPlannerInput().getStartPosition());
         observer.addZ(lidarObserverHeight);
         for (int i = 0; i < 10; i++)
         {
            populateSimulatedVisiblePoints(planarRegionsList, pointsInRegions, observer);
         }
         planarRegionsList = createVisibleRegionsFromPoints(pointsInRegions);
      }


      broadcastPlanningRequest(dataset, planarRegionsList);
      ThreadTools.sleep(1000);

      return simulateWalkingAlongThePathAndAssertGoodResults(dataset, simulateOcclusions);
   }

   private void broadcastPlanningRequest(DataSet dataset, PlanarRegionsList planarRegionsList)
   {
      Quaternion startOrientation = new Quaternion();
      Quaternion goalOrientation = new Quaternion();
      if (dataset.getPlannerInput().hasStartOrientation())
         startOrientation.setYawPitchRoll(dataset.getPlannerInput().getStartYaw(), 0.0, 0.0);
      if (dataset.getPlannerInput().hasGoalOrientation())
         goalOrientation.setYawPitchRoll(dataset.getPlannerInput().getGoalYaw(), 0.0, 0.0);

      PlannerInput plannerInput = dataset.getPlannerInput();

      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);

      FootstepPlannerParametersPacket parametersPacket = new FootstepPlannerParametersPacket();
      FootstepPlannerMessageTools.copyParametersToPacket(parametersPacket, footstepPlannerParameters);

      planarRegionsPublisher.publish(planarRegionsListMessage);
      plannerParametersPublisher.publish(parametersPacket);

      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParameters, footstepPlannerParameters);

      ThreadTools.sleep(100);

      BipedContinuousPlanningRequestPacket requestPacket = new BipedContinuousPlanningRequestPacket();
      requestPacket.setHorizonLength(defaultHorizonLength);
      requestPacket.setTimeout(dataset.getPlannerInput().getTimeoutFlag(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR.toString().toLowerCase()));
      requestPacket.setBestEffortTimeout(defaultBestEffortTimeout);
      requestPacket.getGoalOrientationInWorld().set(goalOrientation);
      requestPacket.getGoalPositionInWorld().set(plannerInput.getGoalPosition());

      SideDependentList<FramePose3D> feetPoses = new SideDependentList<>();
      PoseReferenceFrame startFrame = new PoseReferenceFrame("startFrame", ReferenceFrame.getWorldFrame());
      startFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), dataset.getPlannerInput().getStartPosition()));
      startFrame.setOrientationAndUpdate(startOrientation);
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D footPose = new FramePose3D(startFrame);
         footPose.setY(robotSide.negateIfRightSide(0.5 * defaultNominalWidth));
         footPose.changeFrame(ReferenceFrame.getWorldFrame());

         feetPoses.put(robotSide, footPose);
      }

      requestPacket.getLeftStartPositionInWorld().set(feetPoses.get(RobotSide.LEFT).getPosition());
      requestPacket.getLeftStartOrientationInWorld().set(feetPoses.get(RobotSide.LEFT).getOrientation());
      requestPacket.getRightStartPositionInWorld().set(feetPoses.get(RobotSide.RIGHT).getPosition());
      requestPacket.getRightStartOrientationInWorld().set(feetPoses.get(RobotSide.RIGHT).getOrientation());

      requestPublisher.publish(requestPacket);

      messager.submitMessage(FootstepPlannerMessagerAPI.GoalPosition, dataset.getPlannerInput().getGoalPosition());
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalOrientation, goalOrientation);
      messager.submitMessage(FootstepPlannerMessagerAPI.StartPosition, dataset.getPlannerInput().getStartPosition());


      if (DEBUG)
         LogTools.info("Sending out planning request.");
   }

   private void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      if (DEBUG)
         PrintTools.info("Processed an output from a remote planner.");

      outputFromPlannerReference.set(packet);

      messager.submitMessage(FootstepPlannerMessagerAPI.LowLevelGoalPosition, packet.getLowLevelPlannerGoal().getPosition());
      messager.submitMessage(FootstepPlannerMessagerAPI.LowLevelGoalOrientation, packet.getLowLevelPlannerGoal().getOrientation());
      messager.submitMessage(FootstepPlannerMessagerAPI.BodyPathData, packet.getBodyPath());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, PlanarRegionMessageConverter.convertToPlanarRegionsList(packet.getPlanarRegionsList()));
   }

   private void processFootstepDataListMessage(FootstepDataListMessage packet)
   {
      if (DEBUG)
         PrintTools.info("Processed an output from a remote planner.");

      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanResponse, packet);
      fullStepListFromContinuousToolbox.set(packet);
      receivedFullStepList.set(true);
   }

   private String simulateWalkingAlongThePathAndAssertGoodResults(DataSet dataSet, boolean simulateOcclusions)
   {
      SideDependentList<FramePose3D> feetPoses = new SideDependentList<>();
      Quaternion startOrientation = new Quaternion();
      if (dataSet.getPlannerInput().hasStartOrientation())
         startOrientation.setToYawQuaternion(dataSet.getPlannerInput().getStartYaw());
      PoseReferenceFrame startFrame = new PoseReferenceFrame("startFrame", ReferenceFrame.getWorldFrame());
      startFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), dataSet.getPlannerInput().getStartPosition()));
      startFrame.setOrientationAndUpdate(startOrientation);
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D footPosition = new FramePose3D(startFrame);
         footPosition.setY(robotSide.negateIfRightSide(0.5 * defaultNominalWidth));
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());

         feetPoses.put(robotSide, footPosition);
      }

      PlanarRegionsList visibleRegions = dataSet.getPlanarRegionsList();
      PlanarRegionsList planarRegionsList = dataSet.getPlanarRegionsList();

      HashMap<PlanarRegion, HashMap<ConvexPolygon2D, List<Point3D>>> pointsInRegions = new HashMap<>();
      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
      {
         pointsInRegions.put(planarRegion, new HashMap<>());
         for (ConvexPolygon2D convexPolygon2D : planarRegion.getConvexPolygons())
            pointsInRegions.get(planarRegion).put(convexPolygon2D, new ArrayList<>());
      }

      double tickStartTime = Conversions.nanosecondsToSeconds(System.nanoTime());
      double timeAtStartOfState = -1.0;
      boolean firstTick = true;
      boolean wasInTransfer = true;
      timeReference.set(0.0);

      double expectedDuration = (dataSet.getPlannerInput().getStartPosition().distanceXY(dataSet.getPlannerInput().getGoalPosition())) / 0.2; // uses estimate speed of 0.2
      double maxDuration = 4.0 * expectedDuration;
      boolean timedOut = false;
      String message = "";
      while (!continuousPlanningModule.getToolboxController().isDone() && !planningFailed.getBooleanValue() && !timedOut)
      {
         double currentTime = Conversions.nanosecondsToSeconds(System.nanoTime());

         if (!firstTick && ((currentTime - tickStartTime) < (dt / timeScaleFactor)))
         {
            ThreadTools.sleep(100);
            continue;
         }

         if (!receivedFullStepList.get() || outputFromPlannerReference.get() == null)
            continue;

         if (timeAtStartOfState == -1.0)
            timeAtStartOfState = currentTime;

         FootstepDataListMessage stepList = fullStepListFromContinuousToolbox.get();
         FootstepDataMessage currentStep = stepList.getFootstepDataList().get(0);

         double timeInState = currentTime - timeAtStartOfState;
         boolean isInTransfer = timeInState < currentStep.getTransferDuration();
         boolean isDoneWithSwing = timeInState > currentStep.getTransferDuration() + currentStep.getSwingDuration();

         boolean thereWasAChangeInState = (!isInTransfer && wasInTransfer) || isDoneWithSwing;

         if (thereWasAChangeInState)
         {
            if (simulateOcclusions)
            {
               Point3D observer = new Point3D();
               observer.set(fullStepListFromContinuousToolbox.get().getFootstepDataList().get(0).getLocation());
               observer.addZ(lidarObserverHeight);
               populateSimulatedVisiblePoints(planarRegionsList, pointsInRegions, observer);

               PlanarRegionsList updatedRegions = createVisibleRegionsFromPoints(pointsInRegions);
               if (updatedRegions != null)
                  visibleRegions = updatedRegions;
            }

            planarRegionsPublisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(visibleRegions));
         }

         if (!isInTransfer && wasInTransfer)
         {
            FootstepStatusMessage statusMessage = new FootstepStatusMessage();
            statusMessage.setRobotSide(currentStep.getRobotSide());
            statusMessage.getDesiredFootPositionInWorld().set(currentStep.getLocation());
            statusMessage.getDesiredFootOrientationInWorld().set(currentStep.getOrientation());
            statusMessage.setFootstepStatus(FootstepStatus.STARTED.toByte());

            if (DEBUG)
            {
               LogTools.info("Just started at t = " + timeReference.get() + " step " + RobotSide.fromByte(currentStep.getRobotSide()) + " : " +
                                   currentStep.getLocation() + " Orientation: " + currentStep.getOrientation());
            }

            footstepStatusPublisher.publish(statusMessage);
            feetPoses.remove(RobotSide.fromByte(currentStep.getRobotSide()));
         }
         else if (isDoneWithSwing)
         {
            if (wasInTransfer)
               throw new RuntimeException("This shouldn't happen.");

//            stepList.getFootstepDataList().remove(currentStep);
            timeAtStartOfState = Conversions.nanosecondsToSeconds(System.nanoTime());
            stepList.getFootstepDataList().remove(currentStep);

            FootstepStatusMessage statusMessage = new FootstepStatusMessage();
            statusMessage.setRobotSide(currentStep.getRobotSide());
            statusMessage.getDesiredFootPositionInWorld().set(currentStep.getLocation());
            statusMessage.getDesiredFootOrientationInWorld().set(currentStep.getOrientation());
            statusMessage.getActualFootPositionInWorld().set(currentStep.getLocation());
            statusMessage.getActualFootOrientationInWorld().set(currentStep.getOrientation());
            statusMessage.setFootstepStatus(FootstepStatus.COMPLETED.toByte());

            if (DEBUG)
            {
               LogTools.info("Just finished at t = " + timeReference.get() + " step " + RobotSide.fromByte(currentStep.getRobotSide()) + " : " +
                                   currentStep.getLocation() + " Orientation: " + currentStep.getOrientation());
            }

            footstepStatusPublisher.publish(statusMessage);
            FramePose3D footPose = new FramePose3D();
            footPose.setPosition(currentStep.getLocation());
            footPose.setOrientation(currentStep.getOrientation());
            feetPoses.put(RobotSide.fromByte(currentStep.getRobotSide()), footPose);
         }






         String newMessage = assertPlanIsValid(dataSet, outputFromPlannerReference.get(), stepList, footstepPlannerParameters);
         if (!newMessage.isEmpty())
         {
            DecimalFormat numberFormat = new DecimalFormat("#.00");
            message += "\nAt time " + numberFormat.format(timeReference.get()) + " " + newMessage;
         }


         FramePose3D startPose = feetPoses.get(RobotSide.fromByte(fullStepListFromContinuousToolbox.get().getFootstepDataList().get(0).getRobotSide()).getOppositeSide());
         messager.submitMessage(FootstepPlannerMessagerAPI.StartPosition, new Point3D(startPose.getPosition()));
         messager.submitMessage(FootstepPlannerMessagerAPI.StartOrientation, new Quaternion(startPose.getOrientation()));

         firstTick = false;
         tickStartTime = currentTime;
         timeReference.addAndGet(dt);

         timedOut = timeReference.get() > maxDuration;
         if (timeReference.get() > maxDuration)
         {
            message += "\nFailed. Took too long. Maybe stalled?";
         }

         wasInTransfer = isInTransfer;
      }

      if (outputFromPlannerReference.get() != null)
      {
         Point3DReadOnly pointReached = outputFromPlannerReference.get().getLowLevelPlannerGoal().getPosition();
         Point3DReadOnly goalPosition = dataSet.getPlannerInput().getGoalPosition();
         if (pointReached.distanceXY(goalPosition) > LatticeNode.gridSizeXY)
         {
            message += "Final goal pose was not correct, meaning it did not reach the goal.\n";
            message += "Reached ( " + numberFormat.format(pointReached.getX()) + ", " + numberFormat.format(pointReached.getY()) + ", " +
                  numberFormat.format(pointReached.getZ()) + " ), but was trying to get to " + goalPosition + ".\n";
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

   private void populateSimulatedVisiblePoints(PlanarRegionsList regions, HashMap<PlanarRegion, HashMap<ConvexPolygon2D, List<Point3D>>> pointsInRegions,
                                               Point3D observer)
   {
      Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(observer, 1.0, rays);

      List<PlanarRegion> filteredRegions = PlanarRegionTools.filterPlanarRegionsWithBoundingCircle(new Point2D(observer), Math.sqrt(rayLengthSquared), regions.getPlanarRegionsAsList());

      for (int rayIndex = 0; rayIndex < rays; rayIndex++)
      {
         Point3D pointOnSphere = pointsOnSphere[rayIndex];
         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(pointOnSphere, observer);
         ImmutablePair<Point3D, PlanarRegion> intersectionPair = PlanarRegionTools.intersectRegionsWithRay(filteredRegions, observer, rayDirection);
         if (intersectionPair == null || intersectionPair.getLeft().distanceSquared(observer) > rayLengthSquared)
         {
            continue;
         }

         Point3D intersection = intersectionPair.getLeft();
         PlanarRegion region = intersectionPair.getRight();


         Point3D pointOnPlane = new Point3D(intersection);
         region.transformFromWorldToLocal(pointOnPlane);

         for (ConvexPolygon2D compositePolygon : region.getConvexPolygons())
         {
            if (compositePolygon.isPointInside(pointOnPlane.getX(), pointOnPlane.getY()))
            {
               pointsInRegions.get(region).get(compositePolygon).add(intersection);
               break;
            }
         }
      }

      for (PlanarRegion region : pointsInRegions.keySet())
      {
         for (ConvexPolygon2D polygon : region.getConvexPolygons())
         {
            List<Point3D> pointsInRegion = pointsInRegions.get(region).get(polygon);
            while (pointsInRegion.size() > maxPointsInRegion)
               pointsInRegion.remove(0);
         }
      }
   }

   private PlanarRegionsList createVisibleRegionsFromPoints(HashMap<PlanarRegion, HashMap<ConvexPolygon2D, List<Point3D>>> pointsInRegions)
   {
      List<PlanarRegionSegmentationRawData> segmentationRawData = new ArrayList<>();
      for (PlanarRegion originalRegion : pointsInRegions.keySet())
      {
         Point3D center = new Point3D();
         originalRegion.getBoundingBox3dInWorld().getCenterPoint(center);
         int counter = 0;

         for (ConvexPolygon2D polygon : pointsInRegions.get(originalRegion).keySet())
         {
            List<Point3D> points = pointsInRegions.get(originalRegion).get(polygon);
            int id = 1000 * originalRegion.getRegionId() + counter; // shifts the region id by 1000
            PlanarRegionSegmentationRawData rawData = new PlanarRegionSegmentationRawData(id, originalRegion.getNormal(), center, points);
            segmentationRawData.add(rawData);
            counter++;
         }
      }

      PlanarRegionsList visibleRegionsList;
      try
      {
         visibleRegionsList = PlanarRegionPolygonizer
               .createPlanarRegionsList(segmentationRawData, concaveHullFactoryParameters, polygonizerParameters);
      }
      catch (Exception e)
      {
         visibleRegionsList = null;
      }

      return visibleRegionsList;
   }

   private static String assertPlanIsValid(DataSet dataSet, FootstepPlanningToolboxOutputStatus planResult, FootstepDataListMessage stepMessage,
                                           FootstepPlannerParametersReadOnly parameters)
   {
      String datasetName = dataSet.getName();
      String errorMessage = "";

      List<Pose3D> pathPlan = planResult.getBodyPath();
      Pose3DReadOnly actualGoal = pathPlan.get(pathPlan.size() - 1);
      Point3DReadOnly goalPosition = dataSet.getPlannerInput().getGoalPosition();
      double goalYaw = dataSet.getPlannerInput().getGoalYaw();

      if (goalPosition.distanceXY(actualGoal.getPosition()) > 3.0 * LatticeNode.gridSizeXY)
         errorMessage += datasetName + " did not reach goal position. Made it to " + actualGoal.getPosition() + ", trying to get to " + new Point3D(goalPosition);
      if (Double.isFinite(goalYaw))
      {
         if (AngleTools.computeAngleDifferenceMinusPiToPi(goalYaw, actualGoal.getOrientation().getYaw()) > LatticeNode.gridSizeYaw)
            errorMessage += datasetName + " did not reach goal yaw. Made it to " + actualGoal.getOrientation().getYaw() + ", trying to get to " + goalYaw;
      }

      List<FootstepDataMessage> plannedSteps = stepMessage.getFootstepDataList();
      errorMessage += checkStepOrder(datasetName, plannedSteps);
      errorMessage += checkStepPositions(datasetName, plannedSteps, parameters);

//      if ((VISUALIZE || DEBUG) && !errorMessage.isEmpty())
//         LogTools.error(errorMessage);

      return errorMessage;
   }

   private static String checkStepOrder(String datasetName, List<FootstepDataMessage> plannedSteps)
   {
      if (plannedSteps.size() < 1)
         return "";

      String errorMessage = "";
      RobotSide previousMovingSide = RobotSide.fromByte(plannedSteps.get(0).getRobotSide());
      for (int i = 1; i < plannedSteps.size(); i++)
      {
         RobotSide movingSide = RobotSide.fromByte(plannedSteps.get(i).getRobotSide());
         if (previousMovingSide.getOppositeSide() != movingSide)
            errorMessage += datasetName + " step " + i + " in the plan is out of order.\n";
         previousMovingSide = movingSide;
      }

      return errorMessage;
   }


   private static String checkStepPositions(String datasetName, List<FootstepDataMessage> plannedSteps, FootstepPlannerParametersReadOnly parameters)
   {
      double heightChangeFromWiggling = Math.sqrt(2) * parameters.getMaximumXYWiggleDistance() / Math.sin(parameters.getMinimumSurfaceInclineRadians());
      String errorMessage = "";
      SideDependentList<FootstepDataMessage> previousSteps = new SideDependentList<>();
      for (int i = 0; i < plannedSteps.size(); i++)
      {
         FootstepDataMessage step = plannedSteps.get(i);
         RobotSide stepSide = RobotSide.fromByte(step.getRobotSide());
         FootstepDataMessage previousStep = previousSteps.get(stepSide);

         if (previousStep != null)
         {
            double heightChange = Math.abs(step.getLocation().getZ() - previousStep.getLocation().getZ());
            if (heightChange > parameters.getMaximumStepZ() + 2.0 * heightChangeFromWiggling)
               errorMessage += datasetName + "\n Step " + i + " height changed " + heightChange + ", which was too much. Max is " + parameters.getMaximumStepZ();
         }

         previousSteps.put(stepSide, step);
      }

      return errorMessage;
   }

   private DRCRobotModel getRobotModel()
   {
      return new TestRobotModel();
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
      public DRCSensorSuiteManager getSensorSuiteManager()
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
      public HumanoidRobotSensorInformation getSensorInformation()
      {
         return null;
      }

      @Override
      public InputStream getWholeBodyControllerParametersFile()
      {
         return null;
      }

      @Override
      public FootstepPlannerParametersBasics getFootstepPlannerParameters()
      {
         return getTestFootstepPlannerParameters();
      }

      @Override
      public VisibilityGraphsParametersBasics getVisibilityGraphsParameters()
      {
         return getTestVisibilityGraphsParameters();
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

   public static void main(String[] args) throws Exception
   {
      BipedContinuousPlanningToolboxDataSetTest test = new BipedContinuousPlanningToolboxDataSetTest();
      VISUALIZE = true;
      test.setup();

      String errorMessage = test.runAssertions(DataSetName._20171218_204953_FlatGroundWithWall, true);
      assertTrue(errorMessage, errorMessage.isEmpty());
      LogTools.info("Done!");

      ThreadTools.sleepForever();
      test.tearDown();
   }
}
