package us.ihmc.avatar.networkProcessor.footstepPlanningModule;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerType;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.postProcessing.parameters.DefaultFootstepPostProcessingParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.ApplicationRunner;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.footstepPlanning.ui.RemoteUIMessageConverter;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
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
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public abstract class FootstepPlannerDataSetTest
{
   private static final double bambooTimeScaling = 4.0;

   // Whether to start the UI or not.
   public static boolean VISUALIZE = true;
   // For enabling helpful prints.
   private static boolean DEBUG = true;
   private static boolean VERBOSE = true;

   private FootstepPlannerUI ui = null;
   protected Messager messager = null;

   private final AtomicReference<FootstepPlanningToolboxOutputStatus> plannerOutputStatus = new AtomicReference<>();

   private static final String robotName = "testBot";
   private FootstepPlanningModule planningModule;

   private RealtimeRos2Node ros2Node;

   private RemoteUIMessageConverter messageConverter = null;

   public PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;

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
      tryToStartModule(this::setupFootstepPlanningModule);
      ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_footstep_planner_test");

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

      ThreadTools.sleep(1000);
   }

   @Test
   public void testDataSets()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataset ->
                                                           {
                                                              if (!dataset.hasPlannerInput())
                                                                 return false;
                                                              return dataset.getPlannerInput().getStepPlannerIsTestable() && dataset.getPlannerInput()
                                                                                                                                    .containsFlag(getTimeoutFlag());
                                                           });
      runAssertionsOnAllDatasets(this::runAssertions, dataSets);
   }

   @Test
   @Disabled
   public void runInDevelopmentDataSets()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataset ->
                                                           {
                                                              if (!dataset.hasPlannerInput())
                                                                 return false;
                                                              return dataset.getPlannerInput().getStepPlannerIsInDevelopment() && dataset.getPlannerInput()
                                                                                                                                         .containsFlag(getTimeoutFlag());
                                                           });
      runAssertionsOnAllDatasets(this::runAssertions, dataSets);
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      ros2Node.destroy();

      messager.closeMessager();
      messageConverter.destroy();
      planningModule.closeAndDispose();

      if (ui != null)
         ui.stop();
      ui = null;

      pubSubImplementation = null;
      ros2Node = null;
      planningModule = null;

      messageConverter = null;
      messager = null;
   }

   private void resetAllAtomics()
   {
      plannerOutputStatus.set(null);
   }

   private void tryToStartModule(ModuleStarter runnable)
   {
      try
      {
         runnable.startModule();
      }
      catch (RuntimeException | IOException e)
      {
         LogTools.error("Failed to start a module in the network processor, stack trace:");
         e.printStackTrace();
      }
   }

   private interface ModuleStarter
   {
      void startModule() throws IOException;
   }

   private void setupFootstepPlanningModule() throws IOException
   {
      planningModule = FootstepPlanningModuleLauncher.createModule(getRobotModel());

   }

   private DRCRobotModel getRobotModel()
   {
      return new TestRobotModel();
   }

   public void runAssertionsOnDataset(Function<DataSet, String> dataSetTester, DataSetName dataSetName)
   {
      DataSet dataset = DataSetIOTools.loadDataSet(dataSetName);

      resetAllAtomics();
      String errorMessages = dataSetTester.apply(dataset);
      Assertions.assertTrue(errorMessages.isEmpty(), "Errors:" + errorMessages);
   }


   public void runAssertionsOnAllDatasets(Function<DataSet, String> dataSetTester, List<DataSet> allDatasets)
   {
      if (VERBOSE || DEBUG)
         LogTools.info("Unit test files found: " + allDatasets.size());

      if (allDatasets.isEmpty())
         Assertions.fail("Did not find any datasets to test.");

      int numberOfFailingTests = 0;
      int numberOfTestedSets = 0;
      List<String> failingDatasets = new ArrayList<>();
      for (int i = 0; i < allDatasets.size(); i++)
      {
         DataSet dataset = allDatasets.get(i);
         if (DEBUG || VERBOSE)
            LogTools.info("Testing file: " + dataset.getName());

         numberOfTestedSets++;
         resetAllAtomics();
         String errorMessagesForCurrentFile = dataSetTester.apply(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
         {
            numberOfFailingTests++;
            failingDatasets.add(dataset.getName());
         }

         if (DEBUG || VERBOSE)
         {
            String result = errorMessagesForCurrentFile.isEmpty() ? "passed" : "failed";
            LogTools.info(dataset.getName() + " " + result);
         }

         ThreadTools.sleep(500); // Apparently need to give some time for the prints to appear in the right order.
      }

      String message = "Number of failing datasets: " + numberOfFailingTests + " out of " + numberOfTestedSets;
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
         Assertions.assertEquals(numberOfFailingTests, 0, message);
      }
   }

   public String runAssertions(DataSet dataset)
   {
      resetAllAtomics();
      ThreadTools.sleep(1000);

      packPlanningRequest(dataset, messager);

      resetAllAtomics();
      ThreadTools.sleep(1000);

      return findPlanAndAssertGoodResult(dataset);
   }

   private void packPlanningRequest(DataSet dataset, Messager messager)
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.StartPosition, dataset.getPlannerInput().getStartPosition());
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalPosition, dataset.getPlannerInput().getGoalPosition());
      messager.submitMessage(PlannerType, getPlannerType());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, dataset.getPlanarRegionsList());

      double timeMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double timeout = Double.parseDouble(dataset.getPlannerInput().getAdditionalData(getTimeoutFlag()).get(0));
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeout, timeMultiplier * timeout);

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerHorizonLength, Double.MAX_VALUE);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParameters, getRobotModel().getFootstepPlannerParameters());
      messager.submitMessage(FootstepPlannerMessagerAPI.PostProcessingParametersTopic, new DefaultFootstepPostProcessingParameters());

      double startYaw = dataset.getPlannerInput().hasStartOrientation() ? dataset.getPlannerInput().getStartYaw() : 0.0;
      double goalYaw = dataset.getPlannerInput().hasGoalOrientation() ? dataset.getPlannerInput().getGoalYaw() : 0.0;
      messager.submitMessage(FootstepPlannerMessagerAPI.StartOrientation, new Quaternion(startYaw, 0.0, 0.0));
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalOrientation, new Quaternion(goalYaw, 0.0, 0.0));

      messager.submitMessage(FootstepPlannerMessagerAPI.ComputePath, true);

      if (DEBUG)
         LogTools.info("Sending out planning request packet.");
   }

   private void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      if (DEBUG)
         LogTools.info("Planner status: " + FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult()));

      plannerOutputStatus.set(packet);
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

   protected String getTimeoutFlag()
   {
      return getPlannerType().toString().toLowerCase() + "_timeout";
   }

   private String assertPlansAreValid(String datasetName, FootstepPlanningResult expectedResult, FootstepPlanningResult actualResult,
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



   private String assertPlanIsValid(String datasetName, FootstepPlanningResult result, FootstepPlan plan, Point3D goal)
   {
      String errorMessage = "";

      if(!result.validForExecution())
      {
         errorMessage = "Planning result for " + datasetName + " is invalid, result was " + result;
      }
      else if(!PlannerTools.isGoalNextToLastStep(goal, plan))
      {
         errorMessage = datasetName + " did not reach goal. Made it to " + PlannerTools.getEndPosition(plan) + ", trying to get to " + goal;
      }

      if((VISUALIZE || DEBUG) && !errorMessage.isEmpty())
         LogTools.error(errorMessage);

      return errorMessage;
   }


   private String assertTrue(String datasetName, String message, boolean condition)
   {
      if (VISUALIZE || DEBUG)
      {
         if (!condition)
            LogTools.error(datasetName + ": " + message);
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
      }

      return errorMessage;
   }

   public String findPlanAndAssertGoodResult(DataSet dataset)
   {
      totalTimeTaken = 0;
      double timeoutMultiplier = ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() ? bambooTimeScaling : 1.0;
      double maxTimeToWait = 2.0 * timeoutMultiplier * 60.0;
      String datasetName = dataset.getName();

      String errorMessage = "";

      if (DEBUG)
         LogTools.info("Waiting for result.");

      ConditionChecker continueWaitingCriteria = () ->
      {
         FootstepPlanningToolboxOutputStatus outputStatus = plannerOutputStatus.get();
         if (outputStatus == null)
            return true;
         FootstepPlanningResult result = FootstepPlanningResult.fromByte(outputStatus.getFootstepPlanningResult());
         return result == FootstepPlanningResult.SOLUTION_DOES_NOT_REACH_GOAL;
      };

      errorMessage += waitForResult(continueWaitingCriteria, maxTimeToWait, datasetName);
      if (!errorMessage.isEmpty())
         return errorMessage;

      FootstepPlanningToolboxOutputStatus outputStatus = plannerOutputStatus.get();
      FootstepPlanningResult result = FootstepPlanningResult.fromByte(outputStatus.getFootstepPlanningResult());

      if (DEBUG)
         LogTools.info("Planner terminated with status: " + result);
      if (!result.validForExecution())
         return "Planning result for " + datasetName + " is invalid, result was " + result;

      FootstepPlan footstepPlan = convertToFootstepPlan(outputStatus.getFootstepDataList());
      Point3D goalPosition = dataset.getPlannerInput().getGoalPosition();
      if (!PlannerTools.isGoalNextToLastStep(goalPosition, footstepPlan))
         return datasetName + " did not reach goal. Made it to " + PlannerTools.getEndPosition(footstepPlan) + ", trying to get to " + goalPosition;

      return errorMessage;
   }

   private static interface ConditionChecker
   {
      boolean checkCondition();
   }

   public class TestRobotModel implements DRCRobotModel
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
         return new SmoothCMPPlannerParameters();
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new TestWalkingControllerParameters();
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
         DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
         parameters.setCheckForPathCollisions(true);
         return parameters;
      }

      @Override
      public VisibilityGraphsParametersBasics getVisibilityGraphsParameters()
      {
         DefaultVisibilityGraphParameters visibilityGraphParameters = new DefaultVisibilityGraphParameters();
         return visibilityGraphParameters;
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

   public class TestWalkingControllerParameters extends WalkingControllerParameters
   {

      @Override
      public double getOmega0()
      {
         return 0;
      }

      @Override
      public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
      {
         return false;
      }

      @Override
      public double getMinimumSwingTimeForDisturbanceRecovery()
      {
         return 0;
      }

      @Override
      public double getICPErrorThresholdToSpeedUpSwing()
      {
         return 0;
      }

      @Override
      public boolean allowAutomaticManipulationAbort()
      {
         return false;
      }

      @Override
      public ICPControlGains createICPControlGains()
      {
         return null;
      }

      @Override
      public PDGains getCoMHeightControlGains()
      {
         return null;
      }

      @Override
      public PIDSE3Configuration getSwingFootControlGains()
      {
         return null;
      }

      @Override
      public PIDSE3Configuration getHoldPositionFootControlGains()
      {
         return null;
      }

      @Override
      public PIDSE3Configuration getToeOffFootControlGains()
      {
         return null;
      }

      @Override
      public double getDefaultTransferTime()
      {
         return 0;
      }

      @Override
      public double getDefaultSwingTime()
      {
         return 0;
      }

      @Override
      public FootSwitchFactory getFootSwitchFactory()
      {
         return null;
      }

      @Override
      public String[] getJointsToIgnoreInController()
      {
         return new String[0];
      }

      @Override
      public MomentumOptimizationSettings getMomentumOptimizationSettings()
      {
         return null;
      }

      @Override
      public ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters()
      {
         return null;
      }

      @Override
      public double getMaxICPErrorBeforeSingleSupportForwardX()
      {
         return 0;
      }

      @Override
      public double getMaxICPErrorBeforeSingleSupportInnerY()
      {
         return 0;
      }

      @Override
      public ToeOffParameters getToeOffParameters()
      {
         return null;
      }

      @Override
      public SwingTrajectoryParameters getSwingTrajectoryParameters()
      {
         return new TestSwingTrajectoryParameters();
      }

      @Override
      public ICPOptimizationParameters getICPOptimizationParameters()
      {
         return null;
      }

      @Override
      public double getMaximumLegLengthForSingularityAvoidance()
      {
         return 0;
      }

      @Override
      public double minimumHeightAboveAnkle()
      {
         return 0;
      }

      @Override
      public double nominalHeightAboveAnkle()
      {
         return 0;
      }

      @Override
      public double maximumHeightAboveAnkle()
      {
         return 0;
      }

      @Override
      public double defaultOffsetHeightAboveAnkle()
      {
         return 0;
      }

      @Override
      public SteppingParameters getSteppingParameters()
      {
         return new TestSteppingParameters();
      }
   }

   public class TestSteppingParameters implements SteppingParameters
   {
      @Override
      public double getMaxStepLength()
      {
         return 0.5;
      }

      @Override
      public double getDefaultStepLength()
      {
         return 0;
      }

      @Override
      public double getMaxStepWidth()
      {
         return 0;
      }

      @Override
      public double getMinStepWidth()
      {
         return 0;
      }

      @Override
      public double getInPlaceWidth()
      {
         return 0;
      }

      @Override
      public double getDesiredStepForward()
      {
         return 0;
      }

      @Override
      public double getStepPitch()
      {
         return 0;
      }

      @Override
      public double getMaxStepUp()
      {
         return 0;
      }

      @Override
      public double getMaxStepDown()
      {
         return 0;
      }

      @Override
      public double getMaxSwingHeightFromStanceFoot()
      {
         return 0.5;
      }

      @Override
      public double getMaxAngleTurnOutwards()
      {
         return 0;
      }

      @Override
      public double getMaxAngleTurnInwards()
      {
         return 0;
      }

      @Override
      public double getMinAreaPercentForValidFootstep()
      {
         return 0;
      }

      @Override
      public double getDangerAreaPercentForValidFootstep()
      {
         return 0;
      }

      @Override
      public double getFootForwardOffset()
      {
         return 0;
      }

      @Override
      public double getFootBackwardOffset()
      {
         return 0;
      }

      @Override
      public double getFootWidth()
      {
         return 0;
      }

      @Override
      public double getToeWidth()
      {
         return 0;
      }

      @Override
      public double getFootLength()
      {
         return 0;
      }

      @Override
      public double getActualFootWidth()
      {
         return 0;
      }

      @Override
      public double getActualFootLength()
      {
         return 0;
      }
   }

   public class TestSwingTrajectoryParameters extends SwingTrajectoryParameters
   {

      @Override
      public boolean doToeTouchdownIfPossible()
      {
         return false;
      }

      @Override
      public double getToeTouchdownAngle()
      {
         return 0;
      }

      @Override
      public boolean doHeelTouchdownIfPossible()
      {
         return false;
      }

      @Override
      public double getHeelTouchdownAngle()
      {
         return 0;
      }

      @Override
      public double getDesiredTouchdownHeightOffset()
      {
         return 0;
      }

      @Override
      public double getDesiredTouchdownVelocity()
      {
         return 0.3;
      }

      @Override
      public double getDesiredTouchdownAcceleration()
      {
         return 2.0;
      }

      @Override
      public double getMinMechanicalLegLength()
      {
         return 0;
      }
   }
}
