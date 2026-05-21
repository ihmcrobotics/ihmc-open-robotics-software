package us.ihmc.footstepPlanning.log;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import toolbox_msgs.AStarBodyPathPlannerParametersPacket;
import toolbox_msgs.FootstepPlannerParametersPacket;
import toolbox_msgs.FootstepPlanningRequestPacket;
import toolbox_msgs.FootstepPlanningToolboxOutputStatus;
import toolbox_msgs.SwingPlannerParametersPacket;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader.LoadResult;
import us.ihmc.communication.serialization.Ros2MessageCdrFileTools;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;

import static org.junit.jupiter.api.Assertions.*;

public class FootstepPlannerLoggerTest
{
   /**
    * We use an epsilon here because when data gets logs, it may get slightly rounded depending on the situation.
    * So we want to check that things haven't changed
    * a ton
    */
   private static final double EPSILON = 1e-5;
   private static final String logDirectory = System.getProperty("user.home") + File.separator + "testLog" + File.separator;
   private final FootstepPlannerRequest request = new FootstepPlannerRequest();
   private final FootstepPlanningModule planningModule = new FootstepPlanningModule("TestModule");

   @BeforeEach
   public void setupFootstepPlannerRequest()
   {
      // Set up the request for some tests
      DataSet dataSet = DataSetIOTools.loadDataSet(DataSetName._20190220_172417_EOD_Cinders);

      Pose3D initialMidFootPose = new Pose3D(dataSet.getPlannerInput().getStartPosition(), new Quaternion(dataSet.getPlannerInput().getStartYaw(), 0.0, 0.0));
      Pose3D goalMidFootPose = new Pose3D(dataSet.getPlannerInput().getGoalPosition(), new Quaternion(dataSet.getPlannerInput().getGoalYaw(), 0.0, 0.0));
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setAssumeFlatGround(false);
      request.setPlanBodyPath(true);
   }

   /**
    * This test was created to investigate a problem where logging didn't seem to be working, however, after more investigation it was because it was trying
    * to be run on the Continuous Integration (CI) server.
    * It's possible to set an environmental variable to run these classes "as if they were on the server" and
    * that was happening here.
    * This test isn't much computation and is nice to have lying around to prevent future change from breaking the logger
    * This test likely won't run on CI because it's checking a hard coded path
    */
   @Disabled
   @Test
   public void testFootstepLoggingDefaultDirectory()
   {
      planningModule.handleRequest(request);

      FootstepPlannerLogger logger = new FootstepPlannerLogger(planningModule);

      // Here we compare that the default log directory matches where we expect the logs to go
      String defaultLogsDirectory = FootstepPlannerLogger.defaultLogsDirectory;
      String ihmcFolder = IHMCCommonPaths.ASTAR_FOOTSTEP_PLANNER_DIRECTORY.toString();
      assertEquals(ihmcFolder, defaultLogsDirectory);

      // Log the session and ensure that the latest log directory is not empty, so some data exists inside it
      logger.logSession();
      assertFalse(logger.getLatestLogDirectory().isEmpty());
   }

   /**
    * This test ensures that logging the footstep plan's doesn't take too long.
    * The speed in which this runs at doesn't need ot exact, but
    * this allows us to ensure that logging of footstep plan isn't slow.
    * Otherwise, this will slow down threads if planning over and over again
    */
   @Test
   public void testFootstepLoggingSpeed()
   {
      planningModule.handleRequest(request);

      // Everything above this creates a plan that we will be able to log
      FootstepPlannerLogger logger = new FootstepPlannerLogger(planningModule);
      Stopwatch timer = new Stopwatch();

      // Time how long the logging takes starting at the highest method
      timer.start();
      logger.logSession(logDirectory);
      double timeTaken = timer.totalElapsed();

      long timeInMilliseconds = (long) (timeTaken * 1000);
      // This number is arbitrary, just want to make sure things aren't slow when logging, important if re-planning in a loop
      long timeExpected = 180;

      // Logging the footstep plans should not take to long, the timeExpected is arbitrary, but it ensures that the logging doesn't take too long
      assertTrue(timeInMilliseconds < timeExpected, "Time taken was: " + timeInMilliseconds + ", and the time expected was " + timeExpected);
   }

   /**
    * This test is making sure that all the {@link us.ihmc.tools.property.StoredPropertySet} parameters are getting logged correctly.
    * We want to make sure the parameters we are using in the planning module are getting logged.
    */
   @Test
   public void testLoggedPacketsMatchThePacketsSentToThePlanner()
   {
      // We store the plannerOutput for later use and log the session in a new directory that isn't the default
      FootstepPlannerOutput plannerOutput = planningModule.handleRequest(request);

      // Make sure that logging the session was successful, otherwise there isn't a point to go any farther
      FootstepPlannerLogger logger = new FootstepPlannerLogger(planningModule);
      boolean success = logger.logSession(logDirectory);
      assertTrue(success, "Error generating footstep planner log");

      // Then we want to load the log, again we check that loading the log was a success or there isn't a point to go any farther
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      LoadResult loadResult = logLoader.load(new File(logger.getLatestLogDirectory()));
      assertSame(loadResult, LoadResult.LOADED, "Error loading footstep planner log");

      // Get the log and start comparing the expected packets sent to in the request to the planner
      FootstepPlannerLog log = logLoader.getLog();

      // Expected request packet, this variable gets packed with the data from the request
      FootstepPlanningRequestPacket expectedRequestPacket = new FootstepPlanningRequestPacket();
      request.setPacket(expectedRequestPacket);
      assertArrayEquals(Ros2MessageCdrFileTools.serializeToBytes(expectedRequestPacket),
                        Ros2MessageCdrFileTools.serializeToBytes(log.getRequestPacket()));

      // Expected footstep planner parameters
      FootstepPlannerParametersPacket expectedFootstepPlannerParameters = new FootstepPlannerParametersPacket();
      FootstepPlannerMessageTools.copyParametersToPacket(expectedFootstepPlannerParameters, planningModule.getFootstepPlannerParameters());
      assertArrayEquals(Ros2MessageCdrFileTools.serializeToBytes(expectedFootstepPlannerParameters),
                        Ros2MessageCdrFileTools.serializeToBytes(log.getFootstepParametersPacket()));

      // Expected output status packet, this variable gets packed with the data from the planner output
      FootstepPlanningToolboxOutputStatus expectedOutputStatusPacket = new FootstepPlanningToolboxOutputStatus();
      plannerOutput.setPacket(expectedOutputStatusPacket);
      assertArrayEquals(Ros2MessageCdrFileTools.serializeToBytes(expectedOutputStatusPacket),
                        Ros2MessageCdrFileTools.serializeToBytes(log.getStatusPacket()));

      // Expected body path planner packet
      AStarBodyPathPlannerParametersPacket expectedBodyPathParameters = new AStarBodyPathPlannerParametersPacket();
      expectedBodyPathParameters.set(planningModule.getAStarBodyPathPlannerParameters().getAsPacket());
      assertArrayEquals(Ros2MessageCdrFileTools.serializeToBytes(expectedBodyPathParameters),
                        Ros2MessageCdrFileTools.serializeToBytes(log.getAStarBodyPathPlannerParametersPacket()));

      // Expected swing planner parameters
      SwingPlannerParametersPacket expectedSwingPlannerParameters = new SwingPlannerParametersPacket();
      expectedSwingPlannerParameters.set(planningModule.getSwingPlannerParameters().getAsPacket());
      assertArrayEquals(Ros2MessageCdrFileTools.serializeToBytes(expectedSwingPlannerParameters),
                        Ros2MessageCdrFileTools.serializeToBytes(log.getSwingPlannerParametersPacket()));

      // Delete the log to avoid creating a bunch of logs when running these tests
      FootstepPlannerLogger.deleteOldLogs(0, logDirectory);
   }

   /**
    * When deleting logs, we want to make sure that all the files get deleted, we don't want to be leaving files around or parts of old logs.
    * This test ensures that when a log gets deleted that the directory is empty
    */
   @Test
   public void testDeletingOldLogs()
   {
      // Handle a request so we have something to log
      planningModule.handleRequest(request);

      // Path to log this request
      String pathToLogRequest = System.getProperty("user.home") + File.separator + "deleteMe" + File.separator;

      // Make sure that logging the session was successful, otherwise there isn't a point to go any farther
      FootstepPlannerLogger logger = new FootstepPlannerLogger(planningModule);
      boolean success = logger.logSession(pathToLogRequest);
      assertTrue(success, "Error generating footstep planner log");

      // This should delete this latest log which means the folder should be empty, the folder should still exist however
      FootstepPlannerLogger.deleteOldLogs(0, pathToLogRequest);

      // Check that folder doesn't exist; we expect to delete all the logs and if the directory is empty it gets deleted so it shouldn't exist anymore
      File directory = new File(pathToLogRequest);
      assertNull(directory.list());
   }
}
