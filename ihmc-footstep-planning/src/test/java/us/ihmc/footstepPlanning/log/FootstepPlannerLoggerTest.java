package us.ihmc.footstepPlanning.log;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import toolbox_msgs.msg.dds.FootstepPlannerParametersPacket;
import toolbox_msgs.msg.dds.FootstepPlanningRequestPacket;
import toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import org.junit.jupiter.api.Test;
import toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader.LoadResult;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;

import static org.junit.jupiter.api.Assertions.*;

public class FootstepPlannerLoggerTest
{
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
      request.setHeightMapData(HeightMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(dataSet.getPlanarRegionsList())));
      request.setAssumeFlatGround(false);
      request.setPlanBodyPath(true);
   }

   /**
    * This test was created to investigate a problem where logging didn't seem to be working, however after more investigation it was because it was trying
    * to be run on the Continuous Integration (CI) server. Its possible to set a environmental variable to run these classes "as if they were on the server" and
    * that was happening here.
    *
    * This test isn't much and is nice to have lying around to prevent future change from breaking the logger
    */
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
    * This test ensures that logging the footstep plan's doesn't take too long. The speed in which this takes doesn't need ot be exact but
    * this allows us to ensure that logging of footstep plan isn't slow. Otherwise, this will slow down threads if planning over and over again
    */
   @Disabled
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
      long timeExpected = 150;

      // Logging the footstep plans should not take to long, the timeExpected is arbitrary, but it ensures that the logging doesn't take too long
      assertTrue(timeInMilliseconds < timeExpected, "Time taken was: " + timeInMilliseconds + ", and the time expected was " + timeExpected);
   }

   @Test
   public void testLogger()
   {
      FootstepPlannerOutput plannerOutput = planningModule.handleRequest(request);

      FootstepPlannerLogger logger = new FootstepPlannerLogger(planningModule);
      boolean success = logger.logSession(logDirectory);
      assertTrue(success, "Error generating footstep planner log");

      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      LoadResult loadResult = logLoader.load(new File(logger.getLatestLogDirectory()));
      assertSame(loadResult, LoadResult.LOADED, "Error loading footstep planner log");

      FootstepPlannerLog log = logLoader.getLog();

      FootstepPlanningRequestPacket expectedRequestPacket = new FootstepPlanningRequestPacket();
      FootstepPlannerParametersPacket expectedFootstepParameters = new FootstepPlannerParametersPacket();
      VisibilityGraphsParametersPacket expectedBodyPathParameters = new VisibilityGraphsParametersPacket();
      FootstepPlanningToolboxOutputStatus expectedOutputStatusPacket = new FootstepPlanningToolboxOutputStatus();

      request.setPacket(expectedRequestPacket);
      FootstepPlannerMessageTools.copyParametersToPacket(expectedFootstepParameters, planningModule.getFootstepPlannerParameters());
      plannerOutput.setPacket(expectedOutputStatusPacket);

      //TODO this test is broken because the outputStatus never sets the goal pose, but thats part of the message so it needs to be set in order for this test to pass
      // or the goal pose needs to be changed, maybe removed because there should be a goal pose for each foot
      assertTrue(expectedRequestPacket.epsilonEquals(log.getRequestPacket(), 1e-5));
      assertTrue(expectedFootstepParameters.epsilonEquals(log.getFootstepParametersPacket(), 1e-5));
      assertTrue(expectedOutputStatusPacket.epsilonEquals(log.getStatusPacket(), 1e-5));

      FootstepPlannerLogger.deleteOldLogs(0, logDirectory);
   }
}
