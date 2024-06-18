package us.ihmc.footstepPlanning.log;

import org.junit.jupiter.api.Disabled;
import toolbox_msgs.msg.dds.FootstepPlannerParametersPacket;
import toolbox_msgs.msg.dds.FootstepPlanningRequestPacket;
import toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

import java.io.File;

import static org.junit.jupiter.api.Assertions.*;

public class FootstepPlannerLoggerTest
{
   private static final String logDirectory = System.getProperty("user.home") + File.separator + "testLog" + File.separator;

   @AfterAll
   public static void cleanTestDirectory()
   {
      FootstepPlannerLogger.deleteOldLogs(0, logDirectory);
   }

   /**
    * This test ensures that logging the footstep plan's doesn't take too long. The speed in which this takes doesn't need ot be exact but
    * this allows us to ensure that logging of footstep plan's isn't slow. The will slow down threads if planning over and over again
    */
   @Disabled
   @Test
   public void testFootstepLoggingSpeed()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule("testModule");
      DataSet dataSet = DataSetIOTools.loadDataSet(DataSetName._20190220_172417_EOD_Cinders);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      Pose3D initialMidFootPose = new Pose3D(dataSet.getPlannerInput().getStartPosition(), new Quaternion(dataSet.getPlannerInput().getStartYaw(), 0.0, 0.0));
      Pose3D goalMidFootPose = new Pose3D(dataSet.getPlannerInput().getGoalPosition(), new Quaternion(dataSet.getPlannerInput().getGoalYaw(), 0.0, 0.0));
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setHeightMapData(HeightMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(dataSet.getPlanarRegionsList())));
      request.setAssumeFlatGround(false);
      request.setPlanBodyPath(true);

      planningModule.getFootstepPlannerParameters().setMaxStepZ(0.294);
      planningModule.getFootstepPlannerParameters().setYawWeight(0.17);
      planningModule.getFootstepPlannerParameters().setMaxZPenetrationOnValleyRegions(1.0);

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
      FootstepPlanningModule planningModule = new FootstepPlanningModule("testModule");
      DataSet dataSet = DataSetIOTools.loadDataSet(DataSetName._20190220_172417_EOD_Cinders);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      Pose3D initialMidFootPose = new Pose3D(dataSet.getPlannerInput().getStartPosition(), new Quaternion(dataSet.getPlannerInput().getStartYaw(), 0.0, 0.0));
      Pose3D goalMidFootPose = new Pose3D(dataSet.getPlannerInput().getGoalPosition(), new Quaternion(dataSet.getPlannerInput().getGoalYaw(), 0.0, 0.0));
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setHeightMapData(HeightMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(dataSet.getPlanarRegionsList())));
      request.setAssumeFlatGround(false);
      request.setPlanBodyPath(true);

      planningModule.getFootstepPlannerParameters().setMaxStepZ(0.294);
      planningModule.getFootstepPlannerParameters().setYawWeight(0.17);
      planningModule.getFootstepPlannerParameters().setMaxZPenetrationOnValleyRegions(1.0);

      FootstepPlannerOutput plannerOutput = planningModule.handleRequest(request);

      FootstepPlannerLogger logger = new FootstepPlannerLogger(planningModule);
      boolean success = logger.logSession(logDirectory);
      assertTrue(success, "Error generating footstep planner log");

      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      FootstepPlannerLogLoader.LoadResult loadResult = logLoader.load(new File(logger.getLatestLogDirectory()));
      Assertions.assertSame(loadResult, FootstepPlannerLogLoader.LoadResult.LOADED, "Error loading footstep planner log");

      FootstepPlannerLog log = logLoader.getLog();

      FootstepPlanningRequestPacket expectedRequestPacket = new FootstepPlanningRequestPacket();
      FootstepPlannerParametersPacket expectedFootstepParameters = new FootstepPlannerParametersPacket();
      VisibilityGraphsParametersPacket expectedBodyPathParameters = new VisibilityGraphsParametersPacket();
      FootstepPlanningToolboxOutputStatus expectedOutputStatusPacket = new FootstepPlanningToolboxOutputStatus();

      request.setPacket(expectedRequestPacket);
      FootstepPlannerMessageTools.copyParametersToPacket(expectedFootstepParameters, planningModule.getFootstepPlannerParameters());
      plannerOutput.setPacket(expectedOutputStatusPacket);

      assertTrue(expectedRequestPacket.epsilonEquals(log.getRequestPacket(), 1e-5));
      assertTrue(expectedFootstepParameters.epsilonEquals(log.getFootstepParametersPacket(), 1e-5));
      assertTrue(expectedOutputStatusPacket.epsilonEquals(log.getStatusPacket(), 1e-5));
   }
}
