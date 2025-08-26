package us.ihmc.footstepPlanning;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.footstepPlanning.steppableRegions.TerrainMapData;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.io.File;

public class FootstepPlannerLogLoaderTest
{
   @Test
   @Disabled
   public void testLoadingLogWithTerrainMapData()
   {
      // Note: This test requires that the user changes the file path to be to a location on their machine
      // Note: Load the log from computer, user needs to have a log in order to run this test
      String filePath = "/home/ketchup/.ihmc/logs/astar_footstep_planner/20250429/20250429_103811492_FootstepPlannerLog";
      File footstepPlannerLogFile = new File(filePath);
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      logLoader.load(footstepPlannerLogFile);
      FootstepPlannerLog log = logLoader.getLog();

      // Set the request packet from the log data
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setFromPacket(log.getRequestPacket());

      TerrainMapData terrainMapData = request.getEnvironmentHandler().getTerrainMapData();

      PerceptionDebugTools.printMat("TerrainCost", terrainMapData.getTerrainCostMap(), 10);
      PerceptionDebugTools.printMat("ContactMap", terrainMapData.getContactMap(), 10);

      PerceptionDebugTools.printMat("SnapNormalX", terrainMapData.getSnapNormalXMat(), 10);
      PerceptionDebugTools.printMat("SnapNormalY", terrainMapData.getSnapNormalYMat(), 10);
      PerceptionDebugTools.printMat("SnapNormalZ", terrainMapData.getSnapNormalZMat(), 10);
      PerceptionDebugTools.printMat("SnappedAreaFraction", terrainMapData.getSnappedAreaFractionMat(), 10);
      PerceptionDebugTools.printMat("Steppability", terrainMapData.getSteppabilityMat(), 10);
      PerceptionDebugTools.printMat("SteppableConnections", terrainMapData.getSteppabilityConnectionsMat(), 10);
   }
}
