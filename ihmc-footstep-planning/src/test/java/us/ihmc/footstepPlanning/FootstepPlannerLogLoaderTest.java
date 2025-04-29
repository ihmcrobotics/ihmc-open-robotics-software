package us.ihmc.footstepPlanning;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.io.File;
import java.net.URL;

public class FootstepPlannerLogLoaderTest
{
   @Test
   @Disabled
   public void testLoadingLogWithTerrainMapData()
   {
      // Load the log from computer, user needs to have a log in order to run this test
      String filePath = "/home/ketchup/.ihmc/logs/astar_footstep_planner/20250429/20250429_103811492_FootstepPlannerLog";
      File footstepPlannerLogFile = new File(filePath);
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      logLoader.load(footstepPlannerLogFile);
      FootstepPlannerLog log = logLoader.getLog();

      // Set the request packet from the log data
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setFromPacket(log.getRequestPacket());

      TerrainMapData terrainMapData = request.getTerrainMapData();

      PerceptionDebugTools.printMat("HeightMap", terrainMapData.getHeightMap(), 10);
      PerceptionDebugTools.printMat("Connections", terrainMapData.getSteppabilityConnectionsMat(), 10);
      PerceptionDebugTools.printMat("TerrainCost", terrainMapData.getTerrainCostMap(), 10);
   }
}
