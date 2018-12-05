package us.ihmc.pathPlanning.visibilityGraphs;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3DTest;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegionTest;
import us.ihmc.pathPlanning.visibilityGraphs.dijkstra.DijkstraVisibilityGraphPlannerTest;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTest;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterToolsTest;
import us.ihmc.pathPlanning.visibilityGraphs.tools.JGraphToolsTest;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionToolsTest;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudToolsTest;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsGeometryToolsTest;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityToolsTest;

@RunWith(Suite.class)
@Suite.SuiteClasses({VisibilityToolsTest.class, VisibilityGraphsFactoryTest.class, ConnectionPoint3DTest.class, NavigableRegionTest.class, ClusterTest.class,
      ClusterToolsTest.class, JGraphToolsTest.class, PlanarRegionToolsTest.class, PointCloudToolsTest.class, VisibilityGraphsGeometryToolsTest.class,
      VisibilityToolsTest.class, DijkstraVisibilityGraphPlannerTest.class
//      WaypointDefinedBodyPathPlan.class,
      //      VisibilityGraphsFrameworkTest.class, 
      //      VisibilityGraphOcclusionTest.class, 
})

public class VisibilityGraphsTestSuite
{
   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForPackage(VisibilityGraphsTestSuite.class);
   }
}
