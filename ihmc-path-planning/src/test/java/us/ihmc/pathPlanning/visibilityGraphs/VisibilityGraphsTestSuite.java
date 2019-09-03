package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3DTest;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegionTest;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTest;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterToolsTest;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionToolsTest;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudToolsTest;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsGeometryToolsTest;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityToolsTest;

public class VisibilityGraphsTestSuite
{
   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForPackage(VisibilityGraphsTestSuite.class);
   }
}
