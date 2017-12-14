package us.ihmc.pathPlanning.visibilityGraphs;

import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphsIOTools.VisibilityGraphsUnitTestDataset;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class VisibilityGraphsFrameworkTest
{
   private boolean debug = true;

   @Before
   public void setup()
   {
      debug = debug && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   }

   @Test(timeout = 30000)
   public void testASolutionExists() throws Exception
   {
      List<VisibilityGraphsUnitTestDataset> allDatasets = VisibilityGraphsIOTools.loadAllDatasets();

      if (debug)
      {
         PrintTools.info("Unit test files found: " + allDatasets.size());
      }

      for (VisibilityGraphsUnitTestDataset dataset : allDatasets)
      {
         testFile(dataset);
      }
   }

   private void testFile(VisibilityGraphsUnitTestDataset dataset)
   {
      if (debug)
      {
         PrintTools.info("Processing file: " + dataset.getDatasetName());
      }

      NavigableRegionsManager manager = new NavigableRegionsManager();
      manager.setPlanarRegions(dataset.getPlanarRegionsList().getPlanarRegionsAsList());

      List<Point3DReadOnly> path = manager.calculateBodyPath(dataset.getStart(), dataset.getGoal());

      assertTrue("Path is null!", path != null);
      assertTrue("Path does not contain any waypoints", path.size() > 0);

      if (dataset.hasExpectedPathSize())
         assertTrue("Path size is not equal", path.size() == dataset.getExpectedPathSize());
   }
}
