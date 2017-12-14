package us.ihmc.pathPlanning.visibilityGraphs;

import static org.junit.Assert.assertTrue;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import org.junit.Before;
import org.junit.Test;

import javafx.application.Platform;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

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
      String pathString = getClass().getClassLoader().getResource("Data").getPath();
      if (VisibilityGraphsIOTools.isWindows())
         pathString = pathString.substring(1, pathString.length());

      Path filePath = Paths.get(pathString);
      File fileLocationsForAllData = filePath.toFile();

      if (debug)
      {
         PrintTools.info("Unit test files found: " + fileLocationsForAllData.listFiles().length);
      }

      File[] files = fileLocationsForAllData.listFiles();
      for (int i = 0; i < files.length; i++)
      {
         File fileFolder = files[i];
         String fileName = fileFolder.getName();

         if (debug)
         {
            PrintTools.info("Processing file: " + fileName);
         }

         if (!fileName.contains("UnitTest"))
            continue;

         String simpleFileName = fileName.replace("_UnitTest", "");
         File fileLocationForPlanarRegions = new File(fileFolder.getPath(), simpleFileName);
         File fileLocationForStartGoalParameters = new File(fileFolder.getPath(), "UnitTestParameters.txt");

         PlanarRegionsList planarRegionData = null;

         if (fileLocationForPlanarRegions != null)
            planarRegionData = PlanarRegionDataImporter.importPlanRegionData(fileLocationForPlanarRegions);

         if (planarRegionData == null)
            Platform.exit();

         if (debug)
         {
            PrintTools.info("Running test for : " + fileFolder.getName());
         }

         Point3D start = new Point3D();
         Point3D goal = new Point3D();
         VisibilityGraphsIOTools.readStartGoalParameters(fileLocationForStartGoalParameters, start, goal);
         int expectedPathSize = VisibilityGraphsIOTools.parsePathSize(fileLocationForStartGoalParameters);

         List<PlanarRegion> regions = planarRegionData.getPlanarRegionsAsList();
         ArrayList<PlanarRegion> filteredRegions = new ArrayList<>();

         for (PlanarRegion region : regions)
         {
            if (region.getConcaveHullSize() > 2)
            {
               filteredRegions.add(region);
            }
         }

         NavigableRegionsManager manager = new NavigableRegionsManager(new DefaultVisibilityGraphParameters(), filteredRegions);

         List<Point3DReadOnly> path = manager.calculateBodyPath(start, goal);

         assertTrue("Path is null!", path != null);
         assertTrue("Path does not contain any waypoints", path.size() > 0);
         assertTrue("Path size is not equal", path.size() == expectedPathSize);

      }
   }
}
