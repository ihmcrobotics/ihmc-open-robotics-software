package us.ihmc.pathPlanning.visibilityGraphs;

import static org.junit.Assert.assertTrue;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
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
   private Point3D start;
   private Point3D goal;

   private boolean debug = true;

   @Before
   public void setup()
   {
      debug = debug && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   }

   @Test(timeout = 30000)
   public void testASolutionExists() throws Exception
   {
      Path filePath = Paths.get(getClass().getClassLoader().getResource("Data").getPath());
      File fileLocationsForAllData = filePath.toFile();

      if(debug)
      {
         PrintTools.info("Unit test files found: " + fileLocationsForAllData.listFiles().length);
      }

      File[] files = fileLocationsForAllData.listFiles();
      for (int i = 0; i < files.length; i++)
      {
         File fileFolder = files[i];
         String fileName = fileFolder.getName();

         if(debug)
         {
            PrintTools.info("Processing file: " + fileName);
         }

         if(!fileName.contains("UnitTest"))
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

         readStartGoalParameters(fileLocationForStartGoalParameters);

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
         testPathSize(fileLocationForStartGoalParameters, path);
      }
   }

   public void testPathSize(File file, List<Point3DReadOnly> path)
   {
      BufferedReader br = null;
      FileReader fr = null;

      try
      {

         fr = new FileReader(file);
         br = new BufferedReader(fr);

         String sCurrentLine;

         int index = 0;

         while ((sCurrentLine = br.readLine()) != null)
         {
            if (sCurrentLine.contains("<PathSize,") && sCurrentLine.contains(",PathSize>"))
            {
               double pathSize = Double.parseDouble(sCurrentLine.substring(10, sCurrentLine.indexOf(",PathSize>")));
               assertTrue("Path size is not equal",path.size() == pathSize);
            }
         }
      }
      catch (IOException e)
      {

         e.printStackTrace();

      } finally
      {

         try
         {

            if (br != null)
               br.close();

            if (fr != null)
               fr.close();

         }
         catch (IOException ex)
         {

            ex.printStackTrace();

         }
      }
   }

   public ArrayList<Point3D> readStartGoalParameters(File file)
   {
      BufferedReader br = null;
      FileReader fr = null;

      try
      {

         fr = new FileReader(file);
         br = new BufferedReader(fr);

         String sCurrentLine;

         int index = 0;

         while ((sCurrentLine = br.readLine()) != null)
         {
            if (sCurrentLine.contains("<Start,") && sCurrentLine.contains("Start>"))
            {
               String tempStart = sCurrentLine.substring(7, sCurrentLine.indexOf(",Start>"));
               start = getPoint3DFromStringSet(tempStart);
            }
            
            if (sCurrentLine.contains("<Goal,") && sCurrentLine.contains(",Goal>"))
            {
               String tempGoal = sCurrentLine.substring(6, sCurrentLine.indexOf(",Goal>"));
               goal = getPoint3DFromStringSet(tempGoal);
            }
         }
      }
      catch (IOException e)
      {

         e.printStackTrace();

      } finally
      {

         try
         {

            if (br != null)
               br.close();

            if (fr != null)
               fr.close();

         }
         catch (IOException ex)
         {

            ex.printStackTrace();

         }
      }
      return null;

   }

   private Point3D getPoint3DFromStringSet(String set)
   {

      double x = Double.parseDouble(set.substring(0, set.indexOf(",")));
      set = set.substring(set.indexOf(",") + 1);
      double y = Double.parseDouble(set.substring(0, set.indexOf(",")));
      set = set.substring(set.indexOf(",") + 1);
      double z = Double.parseDouble(set.substring(0));

      return new Point3D(x, y, z);
   }
}
