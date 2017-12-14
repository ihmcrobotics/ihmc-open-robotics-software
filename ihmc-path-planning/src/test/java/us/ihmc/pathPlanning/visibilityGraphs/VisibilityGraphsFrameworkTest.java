package us.ihmc.pathPlanning.visibilityGraphs;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphsIOTools.VisibilityGraphsUnitTestDataset;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.pathPlanning.visibilityGraphs.visualizer.VisibilityGraphsTestVisualizer;
import us.ihmc.robotics.geometry.PlanarRegionsList;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class VisibilityGraphsFrameworkTest extends Application
{
   private static boolean VISUALIZE = true;
   private boolean DEBUG = true;

   private static final SimpleUIMessager messager = new SimpleUIMessager(UIVisibilityGraphsTopics.API);
   private static VisibilityGraphsTestVisualizer ui;

   private static final boolean showBodyPath = true;
   private static final boolean showClusterRawPoints = false;
   private static final boolean showClusterNavigableExtrusions = false;
   private static final boolean showClusterNonNavigableExtrusions = false;
   private static final boolean showRegionInnerConnections = false;
   private static final boolean showRegionInterConnections = false;

   @Before
   public void setup() throws InterruptedException, IOException
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      DEBUG = (VISUALIZE || (DEBUG && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer()));

      if (VISUALIZE)
      {
         messager.startMessager();

         new Thread(() -> launch()).start();

         while (ui == null)
            ThreadTools.sleep(200);

         messager.submitMessage(UIVisibilityGraphsTopics.ShowBodyPath, showBodyPath);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterRawPoints, showClusterRawPoints);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterNavigableExtrusions, showClusterNavigableExtrusions);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusions);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowLocalGraphs, showRegionInnerConnections);
         messager.submitMessage(UIVisibilityGraphsTopics.ShowInterConnections, showRegionInterConnections);
      }
   }

   @After
   public void tearDown() throws Exception
   {
      if (VISUALIZE)
         stop();
   }

   @Test
   public void testASolutionExists() throws Exception
   {
      List<VisibilityGraphsUnitTestDataset> allDatasets = VisibilityGraphsIOTools.loadAllDatasets();

      if (DEBUG)
      {
         PrintTools.info("Unit test files found: " + allDatasets.size());
      }

      AtomicReference<Boolean> nextDatasetRequested = null;
      if (VISUALIZE)
         nextDatasetRequested = messager.createInput(UIVisibilityGraphsTopics.NextDatasetRequest, false);

      for (VisibilityGraphsUnitTestDataset dataset : allDatasets)
      {
         if (VISUALIZE)
            messager.submitMessage(UIVisibilityGraphsTopics.GlobalReset, true);

         testFile(dataset);

         if (VISUALIZE)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.NextDatasetRequest, false);

            while (!nextDatasetRequested.get())
            {
               if (!messager.isMessagerOpen())
                  return; // The ui has been closed

               ThreadTools.sleep(200);
            }
         }
      }
   }

   private void testFile(VisibilityGraphsUnitTestDataset dataset)
   {
      if (DEBUG)
      {
         PrintTools.info("Processing file: " + dataset.getDatasetName());
      }

      NavigableRegionsManager manager = new NavigableRegionsManager();
      PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();

      if (VISUALIZE)
      {
         messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, planarRegionsList);
         messager.submitMessage(UIVisibilityGraphsTopics.StartPosition, dataset.getStart());
         messager.submitMessage(UIVisibilityGraphsTopics.GoalPosition, dataset.getGoal());
      }

      manager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

      List<Point3DReadOnly> path = null;

      try
      {
         path = manager.calculateBodyPath(dataset.getStart(), dataset.getGoal());
      }
      catch (Exception e)
      {

      }

      if (VISUALIZE)
      {
         if (path != null)
         {
            messager.submitMessage(UIVisibilityGraphsTopics.BodyPathData, path);
         }
         messager.submitMessage(UIVisibilityGraphsTopics.NavigableRegionData, manager.getListOfLocalPlanners());
         messager.submitMessage(UIVisibilityGraphsTopics.InterRegionConnectionData, manager.getConnectionPoints());
      }

      assertTrue("Path is null!", path != null);
      if (path != null)
         assertTrue("Path does not contain any waypoints", path.size() > 0);

      if (dataset.hasExpectedPathSize())
         assertTrue("Path size is not equal", path.size() == dataset.getExpectedPathSize());

   }

   private void assertTrue(String message, boolean condition)
   {
      if (VISUALIZE)
      {
         if (!condition)
            PrintTools.error(message);
      }
      else
      {
         Assert.assertTrue(message, condition);
      }
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ui = new VisibilityGraphsTestVisualizer(primaryStage, messager);
      ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
   }
}
