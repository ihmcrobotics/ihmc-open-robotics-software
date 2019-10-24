package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapSolution;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

/**
 * Visualizer for examining visibility graph construction looking at a single pair of planar regions at a time from one of the test case directories.
 * This helps reduce the clutter and focus on each pair.
 * Good for finding bugs, tuning parameters, and finding good test cases to isolate for automatic tests.
 *
 */
public class VisibilityGraphsPlanarRegionPairsDatasetVisualizer
{
   private static final boolean fullyExpandVisibilityGraph = true;

   public static void openResourceAndVisualizePairs(String dataSetName)
   {
      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);
      String datasetName = dataSet.getName();
      LogTools.info("Loaded " + datasetName);

      Point3D start = dataSet.getPlannerInput().getStartPosition();
      Point3D goal = dataSet.getPlannerInput().getGoalPosition();

      PlanarRegionsList planarRegionsList = dataSet.getPlanarRegionsList();

      VisibilityGraphsTestVisualizerApplication visualizerApplication = new VisibilityGraphsTestVisualizerApplication();
      visualizerApplication.startOnAThread();

      SharedMemoryJavaFXMessager messager = visualizerApplication.getMessager();
      AtomicReference<Boolean> previousDatasetRequested = messager.createInput(UIVisibilityGraphsTopics.PreviousDatasetRequest, false);
      AtomicReference<Boolean> nextDatasetRequested = messager.createInput(UIVisibilityGraphsTopics.NextDatasetRequest, false);

      //TODO: Make the export button appear, instead of having to click reload...
      AtomicReference<Boolean> reloadDatasetRequested = messager.createInput(UIVisibilityGraphsTopics.ReloadDatasetRequest, false);
      //      AtomicReference<Boolean> exportUnitTestRequested = messager.createInput(UIVisibilityGraphsTopics.exportUnitTestDataFile, false);

      solveAndVisualize(start, goal, planarRegionsList, visualizerApplication);

      ThreadTools.sleep(3000);

      List<PlanarRegion> planarRegionsAsList = planarRegionsList.getPlanarRegionsAsList();

      int pairNumber = -1;

      int size = planarRegionsAsList.size();

      int indexOne = 0;
      int indexTwo = 1;

      boolean incrementIndices = true;

      //TODO: Also have unit tests for these, but what are the asserts?
      while (true)
      {
         PlanarRegion planarRegionOne = planarRegionsAsList.get(indexOne);

         PlanarRegion planarRegionTwo = planarRegionsAsList.get(indexTwo);

         PlanarRegionsList listToVisualize = new PlanarRegionsList();
         listToVisualize.addPlanarRegion(planarRegionOne);
         listToVisualize.addPlanarRegion(planarRegionTwo);

         DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();

         NavigableRegions navigableRegions = new NavigableRegions(parameters, listToVisualize);
         navigableRegions.createNavigableRegions();

         List<NavigableRegion> naviableRegionsList = navigableRegions.getNaviableRegionsList();

         if (!naviableRegionsList.isEmpty())
         {
            NavigableRegion region0 = naviableRegionsList.get(0);

            List<Cluster> obstacles0 = region0.getObstacleClusters();

            if (!obstacles0.isEmpty())
            {
               if (incrementIndices)
                  pairNumber++;
               else
                  pairNumber--;

               System.out.println("Pair number: " + pairNumber + ", indexOne = " + indexOne + ", indexTwo = " + indexTwo);
               solveAndVisualize(start, goal, listToVisualize, visualizerApplication);

               Boolean previousRequested = previousDatasetRequested.get();
               Boolean nextRequested = nextDatasetRequested.get();
               while (!previousRequested && !nextRequested)
               {
                  if (reloadDatasetRequested.get())
                  {
                     exportPlanarRegionPairDataset(pairNumber, indexOne, indexTwo, planarRegionOne, planarRegionTwo, start, goal);
                     messager.submitMessage(UIVisibilityGraphsTopics.ReloadDatasetRequest, false);
                     ThreadTools.sleep(300);
                  }

                  ThreadTools.sleep(100);
                  previousRequested = previousDatasetRequested.get();
                  nextRequested = nextDatasetRequested.get();
               }

               if (previousRequested)
               {
                  incrementIndices = false;
               }
               if (nextRequested)
               {
                  incrementIndices = true;
               }

               messager.submitMessage(UIVisibilityGraphsTopics.PreviousDatasetRequest, false);
               messager.submitMessage(UIVisibilityGraphsTopics.NextDatasetRequest, false);
            }
         }

         int[] newIndices;

         if (incrementIndices)
         {
            newIndices = incrementIndices(indexOne, indexTwo, size);
         }

         else
         {
            newIndices = decrementIndices(indexOne, indexTwo, size);
         }

         indexOne = newIndices[0];
         indexTwo = newIndices[1];
      }
   }

   private static int[] incrementIndices(int indexOne, int indexTwo, int size)
   {
      indexTwo++;

      if (indexTwo >= size)
      {
         indexTwo = 0;
         indexOne = (indexOne + 1) % size;
      }
      if (indexOne == indexTwo)
      {
         return incrementIndices(indexOne, indexTwo, size);
      }

      return new int[] {indexOne, indexTwo};
   }

   private static int[] decrementIndices(int indexOne, int indexTwo, int size)
   {
      indexTwo--;

      if (indexTwo < 0)
      {
         indexTwo = size - 1;
         indexOne--;

         if (indexOne < 0)
         {
            indexOne = size - 1;
         }
      }
      if (indexOne == indexTwo)
      {
         return decrementIndices(indexOne, indexTwo, size);
      }

      return new int[] {indexOne, indexTwo};
   }

   private static void solveAndVisualize(Point3D start, Point3D goal, PlanarRegionsList planarRegionsList,
                                         VisibilityGraphsTestVisualizerApplication visualizerApplication)
   {
      visualizerApplication.submitPlanarRegionsListToVisualizer(planarRegionsList);

      DefaultVisibilityGraphParameters parameters = new DefaultVisibilityGraphParameters();
      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(parameters, planarRegionsList.getPlanarRegionsAsList());
      navigableRegionsManager.calculateBodyPath(start, goal, fullyExpandVisibilityGraph);
      VisibilityMapSolution visibilityMapSolution = navigableRegionsManager.getVisibilityMapSolution();
      visualizerApplication.submitVisibilityGraphSolutionToVisualizer(visibilityMapSolution);
   }

   private static void exportPlanarRegionPairDataset(int pairIndex, int indexOne, int indexTwo, PlanarRegion planarRegionOne,
                                                     PlanarRegion planarRegionTwo, Point3D start, Point3D goal)
   {
      PlanarRegionsList planarRegionsListToExport = new PlanarRegionsList();
      planarRegionsListToExport.addPlanarRegion(planarRegionOne);
      planarRegionsListToExport.addPlanarRegion(planarRegionTwo);

      String datasetNameToExport = PlanarRegionFileTools.getDate() + "_" + pairIndex + "_" + indexOne + "_" + indexTwo;
      DataSet dataSet = new DataSet(datasetNameToExport, planarRegionsListToExport);
      PlannerInput plannerInput = new PlannerInput();
      plannerInput.setStartPosition(start);
      plannerInput.setGoalPosition(goal);
      dataSet.setPlannerInput(plannerInput);
      DataSetIOTools.exportDataSet(dataSet);
   }

   public static void main(String[] args)
   {
      LogTools.info("Click on Next to cycle through the PlanarRegion pairs.");
      String resourceName = "20171215_214730_CinderBlockField";
      openResourceAndVisualizePairs(resourceName);
   }

}
