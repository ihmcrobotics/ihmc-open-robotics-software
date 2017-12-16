package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class VisibilityGraphsDataExporter
{
   private final ExecutorService executor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   private final AtomicReference<PlanarRegionsList> planarRegionsState;
   private final AtomicReference<String> dataDirectoryPath;

   private AtomicReference<Point3D> start;
   private AtomicReference<Point3D> goal;

   public VisibilityGraphsDataExporter(REAMessager messager)
   {
      planarRegionsState = messager.createInput(UIVisibilityGraphsTopics.PlanarRegionData);
      start = messager.createInput(UIVisibilityGraphsTopics.StartPosition);
      goal = messager.createInput(UIVisibilityGraphsTopics.GoalPosition);
      dataDirectoryPath = messager.createInput(UIVisibilityGraphsTopics.exportUnitTestPath, null);
      messager.registerTopicListener(UIVisibilityGraphsTopics.exportUnitTestDataFile, this::exportVisibilityGraphsData);
   }

   public VisibilityGraphsDataExporter(File dataDirectoryPath)
   {
      planarRegionsState = new AtomicReference<>(null);
      this.dataDirectoryPath = new AtomicReference<>(dataDirectoryPath.getAbsolutePath());
   }

   private void exportVisibilityGraphsData(boolean export)
   {
      PlanarRegionsList planarRegionData = planarRegionsState.get();
      executor.execute(() -> executeOnThread(planarRegionData, start.get(), goal.get()));
   }

   private void executeOnThread(PlanarRegionsList planarRegionData, Point3D start, Point3D goal)
   {
      if (dataDirectoryPath.get() == null)
      {
         PrintTools.error("The path to the data directory is null.");
         return;
      }

      if (planarRegionData == null)
      {
         PrintTools.error("No planar regions, not exporting the data.");
         return;
      }

      if (start == null)
      {
         PrintTools.error("No start position, not exporting the data.");
         return;
      }

      if (goal == null)
      {
         PrintTools.error("No goal position, not exporting the data.");
         return;
      }

      Path folderPath = Paths.get(dataDirectoryPath.get());
      String datasetName = VisibilityGraphsIOTools.createDefaultTimeStampedDatasetFolderName();
      VisibilityGraphsIOTools.exportDataset(folderPath, datasetName, planarRegionData, start, goal);
   }

   public void stop()
   {
      executor.shutdownNow();
   }
}
