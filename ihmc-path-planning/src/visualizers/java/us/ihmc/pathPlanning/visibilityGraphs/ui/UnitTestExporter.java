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
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class UnitTestExporter
{
   private final ExecutorService executor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   private final AtomicReference<PlanarRegionsList> planarRegionsState;
   private final AtomicReference<String> dataDirectoryPath;

   private AtomicReference<Point3D> start;
   private AtomicReference<Point3D> goal;

   public UnitTestExporter(REAMessager messager)
   {
      planarRegionsState = messager.createInput(UIVisibilityGraphsTopics.PlanarRegionData);
      start = messager.createInput(UIVisibilityGraphsTopics.StartPosition);
      goal = messager.createInput(UIVisibilityGraphsTopics.GoalPosition);
      dataDirectoryPath = messager.createInput(UIVisibilityGraphsTopics.exportUnitTestPath, null);
      messager.registerTopicListener(UIVisibilityGraphsTopics.exportUnitTestDataFile, this::exportPlanarRegionData);
   }

   public UnitTestExporter(File dataDirectoryPath)
   {
      planarRegionsState = new AtomicReference<>(null);
      this.dataDirectoryPath = new AtomicReference<>(dataDirectoryPath.getAbsolutePath());
   }

   public void exportPlanarRegionData(PlanarRegionsList planarRegionsList)
   {
      planarRegionsState.set(planarRegionsList);
      exportPlanarRegionData(true);
   }

   public void exportPlanarRegionData(PlanarRegion planarRegion)
   {
      planarRegionsState.set(new PlanarRegionsList(planarRegion));
      exportPlanarRegionData(true);
   }

   private void exportPlanarRegionData(boolean export)
   {
      PlanarRegionsList planarRegionData = planarRegionsState.get();
      if (planarRegionData != null)
         executor.execute(() -> executeOnThread(planarRegionData));
   }

   private void executeOnThread(PlanarRegionsList planarRegionData)
   {
      if (dataDirectoryPath.get() == null)
      {
         PrintTools.error("The path to the data directory is null.");
         return;
      }

      Path folderPath = Paths.get(dataDirectoryPath.get());
      String datasetName = VisibilityGraphsIOTools.createDefaultTimeStampedDatasetFolderName();
      VisibilityGraphsIOTools.exportDataset(folderPath, datasetName, planarRegionData, start.get(), goal.get());
   }

   public void stop()
   {
      executor.shutdownNow();
   }
}
