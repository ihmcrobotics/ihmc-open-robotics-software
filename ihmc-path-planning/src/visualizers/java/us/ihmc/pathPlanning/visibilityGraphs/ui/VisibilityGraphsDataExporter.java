package us.ihmc.pathPlanning.visibilityGraphs.ui;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.tools.thread.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.atomic.AtomicReference;

public class VisibilityGraphsDataExporter
{
   private final ExecutorService executor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   private final AtomicReference<PlanarRegionsList> planarRegionsState;
   private final AtomicReference<String> dataDirectoryPath;

   private AtomicReference<Point3D> start;
   private AtomicReference<Point3D> goal;

   public VisibilityGraphsDataExporter(Messager messager)
   {
      planarRegionsState = messager.createInput(UIVisibilityGraphsTopics.PlanarRegionData);
      start = messager.createInput(UIVisibilityGraphsTopics.StartPosition);
      goal = messager.createInput(UIVisibilityGraphsTopics.GoalPosition);
      dataDirectoryPath = messager.createInput(UIVisibilityGraphsTopics.exportUnitTestPath, null);
      messager.registerTopicListener(UIVisibilityGraphsTopics.exportUnitTestDataFile, this::exportVisibilityGraphsData);
   }

   private void exportVisibilityGraphsData(boolean export)
   {
      PlanarRegionsList planarRegionData = planarRegionsState.get();
      executor.execute(() -> executeOnThread(planarRegionData, start.get(), goal.get()));
   }

   private void executeOnThread(PlanarRegionsList planarRegionData, Point3D start, Point3D goal)
   {
      DataSet dataSet = new DataSet(PlanarRegionFileTools.getDate() + "_DataSet", planarRegionData);
      PlannerInput plannerInput = new PlannerInput();
      plannerInput.setStartPosition(start);
      plannerInput.setGoalPosition(goal);
      dataSet.setPlannerInput(plannerInput);

      DataSetIOTools.exportDataSet(dataDirectoryPath.get(), dataSet);
   }

   public void stop()
   {
      executor.shutdownNow();
   }
}
