package us.ihmc.quadrupedFootstepPlanning.pawPlanning.tools;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawPlannerType;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawPlannerMessagerAPI;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.atomic.AtomicReference;

public class PawPlannerDataExporter
{
   private final ExecutorService executor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   private final AtomicReference<PlanarRegionsList> planarRegionsState;
   private final AtomicReference<String> dataDirectoryPath;

   private final AtomicReference<Point3D> startPosition;
   private final AtomicReference<Point3D> goalPosition;
   private final AtomicReference<Quaternion> startOrientation;
   private final AtomicReference<Quaternion> goalOrientation;
   private final AtomicReference<Double> timeout;
   private final AtomicReference<PawPlannerType> plannerType;

   public PawPlannerDataExporter(Messager messager)
   {
      planarRegionsState = messager.createInput(PawPlannerMessagerAPI.PlanarRegionDataTopic);
      startPosition = messager.createInput(PawPlannerMessagerAPI.StartPositionTopic);
      startOrientation = messager.createInput(PawPlannerMessagerAPI.StartOrientationTopic);
      goalPosition = messager.createInput(PawPlannerMessagerAPI.GoalPositionTopic);
      goalOrientation = messager.createInput(PawPlannerMessagerAPI.GoalOrientationTopic);
      timeout = messager.createInput(PawPlannerMessagerAPI.PlannerTimeoutTopic);
      plannerType = messager.createInput(PawPlannerMessagerAPI.PlannerTypeTopic);
      dataDirectoryPath = messager.createInput(PawPlannerMessagerAPI.exportUnitTestPath, null);
      messager.registerTopicListener(PawPlannerMessagerAPI.exportUnitTestDataFile, this::exportPawPlannerData);
   }

   private void exportPawPlannerData(boolean export)
   {
      PlanarRegionsList planarRegionData = planarRegionsState.get();
      executor.execute(
            () -> executeOnThread(planarRegionData, startPosition.get(), startOrientation.get(), goalPosition.get(), goalOrientation.get(), plannerType.get(),
                                  timeout.get()));
   }

   private void executeOnThread(PlanarRegionsList planarRegionData, Point3DReadOnly startPosition, QuaternionReadOnly startOrientation,
                                Point3DReadOnly goalPosition, QuaternionReadOnly goalOrientation, PawPlannerType pawPlannerType, double timeout)
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

      if (startPosition == null)
      {
         PrintTools.error("No start position, not exporting the data.");
         return;
      }

      if (goalPosition == null)
      {
         PrintTools.error("No goal position, not exporting the data.");
         return;
      }

      if (pawPlannerType == null)
      {
         PrintTools.error("No planner type, not exporting the data.");
         return;
      }

      if (timeout < 0 || !Double.isFinite(timeout))
      {
         PrintTools.error("No timeout, not exporting the data.");
         return;
      }

      DataSet dataSet = new DataSet(PlanarRegionFileTools.getDate() + "_DataSet", planarRegionData);
      PlannerInput plannerInput = new PlannerInput();
      plannerInput.setStartPosition(startPosition);
      plannerInput.setGoalPosition(goalPosition);
      plannerInput.setStartYaw(startOrientation.getYaw());
      plannerInput.setGoalYaw(goalOrientation.getYaw());
      DataSetIOTools.exportDataSet(dataDirectoryPath.get(), dataSet);
   }

   public void stop()
   {
      executor.shutdownNow();
   }
}
