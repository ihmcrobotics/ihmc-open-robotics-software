package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.Executor;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class UnitTestExporter
{
   private final Executor executor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   private final AtomicReference<PlanarRegionsList> planarRegionsState;
   private final AtomicReference<String> dataDirectoryPath;

   private AtomicReference<Point3D> start;
   private AtomicReference<Point3D> goal;

   public UnitTestExporter(REAMessager messager)
   {
      planarRegionsState = messager.createInput(UIVisibilityGraphsTopics.PlanarRegionData);
      start = messager.createInput(UIVisibilityGraphsTopics.StartPosition);
      goal = messager.createInput(UIVisibilityGraphsTopics.GoalPosition);
      dataDirectoryPath = messager.createInput(UIVisibilityGraphsTopics.exportUnitTestPath, new File("Data/PlanarRegion/").getAbsolutePath());
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
      Path folderPath = Paths.get(dataDirectoryPath.get() + File.separator + PlanarRegionFileTools.createDefaultTimeStampedFolderName() + "_UnitTest");

      Path folderPath2 = Paths.get(dataDirectoryPath.get() + File.separator + PlanarRegionFileTools.createDefaultTimeStampedFolderName() + "_UnitTest"
            + File.separator + PlanarRegionFileTools.createDefaultTimeStampedFolderName());

      PlanarRegionFileTools.exportPlanarRegionData(folderPath2, planarRegionData);
      createUnitTestParametersFile(folderPath.toString());
   }

   private void createUnitTestParametersFile(String folderPath)
   {
      String filename = "UnitTestParameters.txt";

      File file = new File(folderPath + File.separator + filename);

      try
      {
         // if file doesnt exists, then create it
         if (!file.exists())
         {
            file.createNewFile();
         }

         FileWriter fw = new FileWriter(file.getAbsoluteFile());
         BufferedWriter bw = new BufferedWriter(fw);

         bw.write("<Start," + start.get().getX() + "," + start.get().getY() + "," + start.get().getZ() + ",Start>");
         bw.write(System.getProperty("line.separator"));

         bw.write("<Goal," + goal.get().getX() + "," + goal.get().getY() + "," + goal.get().getZ() + ",Goal>");
         bw.write(System.getProperty("line.separator"));

         bw.close();
      }
      catch (IOException e1)
      {
         // TODO Auto-generated catch block
         e1.printStackTrace();
      }
   }
}
