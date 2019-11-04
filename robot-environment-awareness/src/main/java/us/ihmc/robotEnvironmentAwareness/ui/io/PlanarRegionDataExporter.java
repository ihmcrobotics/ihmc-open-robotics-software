package us.ihmc.robotEnvironmentAwareness.ui.io;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.Executor;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import javafx.stage.Window;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionDataExporter
{
   private final Executor executor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   private final AtomicReference<PlanarRegionsListMessage> planarRegionsState;
   private final AtomicReference<String> dataDirectoryPath;

   public PlanarRegionDataExporter(REAUIMessager uiMessager)
   {
      planarRegionsState = uiMessager.createInput(REAModuleAPI.PlanarRegionsState);
      dataDirectoryPath = uiMessager.createInput(REAModuleAPI.UIPlanarRegionDataExporterDirectory, new File("Data/PlanarRegion/").getAbsolutePath());
      uiMessager.registerTopicListener(REAModuleAPI.UIPlanarRegionDataExportRequest, this::exportPlanarRegionData);
   }

   public PlanarRegionDataExporter(File dataDirectoryPath)
   {
      planarRegionsState = new AtomicReference<>(null);
      this.dataDirectoryPath = new AtomicReference<>(dataDirectoryPath.getAbsolutePath());
   }

   public void exportPlanarRegionData(PlanarRegionsList planarRegionsList)
   {
      planarRegionsState.set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
      exportPlanarRegionData(true);
   }

   public void exportPlanarRegionData(PlanarRegion planarRegion)
   {
      planarRegionsState.set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(new PlanarRegionsList(planarRegion)));
      exportPlanarRegionData(true);
   }

   private void exportPlanarRegionData(boolean export)
   {
      PlanarRegionsListMessage planarRegionData = planarRegionsState.get();
      if (planarRegionData != null)
         executor.execute(() -> executeOnThread(planarRegionData));
   }

   private void executeOnThread(PlanarRegionsListMessage planarRegionData)
   {
      Path folderPath = Paths.get(dataDirectoryPath.get() + File.separator + PlanarRegionFileTools.createDefaultTimeStampedFolderName());
      PlanarRegionFileTools.exportPlanarRegionData(folderPath, PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionData));
   }

   // STATIC TOOL

   public static void exportUsingFileChooser(Window ownerWindow, PlanarRegionsList planarRegionsList)
   {
      File dataFolder = PlanarRegionDataImporter.chooseFile(ownerWindow);
      if (dataFolder != null)
      {
         PlanarRegionFileTools.exportPlanarRegionData(dataFolder.toPath(), planarRegionsList);
      }
   }
}
