package us.ihmc.robotEnvironmentAwareness.ui.io;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.converters.REAPlanarRegionsConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationNodeData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;

public class PlanarRegionSegmentationDataExporter
{
   private final Executor executor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   private final AtomicReference<PlanarRegionSegmentationMessage[]> planarRegionSegmentationState;
   private final AtomicReference<String> dataDirectoryPath;

   public PlanarRegionSegmentationDataExporter(REAUIMessager uiMessager)
   {
      planarRegionSegmentationState = uiMessager.createInput(REAModuleAPI.PlanarRegionsSegmentationState);
      dataDirectoryPath = uiMessager.createInput(REAModuleAPI.UISegmentationDataExporterDirectory, new File("Data/Segmentation/").getAbsolutePath());
      uiMessager.registerTopicListener(REAModuleAPI.UISegmentationDataExportRequest, this::exportSegmentationData);
   }

   public PlanarRegionSegmentationDataExporter(File dataDirectoryPath)
   {
      planarRegionSegmentationState = new AtomicReference<>(null);
      this.dataDirectoryPath = new AtomicReference<>(dataDirectoryPath.getAbsolutePath());
   }

   public void exportSegmentationRawData(List<PlanarRegionSegmentationRawData> rawData)
   {
      planarRegionSegmentationState.set(PlanarRegionSegmentationRawData.toMessageArray(rawData));
      exportSegmentationData(true);
   }

   public void exportSegmentationRawData(PlanarRegionSegmentationRawData rawData)
   {
      planarRegionSegmentationState.set(new PlanarRegionSegmentationMessage[] {rawData.toMessage()});
      exportSegmentationData(true);
   }

   public void exportSegmentationData(List<PlanarRegionSegmentationNodeData> nodeData)
   {
      planarRegionSegmentationState.set(REAPlanarRegionsConverter.createPlanarRegionSegmentationMessages(nodeData));
      exportSegmentationData(true);
   }

   public void exportSegmentationData(PlanarRegionSegmentationNodeData nodeData)
   {
      PlanarRegionSegmentationMessage message = REAPlanarRegionsConverter.createPlanarRegionSegmentationMessage(nodeData);
      planarRegionSegmentationState.set(new PlanarRegionSegmentationMessage[] {message});
      exportSegmentationData(true);
   }

   private void exportSegmentationData(boolean export)
   {
      PlanarRegionSegmentationMessage[] segmentationData = planarRegionSegmentationState.get();
      if (segmentationData != null)
         executor.execute(() -> executeOnThread(segmentationData));
   }

   private void executeOnThread(PlanarRegionSegmentationMessage[] segmentationData)
   {
      Path folderPath = Paths.get(dataDirectoryPath.get() + File.separator + getDate() + "PlanarRegionSegmentation");
      try
      {
         if (folderPath.toFile().exists())
            return;
         Files.createDirectories(folderPath);
         File header = new File(folderPath.toFile(), "header.txt");
         writeHeaderFile(header, segmentationData);
         writeRegionData(folderPath, segmentationData);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return;
      }
   }

   private void writeHeaderFile(File header, PlanarRegionSegmentationMessage[] segmentationData) throws IOException
   {
      FileWriter fileWriter = new FileWriter(header);

      for (PlanarRegionSegmentationMessage message : segmentationData)
      {
         Point3D32 origin = message.getOrigin();
         Vector3D32 normal = message.getNormal();
         fileWriter.write("regionId: ");
         fileWriter.write(Integer.toString(message.getRegionId()));
         fileWriter.write(", origin: ");
         fileWriter.write(origin.getX() + ", " + origin.getY() + ", " + origin.getZ());
         fileWriter.write(", normal: ");
         fileWriter.write(normal.getX() + ", " + normal.getY() + ", " + normal.getZ());
         fileWriter.write("\n");
      }

      fileWriter.close();
   }

   private void writeRegionData(Path folderPath, PlanarRegionSegmentationMessage[] segmentationData) throws IOException
   {
      for (PlanarRegionSegmentationMessage message : segmentationData)
      {
         File regionFile = new File(folderPath.toFile(), "region" + message.getRegionId());
         FileWriter fileWriter = new FileWriter(regionFile);

         for (Point3D32 hitLocation : message.getHitLocations())
         {
            fileWriter.write(hitLocation.getX() + ", " + hitLocation.getY() + ", " + hitLocation.getZ() + "\n");
         }

         fileWriter.close();
      }
   }

   private static String getDate()
   {
      DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_");
      return LocalDateTime.now().format(dateTimeFormatter);
   }
}
