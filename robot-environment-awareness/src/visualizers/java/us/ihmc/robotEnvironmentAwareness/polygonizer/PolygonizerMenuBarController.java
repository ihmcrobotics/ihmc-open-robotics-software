package us.ihmc.robotEnvironmentAwareness.polygonizer;

import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javafx.fxml.FXML;
import javafx.stage.Window;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationRawDataImporter;

public class PolygonizerMenuBarController
{
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private JavaFXMessager messager;
   private Window ownerWindow;

   public void initialize(JavaFXMessager messager, Window ownerWindow)
   {
      this.messager = messager;
      this.ownerWindow = ownerWindow;
   }

   @FXML
   public void load()
   {
      PlanarRegionSegmentationRawDataImporter dataImporter = PlanarRegionSegmentationRawDataImporter.createImporterWithFileChooser(ownerWindow);
      executorService.execute(() -> loadOnThread(dataImporter));
   }

   private void loadOnThread(PlanarRegionSegmentationRawDataImporter dataImporter)
   {
      if (dataImporter != null)
      {
         try
         {
            dataImporter.loadPlanarRegionSegmentationData();
            messager.submitMessage(Polygonizer.PolygonizerInput, Polygonizer.toInputList(dataImporter.getPlanarRegionSegmentationRawData()));
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   public void shutdown()
   {
      executorService.shutdownNow();
   }
}
