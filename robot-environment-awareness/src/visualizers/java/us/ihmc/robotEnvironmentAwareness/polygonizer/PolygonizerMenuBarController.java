package us.ihmc.robotEnvironmentAwareness.polygonizer;

import java.io.IOException;
import java.util.concurrent.ExecutorService;

import javafx.fxml.FXML;
import javafx.stage.Window;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationRawDataImporter;

public class PolygonizerMenuBarController
{
   private JavaFXMessager messager;
   private ExecutorService executorService;
   private Window ownerWindow;

   public void initialize(JavaFXMessager messager, ExecutorService executorService, Window ownerWindow)
   {
      this.messager = messager;
      this.executorService = executorService;
      this.ownerWindow = ownerWindow;
   }

   @FXML
   public void reload()
   {
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationReload, true);
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
            dataImporter.recenterData();
            messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, dataImporter.getPlanarRegionSegmentationRawData());
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }
}
