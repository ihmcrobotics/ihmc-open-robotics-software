package us.ihmc.parameterTuner.guiElements.main;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;

import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.ButtonType;
import javafx.scene.layout.Region;
import javafx.stage.FileChooser;
import us.ihmc.parameterTuner.JavaFXExceptionTools;
import us.ihmc.parameterTuner.ParameterSavingTools;
import us.ihmc.parameterTuner.ParameterTuningTools;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.parameterTuner.guiElements.tuners.Tuner;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class ParameterFileLoader
{
   private Map<String, Tuner> tunerMap;

   public void load(Scene scene)
   {
      if (tunerMap == null)
      {
         return;
      }

      displayWarning();

      List<GuiRegistry> registries = showUserDialogAndParseFile(scene);
      if (registries == null)
      {
         return;
      }

      String message = "";
      for (GuiRegistry registry : registries)
      {
         List<GuiParameter> allParameters = registry.getAllParameters();
         for (GuiParameter parameter : allParameters)
         {
            Tuner tuner = tunerMap.get(parameter.getUniqueName());
            if (tuner == null)
            {
               message += parameter.getUniqueName() + "\n";
            }
            else
            {
               Platform.runLater(() -> tuner.getParameter().setValue(parameter.getCurrentValue()));
            }
         }
      }

      if (!message.isEmpty())
      {
         JavaFXExceptionTools.createExceptionDialog(new Throwable("Tried to load non-existing parameters\n" + message));
      }
   }

   private boolean displayWarning()
   {
      Alert alert = new Alert(AlertType.CONFIRMATION);
      alert.setTitle("Load File");
      alert.getDialogPane().setMinHeight(Region.USE_PREF_SIZE);
      alert.getDialogPane().setMinWidth(Region.USE_PREF_SIZE);
      alert.setHeaderText("Do you want to load values?");
      alert.setContentText("This will modify all values defined in the file at once.");
      return alert.showAndWait().get() == ButtonType.OK;
   }

   public void setTunerMap(Map<String, Tuner> tunerMap)
   {
      this.tunerMap = tunerMap;
   }

   private List<GuiRegistry> showUserDialogAndParseFile(Scene scene)
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.getExtensionFilters().add(ParameterSavingTools.getExtensionFilter());
      fileChooser.setTitle("Select Parameter File");

      // If a file is open initialize the folder path to the directory of the active file:
      File initialPath = ParameterSavingNode.getDefaultFilePath();
      if (initialPath != null)
      {
         fileChooser.setInitialDirectory(initialPath);
      }
      File file = fileChooser.showOpenDialog(scene.getWindow());

      return openFileSafe(file);
   }

   private List<GuiRegistry> openFileSafe(File file)
   {
      try
      {
         return openFile(file);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   private List<GuiRegistry> openFile(File file) throws IOException
   {
      if (file != null)
      {
         List<Registry> xmlRegistries = ParameterTuningTools.getParameters(file);
         return ParameterTuningTools.buildGuiRegistryFromXML(xmlRegistries);
      }
      return null;
   }
}
