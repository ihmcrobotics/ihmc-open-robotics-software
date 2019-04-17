package us.ihmc.parameterTuner.guiElements.main;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.prefs.Preferences;

import javafx.event.ActionEvent;
import javafx.geometry.Pos;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.layout.HBox;
import javafx.scene.text.Text;
import javafx.stage.FileChooser;
import us.ihmc.parameterTuner.ParameterSavingTools;
import us.ihmc.parameterTuner.ParameterTuningTools;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class ParameterSavingNode extends HBox
{
   private final CheckBox merge = new CheckBox("Merge");
   private final CheckBox modified = new CheckBox("Modified Only");
   private final Button save = new Button("Save");
   private final Button saveAs = new Button("Save as...");
   private final Text fileName = new Text();

   private File activeFile;
   private List<GuiRegistry> registries;
   private List<String> rootRegistryNames;

   private final List<ParameterFileSavedListener> listeners = new ArrayList<>();

   public ParameterSavingNode(boolean showFile, boolean enableSave)
   {
      setupNode(showFile, enableSave);
      setActiveFile(null);
   }

   public void addSavedListener(ParameterFileSavedListener listener)
   {
      listeners.add(listener);
   }

   private void informListeners()
   {
      listeners.stream().forEach(listener -> listener.parameterFileSaved(activeFile));
   }

   public void setActiveFile(File activeFile)
   {
      this.activeFile = activeFile;
      setDefaultFilePath(activeFile);

      if (activeFile != null)
      {
         save.setDisable(false);
         fileName.setText(activeFile.getName());
      }
      else
      {
         save.setDisable(true);
         fileName.setText("");
      }
   }

   public void setRegistries(List<GuiRegistry> registries)
   {
      this.registries = registries;
   }

   public void setRootRegistries(List<String> rootRegistryNames)
   {
      this.rootRegistryNames = rootRegistryNames;
   }

   private void setupNode(boolean showFile, boolean enableSave)
   {
      merge.setSelected(true);
      modified.setSelected(true);

      setMaxHeight(Double.NEGATIVE_INFINITY);
      setMaxWidth(Double.NEGATIVE_INFINITY);
      setSpacing(10.0);
      setAlignment(Pos.CENTER_LEFT);

      getChildren().add(modified);
      getChildren().add(merge);
      if (enableSave)
      {
         getChildren().add(save);
      }
      getChildren().add(saveAs);
      if (showFile)
      {
         getChildren().add(fileName);
      }

      save.setOnAction(event -> {
         try
         {
            handleSave(event);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      });

      saveAs.setOnAction(event -> {
         try
         {
            handleSaveAs(event);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      });
   }

   private void handleSave(ActionEvent event) throws IOException
   {
      List<GuiRegistry> rootRegistries = ParameterSavingTools.findRootRegistries(registries, rootRegistryNames);

      List<GuiRegistry> registriesAfterModified;
      if (modified.isSelected())
      {
         registriesAfterModified = ParameterSavingTools.filterModified(rootRegistries);
      }
      else
      {
         registriesAfterModified = rootRegistries;
      }

      List<GuiRegistry> registriesAfterMerge;
      if (merge.isSelected() && activeFile.exists())
      {
         List<Registry> xmlRegistries = ParameterTuningTools.getParameters(activeFile);
         List<GuiRegistry> existingRegistry = ParameterTuningTools.buildGuiRegistryFromXML(xmlRegistries);
         registriesAfterMerge = ParameterSavingTools.merge(existingRegistry, registriesAfterModified);
      }
      else
      {
         registriesAfterMerge = registriesAfterModified;
      }

      List<Registry> xmlRegistries = ParameterTuningTools.buildXMLRegistriesFromGui(registriesAfterMerge);
      ParameterSavingTools.save(activeFile, xmlRegistries);
      setDefaultFilePath(activeFile);
      informListeners();
   }

   private void handleSaveAs(ActionEvent event) throws IOException
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.getExtensionFilters().add(ParameterSavingTools.getExtensionFilter());
      fileChooser.setTitle("Select Parameter File");
      File defaultFilePath = getDefaultFilePath();
      if (defaultFilePath != null)
      {
         fileChooser.setInitialDirectory(defaultFilePath);
      }
      File file = fileChooser.showSaveDialog(saveAs.getScene().getWindow());

      if (file != null)
      {
         setActiveFile(file);
         handleSave(event);
      }
   }

   /**
    * Returns the file that was last opened or saved to.
    *
    * @return the most-recently-used file.
    */
   public static File getDefaultFilePath()
   {
      Preferences prefs = Preferences.userNodeForPackage(ParameterSavingNode.class);
      String filePath = prefs.get("filePath", null);

      if (filePath != null && Files.isDirectory(Paths.get(filePath)))
         return new File(filePath);
      else
         return null;
   }

   /**
    * Stores the given file's path as the most-recently-used path. The path is persisted across program runs.
    *
    * @param file the file
    */
   private void setDefaultFilePath(File file)
   {
      Preferences prefs = Preferences.userNodeForPackage(ParameterSavingNode.class);
      if (file != null)
      {
         if (!file.isDirectory())
            file = file.getParentFile();

         prefs.put("filePath", file.getAbsolutePath());
      }
   }
}
