package us.ihmc.parameterTuner.guiElements.main;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

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
   private GuiRegistry registry;

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

   public File getActiveFile()
   {
      return activeFile;
   }

   public void setRegistry(GuiRegistry registry)
   {
      this.registry = registry;
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
      // if overwriting a file pop up summary dialog
      boolean isModified = modified.isSelected();
      boolean isMerge = merge.isSelected();
      if (activeFile.exists() && !ParameterSavingTools.confirmSave(isModified, isMerge, activeFile.getName()))
      {
         return;
      }

      GuiRegistry registryAfterModified;
      if (isModified)
      {
         registryAfterModified = ParameterSavingTools.filterModified(registry);
      }
      else
      {
         registryAfterModified = registry;
      }

      GuiRegistry registryAfterMerge;
      if (isMerge && activeFile.exists())
      {
         List<Registry> xmlRegistries = ParameterTuningTools.getParameters(activeFile);
         GuiRegistry existingRegistry = ParameterTuningTools.buildGuiRegistryFromXML(xmlRegistries);
         registryAfterMerge = ParameterSavingTools.merge(existingRegistry, registryAfterModified);
      }
      else
      {
         registryAfterMerge = registryAfterModified;
      }

      List<Registry> xmlRegistries = ParameterTuningTools.buildXMLRegistryFromGui(registryAfterMerge);
      ParameterSavingTools.save(activeFile, xmlRegistries);
      informListeners();
   }

   private void handleSaveAs(ActionEvent event) throws IOException
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.getExtensionFilters().add(ParameterSavingTools.getExtensionFilter());
      fileChooser.setTitle("Select Parameter File");
      if (activeFile != null)
      {
         fileChooser.setInitialDirectory(activeFile.getParentFile());
      }
      File file = fileChooser.showSaveDialog(saveAs.getScene().getWindow());

      if (file != null)
      {
         setActiveFile(file);
         handleSave(event);
      }
   }
}
