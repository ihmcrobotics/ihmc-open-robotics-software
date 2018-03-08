package us.ihmc.parameterTuner.guiElements.tabs;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.prefs.Preferences;

import org.apache.commons.lang3.StringUtils;

import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.TabPane;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Window;
import us.ihmc.parameterTuner.guiElements.GuiElement;
import us.ihmc.parameterTuner.guiElements.main.ParameterSavingNode;
import us.ihmc.parameterTuner.guiElements.tuners.Tuner;

public class TabSavingTools
{
   public static void saveTab(TuningTab tab, Window window)
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.getExtensionFilters().add(getExtensionFilter());
      fileChooser.setTitle("Save Tab");
      File defaultFilePath = getDefaultFilePath();
      if (defaultFilePath != null)
      {
         fileChooser.setInitialDirectory(defaultFilePath);
      }
      fileChooser.setInitialFileName(tab.getName() + ".tab");

      File file = fileChooser.showSaveDialog(window);
      if (file != null)
      {
         setDefaultFilePath(file);
         saveTab(tab, file);
      }
   }

   private static void saveTab(TuningTab tab, File file)
   {
      try
      {
         FileWriter writer = new FileWriter(file);
         writer.write(tab.getName() + "\n");
         List<String> parameters = tab.getParameterUniqueNames();
         for (String name : parameters)
         {
            writer.write(name + "\n");
         }
         writer.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public static void loadTab(TabPane tabPane, Map<String, Tuner> tunerMap, Window window)
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.getExtensionFilters().add(getExtensionFilter());
      fileChooser.setTitle("Load Tab");
      File defaultFilePath = getDefaultFilePath();
      if (defaultFilePath != null)
      {
         fileChooser.setInitialDirectory(defaultFilePath);
      }

      File file = fileChooser.showOpenDialog(window);
      if (file != null)
      {
         setDefaultFilePath(file);
         loadTab(tabPane, tunerMap, file);
      }
   }

   private static void loadTab(TabPane tabPane, Map<String, Tuner> tunerMap, File file)
   {
      try
      {
         FileReader fileReader = new FileReader(file);
         BufferedReader reader = new BufferedReader(fileReader);

         String name = reader.readLine();
         if (name == null)
         {
            reader.close();
            return;
         }
         TuningTab newTab = new TuningTab(name, tabPane);
         newTab.setTunerMap(tunerMap);

         String parameter = reader.readLine();
         List<String> namesNotFound = new ArrayList<>();
         while (parameter != null)
         {
            if (tunerMap.containsKey(parameter))
            {
               newTab.handleNewParameter(parameter);
            }
            else
            {
               namesNotFound.add(parameter);
            }
            parameter = reader.readLine();
         }
         reader.close();

         if (!namesNotFound.isEmpty())
         {
            showParametersNotFound(namesNotFound);
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private static void showParametersNotFound(List<String> namesNotFound)
   {
      String names = "";
      for (String name : namesNotFound)
      {
         String[] splitNamespace = StringUtils.split(name, GuiElement.SEPERATOR);
         String simpleName = splitNamespace[splitNamespace.length - 1];
         names += simpleName + "\n";
      }

      Alert alert = new Alert(AlertType.INFORMATION);
      alert.setTitle("Missing Parameters");
      alert.setHeaderText("Unable to locate parameters:");
      alert.setContentText(names);
      alert.showAndWait();
   }

   private static ExtensionFilter getExtensionFilter()
   {
      return new FileChooser.ExtensionFilter("Tab Files (*.tab)", "*.tab");
   }

   private static File getDefaultFilePath()
   {
      Preferences prefs = Preferences.userNodeForPackage(ParameterSavingNode.class);
      String filePath = prefs.get("tuningTabConfigurationPath", null);

      if (filePath != null && Files.isDirectory(Paths.get(filePath)))
         return new File(filePath);
      else
         return null;
   }

   private static void setDefaultFilePath(File file)
   {
      Preferences prefs = Preferences.userNodeForPackage(ParameterSavingNode.class);
      if (file != null)
      {
         if (!file.isDirectory())
            file = file.getParentFile();

         prefs.put("tuningTabConfigurationPath", file.getAbsolutePath());
      }
   }
}
