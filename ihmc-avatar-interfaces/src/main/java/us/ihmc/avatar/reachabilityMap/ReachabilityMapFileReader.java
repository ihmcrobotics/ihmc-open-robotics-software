package us.ihmc.avatar.reachabilityMap;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import us.ihmc.javafx.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;

public interface ReachabilityMapFileReader
{
   Voxel3DGrid read(File fileToLoad, ReachabilityMapRobotInformation robotInformation);

   String getFileType();

   String getFileExtension();

   default File openSelectionFileDialog()
   {
      File initialDirectory = SessionVisualizerIOTools.getDefaultFilePath("humanoid-reachability-map-load");
      if (initialDirectory == null)
         initialDirectory = new File(".");
      File result = openSelectionFileDialog(initialDirectory);
      if (result != null)
         SessionVisualizerIOTools.setDefaultFilePath("humanoid-reachability-map-load", result.getParentFile());
      return result;
   }

   default File openSelectionFileDialog(File initialDirectory)
   {
      JavaFXMissingTools.startup();
      return JavaFXMissingTools.runAndWait(() ->
      {
         FileChooser fileChooser = new FileChooser();
         fileChooser.setTitle("Choose reachability map to load");
         if (initialDirectory != null)
            fileChooser.setInitialDirectory(initialDirectory);
         fileChooser.getExtensionFilters().add(new ExtensionFilter(getFileType(), "*" + getFileExtension()));
         fileChooser.getExtensionFilters().add(new ExtensionFilter("All Files", "*.*"));
         return fileChooser.showOpenDialog(null);
      });
   }

   default File findLatestFile(Class<?> classForFilePath, ReachabilityMapRobotInformation robotInformation)
   {
      File folder = deriveResourcesFolder(classForFilePath);

      if (!folder.exists())
         return null;

      String fileExtension = getFileExtension();
      String robotName = robotInformation.getRobotDefinition().getName();

      List<File> reachabilityMapFiles = new ArrayList<>(Arrays.asList(folder.listFiles(f -> f.getName().endsWith(robotName + fileExtension))));

      if (reachabilityMapFiles.isEmpty())
         return null;

      Collections.sort(reachabilityMapFiles);

      return reachabilityMapFiles.get(reachabilityMapFiles.size() - 1);
   }

   static File deriveResourcesFolder(Class<?> classForFilePath)
   {
      return ReachabilityMapFileWriter.deriveResourcesPath(classForFilePath).toFile();
   }
}
