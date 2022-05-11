package us.ihmc.avatar.reachabilityMap;

import java.io.File;

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
      JavaFXMissingTools.startup();
      return JavaFXMissingTools.runAndWait(() ->
      {
         FileChooser fileChooser = new FileChooser();
         fileChooser.setTitle("Choose reachability map to load");
         File initialDirectory = SessionVisualizerIOTools.getDefaultFilePath("humanoid-reachability-map-load");
         if (initialDirectory == null)
            initialDirectory = new File(".");
         fileChooser.setInitialDirectory(initialDirectory);
         fileChooser.getExtensionFilters().add(new ExtensionFilter(getFileType(), "*" + getFileExtension()));
         fileChooser.getExtensionFilters().add(new ExtensionFilter("All Files", "*.*"));
         File result = fileChooser.showOpenDialog(null);
         if (result != null)
            SessionVisualizerIOTools.setDefaultFilePath("humanoid-reachability-map-load", result.getParentFile());
         return result;
      });
   }

}
