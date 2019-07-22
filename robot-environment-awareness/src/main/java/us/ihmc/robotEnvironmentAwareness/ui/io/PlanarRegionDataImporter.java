package us.ihmc.robotEnvironmentAwareness.ui.io;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.prefs.Preferences;

import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionDataImporter
{
   public static PlanarRegionsList importUsingFileChooser(Window ownerWindow)
   {
      DirectoryChooser directoryChooser = new DirectoryChooser();
      File initialDirectory = getDefaultFilePath();

      if (initialDirectory == null)
      {
         initialDirectory = new File("../../Data/PlanarRegion");
      }

      if ((initialDirectory == null) || !initialDirectory.exists() || !initialDirectory.isDirectory())
         initialDirectory = new File(".");
      directoryChooser.setInitialDirectory(initialDirectory);

      File result = directoryChooser.showDialog(ownerWindow);
      if (result == null)
         return null;
      else
      {
         setDefaultFilePath(result);
         return PlanarRegionFileTools.importPlanarRegionData(result);
      }
   }

   public static PlanarRegionsList importPlanarRegionData(File dataFolder)
   {
      return PlanarRegionFileTools.importPlanarRegionData(dataFolder);
   }

   /**
    * Returns the file that was last opened or saved to.
    *
    * @return the most-recently-used file.
    */
   private static File getDefaultFilePath()
   {
      Preferences prefs = Preferences.userNodeForPackage(PlanarRegionsList.class);
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
   private static void setDefaultFilePath(File file)
   {
      Preferences prefs = Preferences.userNodeForPackage(PlanarRegionsList.class);
      if (file != null)
      {
         if (!file.isDirectory())
            file = file.getParentFile();

         prefs.put("filePath", file.getAbsolutePath());
      }
   }
}
