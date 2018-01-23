package us.ihmc.robotEnvironmentAwareness.ui.io;

import java.io.File;

import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionDataImporter
{
   public static PlanarRegionsList importUsingFileChooser(Window ownerWindow)
   {
      DirectoryChooser directoryChooser = new DirectoryChooser();
      File initialDirectory = new File("../../Data/PlanarRegion");
      if (!initialDirectory.exists() || !initialDirectory.isDirectory())
         initialDirectory = new File(".");
      directoryChooser.setInitialDirectory(initialDirectory);

      File result = directoryChooser.showDialog(ownerWindow);
      if (result == null)
         return null;
      else
         return PlanarRegionFileTools.importPlanarRegionData(result);
   }

   public static PlanarRegionsList importPlanarRegionData(File dataFolder)
   {
      return PlanarRegionFileTools.importPlanarRegionData(dataFolder);
   }
}
