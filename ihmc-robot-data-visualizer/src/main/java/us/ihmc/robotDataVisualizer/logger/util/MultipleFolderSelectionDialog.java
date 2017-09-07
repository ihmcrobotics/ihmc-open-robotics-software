package us.ihmc.robotDataVisualizer.logger.util;

import java.io.File;

import javax.swing.JFileChooser;

public class MultipleFolderSelectionDialog
{
   public static File[] getDirectories()
   {
      final JFileChooser folderDialog = new JFileChooser("Choose logging directories");
      folderDialog.setMultiSelectionEnabled(true);
      folderDialog.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
      
      int selection = folderDialog.showOpenDialog(null);
      if (selection != JFileChooser.APPROVE_OPTION) {
          return null;
      }
      
      File[] folders = folderDialog.getSelectedFiles();
      return folders;
   }
}
