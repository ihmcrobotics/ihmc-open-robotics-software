package us.ihmc.rdx;

import java.awt.FileDialog;
import java.awt.Frame;
import java.io.File;

public class RDXFileDialogDemo
{
   public static void main(String[] args)
   {
      Frame frame = new Frame();
      FileDialog fileDialog = new FileDialog(frame, "Choose a .txt file", FileDialog.LOAD);
      fileDialog.setFile("*.txt");
      fileDialog.setVisible(true);

      String directory = fileDialog.getDirectory();
      String filename = fileDialog.getFile();

      if (filename != null)
      {
         File selectedFile = new File(directory, filename);
         System.out.println("Selected file: " + selectedFile.getAbsolutePath());
      }
      else
      {
         System.out.println("No file selected");
      }

      frame.dispose();
      System.exit(0);
   }
}
