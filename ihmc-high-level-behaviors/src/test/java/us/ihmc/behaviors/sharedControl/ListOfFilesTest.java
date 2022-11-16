package us.ihmc.behaviors.sharedControl;

import us.ihmc.tools.io.WorkspaceDirectory;

import java.io.File;

public class ListOfFilesTest
{
   public static void main(String args[])
   {
      // try-catch block to handle exceptions
      try {

         WorkspaceDirectory demoDir = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc/demos");
         String demoDirAbs = demoDir.getDirectoryPath().toAbsolutePath().toString();
         String demoTrainingDirAbs = demoDirAbs + "/PushDoor";
         // Create a file object
         File demoFolder = new File(demoTrainingDirAbs);

         // Get all the names of the files present
         // in the given directory
         File[] files = demoFolder.listFiles();

         System.out.println("Files are:");

         // Display the names of the files
         for (int i = 0; i < files.length; i++) {
            System.out.println(files[i].getName());
         }
      }
      catch (Exception e) {
         System.err.println(e.getMessage());
      }
   }
}
