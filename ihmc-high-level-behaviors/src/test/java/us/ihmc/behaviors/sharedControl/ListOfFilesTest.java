package us.ihmc.behaviors.sharedControl;

import us.ihmc.tools.io.WorkspaceDirectory;
import org.junit.Test;
import static org.junit.Assert.*;

import java.io.File;

public class ListOfFilesTest
{
   @Test
   public void testListOfFiles()
   {
      // try-catch block to handle exceptions
      try {
         WorkspaceDirectory demoDir = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc/demos");
         String demoDirAbs = demoDir.getFilesystemDirectory().toAbsolutePath().toString();
         String demoTrainingDirAbs = demoDirAbs + "/PushDoor";
         System.out.println("Folder is: " + demoTrainingDirAbs);
         // Create a file object
         File demoFolder = new File(demoTrainingDirAbs);

         // Get all the names of the files present
         // in the given directory
         File[] files = demoFolder.listFiles();

         System.out.println("Files are:");
         assertTrue(files.length>0);
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
