package us.ihmc.behaviors.sharedControl;

import us.ihmc.promp.ProMPUtil;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;

public class ListOfFilesTest
{
   @Test
   public void testListOfFiles()
   {
      // try-catch block to handle exceptions
      try {
         String demoDirAbs = ProMPUtil.getDemosDirectory().toString();
         String demoTrainingDirAbs = demoDirAbs + "/ReachHandle";
         System.out.println("Folder is: " + demoTrainingDirAbs);
         // Create a file object
         File demoFolder = new File(demoTrainingDirAbs);

         // Get all the names of the .csv files present
         // in the given directory
         File[] files = demoFolder.listFiles((dir, name) -> name.toLowerCase().endsWith(".csv") && !new File(dir, name).isDirectory());

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
